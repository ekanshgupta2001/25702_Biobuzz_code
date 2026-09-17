package org.firstinspires.ftc.teamcode.commands;

import com.pedropathing.ivy.Command;

import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.subsystems.Shooter;
import org.firstinspires.ftc.teamcode.subsystems.Storage;
import org.firstinspires.ftc.teamcode.subsystems.Transfer;
import org.firstinspires.ftc.teamcode.util.time.Clock;

import java.util.function.IntSupplier;

/**
 * The shooting cycle as one command: the spin-up wait, then for each piece the storage → transfer
 * → flywheel hand-off and the flywheel's recovery, as a state machine on the robot's clock and
 * sensors.
 *
 * <h2>Why a state machine and not Ivy groups</h2>
 * Ivy's {@code Sequential} hands off one child per loop and never executes the child it has just
 * started, so every child boundary costs a whole loop, an {@code instant} costs one by itself,
 * and nested groups compound. The group tree this replaces was nine levels deep and spent seven
 * idle loops between "flywheel recovered" and "next pulse commanded": about 35 loops, 0.7 s at
 * 20 ms, on a four-piece run. Here every zero-duration transition chains inside one
 * {@code execute()}, so the next state's motor intent is set in the same loop the previous state
 * ended, and the first pulse is commanded in the loop the cycle starts.
 *
 * <p>Requires the storage and the transfer only. The shooter is held by the enclosing macro
 * ({@code Macros.heldFlywheel()}) from before this starts to after it ends, and the drivetrain is
 * never touched, so driving and the aim lock continue through a shot. The cycle calls the
 * subsystems' intent setters ({@code advance()}, {@code feed()}, {@code stop()}) under that
 * requirement; nothing here touches hardware.
 *
 * <h2>Sensors, or not</h2>
 * With an exit sensor the storage advances until the exit edge (or {@code ADVANCE_TIMEOUT_MS}); a
 * piece already in the lift is not advanced again. With a feed sensor the transfer lifts until a
 * piece is staged and feeds until the sensor clears plus a dwell. Without an exit sensor the
 * storage runs a timed pulse <em>beside</em> the transfer leg, started in the same loop, so the
 * side wheels never push the queue into a stopped transfer (fixthese B2); without a feed sensor
 * the transfer leg is one {@code Macros.SENSORLESS_FEED_PULSE_MS} pulse. A shot is counted only
 * when the sensors that exist agree, and without an exit sensor the storage count is dead-reckoned
 * down so the intake interlock releases. The flywheel's recovery is waited for only between
 * pieces; after the last only {@code SHOT_RECOVERY_MIN_MS}, so the wheel is never handed back
 * with the piece still in it and no time is spent on a recovery nobody needs.
 */
final class ShootCycle {
    enum State { SPIN_UP, ADVANCE, LIFT, FEED, RECOVER, DONE }

    private final Storage storage;
    private final Transfer transfer;
    private final Shooter shooter;
    private final Clock clock;
    private final IntSupplier pieces;
    private final Runnable onShot;

    private State state = State.DONE;
    private long stateStartedAt = 0;
    private long feedClearedAt = -1;
    /** The sensorless storage pulse's end, or -1 while none runs. It runs beside the state. */
    private long pulseEndsAt = -1;
    private int toFire = 0;
    private int attempts = 0;
    private int exitsAtStart = 0;
    private boolean preloaded = false;
    private boolean staged = false;
    private boolean transferDone = false;

    private ShootCycle(Robot robot, IntSupplier pieces, Runnable onShot) {
        this.storage = robot.storage;
        this.transfer = robot.transfer;
        this.shooter = robot.shooter;
        this.clock = robot.getClock();
        this.pieces = pieces;
        this.onShot = onShot;
    }

    /**
     * @param pieces read when the command starts: how many pieces to fire (the macro snapshots
     *               {@code piecesOnBoard()} in its leading instant)
     * @param onShot run once per counted shot
     */
    static Command command(Robot robot, IntSupplier pieces, Runnable onShot) {
        ShootCycle cycle = new ShootCycle(robot, pieces, onShot);
        return Command.build()
                .setStart(cycle::begin)
                .setExecute(cycle::step)
                .setDone(() -> cycle.state == State.DONE)
                .setEnd(ec -> cycle.end())
                .requiring(robot.storage, robot.transfer);
    }

    /** Resets everything, so the same command can be scheduled again, and takes the first step. */
    private void begin() {
        toFire = pieces.getAsInt();
        attempts = 0;
        pulseEndsAt = -1;
        transferDone = false;
        if (toFire <= 0) {
            state = State.DONE;
            return;
        }
        enter(State.SPIN_UP, clock.nowMs());
        step();     // an already-latched wheel feeds in this very loop
    }

    /** Idempotent, and valid before {@link #begin()}: a group may end a child it never started. */
    private void end() {
        storage.stop();
        transfer.stop();
        pulseEndsAt = -1;
        state = State.DONE;
    }

    private void step() {
        long now = clock.nowMs();
        if (pulseEndsAt >= 0 && now >= pulseEndsAt) {
            storage.stop();
            pulseEndsAt = -1;
        }
        // Each true return entered a new state; keep going until one has to wait on the clock or
        // a sensor. Bounded: attempts strictly increases and every per-piece path waits somewhere
        // unless that subsystem is unavailable.
        while (advance(now)) {
            // intentionally empty
        }
    }

    /** Runs the current state's exit test and, when it passes, the next state's entry action. */
    private boolean advance(long now) {
        long elapsed = now - stateStartedAt;
        switch (state) {
            case SPIN_UP:
                // waitForSpeedCommand's condition: at speed, or the timeout, or no shooter at all.
                if (!shooter.isAvailable() || shooter.atSpeed() || elapsed >= Shooter.SPINUP_TIMEOUT_MS) {
                    return nextPiece(now);
                }
                return false;
            case ADVANCE:
                if (storage.getExitEvents() > exitsAtStart || elapsed >= Storage.ADVANCE_TIMEOUT_MS) {
                    storage.stop();
                    return enterTransfer(now);
                }
                return false;
            case LIFT:
                if (transfer.pieceAtFeed() || elapsed >= Transfer.LIFT_TIMEOUT_MS) {
                    staged = transfer.pieceAtFeed();
                    return enterFeed(now);
                }
                return false;
            case FEED:
                if (!transferDone && feedFinished(now, elapsed)) {
                    transfer.stop();
                    transferDone = true;
                }
                if (!transferDone || pulseEndsAt >= 0) return false;    // both legs must have ended
                if (pieceWasShot()) {
                    onShot.run();
                    if (!storage.hasExitSensor()) storage.markExited();
                }
                return enter(State.RECOVER, now);
            case RECOVER: {
                boolean more = attempts < toFire;
                boolean over = !shooter.isAvailable()
                        || (elapsed >= Shooter.SHOT_RECOVERY_MIN_MS
                            && (!more || shooter.atSpeed()
                                || elapsed >= Shooter.SHOT_RECOVERY_MIN_MS + Shooter.SHOT_RECOVERY_TIMEOUT_MS));
                if (!over) return false;
                return more ? nextPiece(now) : enter(State.DONE, now);
            }
            default:
                return false;
        }
    }

    /** One more piece: snapshot the per-shot flags, then the storage leg (sensed or timed). */
    private boolean nextPiece(long now) {
        attempts++;
        exitsAtStart = storage.getExitEvents();
        preloaded = transfer.hasPieceInLift() || transfer.pieceAtFeed();
        transferDone = false;
        if (storage.hasExitSensor()) {
            if (preloaded || !storage.hasPiece() || !storage.isAvailable()) return enterTransfer(now);
            storage.advance();
            return enter(State.ADVANCE, now);
        }
        // No exit edge to end on: a timed pulse, run beside the transfer leg from this same loop.
        if (storage.isAvailable()) {
            storage.advance();
            pulseEndsAt = now + Macros.SENSORLESS_FEED_PULSE_MS;
        }
        return enterTransfer(now);
    }

    /** The transfer leg: lift until staged where a feed sensor exists, else straight to the feed. */
    private boolean enterTransfer(long now) {
        // Without a feed sensor the timed pulse is taken to have staged the piece; with one, the
        // lift records what the sensor saw. A piece already staged is never lifted onto (double-feed).
        staged = !transfer.hasFeedSensor() || transfer.pieceAtFeed();
        if (!transfer.hasFeedSensor() || !transfer.isAvailable() || transfer.pieceAtFeed()) {
            return enterFeed(now);
        }
        transfer.liftPiece();
        return enter(State.LIFT, now);
    }

    /** The feed: sensed (until clear plus dwell) or a timed pulse; skipped when nothing is staged. */
    private boolean enterFeed(long now) {
        feedClearedAt = -1;
        boolean skip = !transfer.isAvailable() || (transfer.hasFeedSensor() && !transfer.pieceAtFeed());
        if (!skip) transfer.feed();
        transferDone = skip;
        return enter(State.FEED, now);
    }

    private boolean feedFinished(long now, long elapsed) {
        if (!transfer.hasFeedSensor()) return elapsed >= Macros.SENSORLESS_FEED_PULSE_MS;
        if (feedClearedAt < 0 && !transfer.pieceAtFeed()) feedClearedAt = now;
        return (feedClearedAt >= 0 && now - feedClearedAt >= Transfer.FEED_CLEAR_DWELL_MS)
                || elapsed >= Transfer.FEED_TIMEOUT_MS;
    }

    /** The shot counts only when the sensors that exist agree (a pulse count without them). */
    private boolean pieceWasShot() {
        boolean left = !storage.hasExitSensor() || preloaded || storage.getExitEvents() > exitsAtStart;
        boolean fed = !transfer.hasFeedSensor() || (staged && !transfer.pieceAtFeed());
        return left && fed;
    }

    private boolean enter(State next, long now) {
        state = next;
        stateStartedAt = now;
        return true;
    }
}
