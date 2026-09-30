package org.firstinspires.ftc.teamcode.commands;

import com.pedropathing.ivy.Command;

import org.firstinspires.ftc.teamcode.subsystems.Intake;
import org.firstinspires.ftc.teamcode.subsystems.Shooter;

/**
 * The shooting cycle: spin up, then pulse the tunnel once per piece, waiting for the flywheel to
 * recover in between.
 *
 * <h2>Why this is one hand-written command and not an Ivy group tree</h2>
 * Ivy's {@code Sequential} hands off one child per {@code execute()} and never executes the child it
 * has just started, so <em>every child boundary costs a whole 20 ms loop</em> and an {@code instant}
 * costs one by itself. The previous nine-level tree spent 7 idle loops between "flywheel recovered"
 * and "next pulse commanded", about 0.7 s across a four-piece run. A state machine chains every
 * zero-duration transition inside a single {@code execute()}, so the next pulse is commanded in the
 * same loop the flywheel comes back. It also sidesteps the {@code Repeat.end()} NPE
 * (docs/01 B.5 trap 8) for free.
 *
 * <h2>Feeding is running the intake</h2>
 * One motor drives the roller and the tunnel, and there is no gate and no sensor anywhere in the
 * path. A piece reaches the flywheel because the tunnel ran long enough to carry it there, so a
 * "shot" is a {@link #FEED_PULSE_MS} pulse of the intake. That makes {@link #getShotsFired()}
 * honestly a count of pulses, not of pieces — nothing on this robot can know the difference, and the
 * telemetry says so.
 *
 * <h2>What it requires</h2>
 * The intake only. The flywheel is held by {@code Shooter.armedCommand()}, passed to
 * {@code Macros.reporting} as an alongside child, so an armed wheel is never told to stop when a
 * shot starts, and the drivetrain is left alone so the driver keeps translating and aiming through
 * a shot.
 */
public class Shoot {
    /**
     * How long the tunnel runs to carry one piece into the flywheel. Measure with
     * {@code Bench: Intake}: too short leaves the piece short of the wheel, too long feeds two.
     */
    public static long FEED_PULSE_MS = 400;
    /** Quiet time after a pulse before the recovery check, so the dip has actually started. */
    public static long SETTLE_AFTER_PULSE_MS = 80;
    /** A flywheel that has not recovered by then gets the next piece anyway. */
    public static long RECOVER_TIMEOUT_MS = 900;
    /** Spin-up deadline. Past this the cycle gives up rather than holding the intake all match. */
    public static long SPIN_UP_TIMEOUT_MS = 3000;
    /** After the last pulse, long enough for the piece to leave before the wheel is released. */
    public static long FINAL_DWELL_MS = 150;

    private enum State { SPIN_UP, FEED, SETTLE, RECOVER, DWELL, DONE }

    private final Intake intake;
    private final Shooter shooter;
    private final int pieces;

    private State state = State.DONE;
    private long stateSinceMs = 0;
    private int fired = 0;

    public Shoot(Intake intake, Shooter shooter, int pieces) {
        this.intake = intake;
        this.shooter = shooter;
        this.pieces = Math.max(0, pieces);
    }

    public int getShotsFired() {
        return fired;
    }

    /** True once every requested pulse has run (as opposed to having been cut short). */
    public boolean completed() {
        return fired >= pieces;
    }

    /**
     * The command. Requires the intake for the whole cycle so the default idle command cannot
     * re-assert "stopped" between pulses.
     */
    public Command build() {
        return Command.build()
                .setStart(this::start)
                .setExecute(this::step)
                .setDone(() -> state == State.DONE)
                .setEnd(ec -> {
                    intake.stop();
                    state = State.DONE;
                })
                .requiring(intake);
    }

    private void start() {
        fired = 0;
        // A zero target would make atTarget() true on the first tick (|0 - 0| < tolerance) and the
        // tunnel would push pieces into a dead flywheel. The caller sets the speed; if it did not,
        // this cycle does nothing rather than dumping the load on the floor.
        if (pieces == 0 || shooter.getTarget() <= 0) {
            state = State.DONE;
            return;
        }
        enter(State.SPIN_UP);
    }

    private void enter(State next) {
        state = next;
        stateSinceMs = System.currentTimeMillis();
        // The action for a state is applied on entry, so a transition costs no loop.
        if (next == State.FEED) intake.in();
        else intake.stop();
    }

    private long elapsed() {
        return System.currentTimeMillis() - stateSinceMs;
    }

    /**
     * One pass of the machine. Every transition is chained in this same call, so a state whose exit
     * condition is already true does not cost a loop.
     */
    private void step() {
        // Bounded so a single call cannot spin forever if two states ever transition into each other.
        for (int guard = 0; guard < State.values().length + 1; guard++) {
            switch (state) {
                case SPIN_UP:
                    if (shooter.atTarget() || elapsed() >= SPIN_UP_TIMEOUT_MS) {
                        enter(State.FEED);
                        continue;
                    }
                    return;
                case FEED:
                    if (elapsed() >= FEED_PULSE_MS) {
                        fired++;
                        enter(fired >= pieces ? State.DWELL : State.SETTLE);
                        continue;
                    }
                    return;
                case SETTLE:
                    if (elapsed() >= SETTLE_AFTER_PULSE_MS) {
                        enter(State.RECOVER);
                        continue;
                    }
                    return;
                case RECOVER:
                    if (shooter.atTarget() || elapsed() >= RECOVER_TIMEOUT_MS) {
                        enter(State.FEED);
                        continue;
                    }
                    return;
                case DWELL:
                    if (elapsed() >= FINAL_DWELL_MS) {
                        enter(State.DONE);
                        continue;
                    }
                    return;
                default:
                    return;
            }
        }
    }
}
