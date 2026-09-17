package org.firstinspires.ftc.teamcode.commands;

import static com.pedropathing.ivy.commands.Commands.conditional;
import static com.pedropathing.ivy.commands.Commands.instant;
import static com.pedropathing.ivy.commands.Commands.lazy;
import static com.pedropathing.ivy.commands.Commands.onInterrupt;
import static com.pedropathing.ivy.commands.Commands.waitUntil;
import static com.pedropathing.ivy.groups.Groups.deadline;
import static com.pedropathing.ivy.groups.Groups.parallel;
import static com.pedropathing.ivy.groups.Groups.race;
import static com.pedropathing.ivy.groups.Groups.sequential;

import com.pedropathing.api.Paths;
import com.pedropathing.ivy.Command;
import com.pedropathing.math.Pose;
import com.pedropathing.paths.Path;

import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.subsystems.Drivetrain;
import org.firstinspires.ftc.teamcode.subsystems.Limelight;
import org.firstinspires.ftc.teamcode.subsystems.Shooter;
import org.firstinspires.ftc.teamcode.subsystems.Storage;
import org.firstinspires.ftc.teamcode.util.math.Angles;

import java.util.Locale;
import java.util.function.BooleanSupplier;
import java.util.function.Supplier;

/**
 * Multi-subsystem, one-button actions composed from the subsystems' own command factories.
 *
 * <p>Every macro follows four rules (docs/03 section 9; the Guide's lessons):
 * <ol>
 *   <li><b>Bounded.</b> Every wait is raced against a timeout on the robot's {@code Clock}
 *       ({@link Waits}), so a macro can never hold its subsystems for the rest of the match.</li>
 *   <li><b>Reports an outcome, always.</b> Every macro is built through {@link #reporting}:
 *       {@code begin(name)} at the start, the outcome measured after the fact against a snapshot
 *       taken at the start (a perfect drive that collected nothing is a failure), and if the group is
 *       ended from outside before it gets that far (a {@link Waits#bounded} race, an operator abort,
 *       {@code Scheduler.cancel}) the outcome is {@link Outcome#CANCELLED}, never left
 *       {@link Outcome#RUNNING}.</li>
 *   <li><b>Declares its resources through what it composes.</b> Requirements come from the
 *       subsystem commands inside the group, so scheduling a macro suspends the default commands
 *       it needs and ending it restores them. Nothing here touches hardware directly.</li>
 *   <li><b>Builds vision paths lazily.</b> Paths are built inside {@code followLazyCommand}
 *       suppliers at command start, never stored.</li>
 * </ol>
 *
 * <p>Nothing here names a field location or a game piece: the season-specific targets (which HIVE
 * CELL, which AprilTag IDs) are passed in by the OpMode from {@code game/}.
 *
 * <p><b>Aiming is the drivetrain's job on V1.</b> The shooter is fixed and fires out the rear
 * ({@code Shooter.HEADING_OFFSET_RAD}), so {@link #aimHeading} is the one law that turns a target
 * into a field heading, refined by a visible tag. {@link #aimAt} is the bounded one-shot turn;
 * Teleop feeds the same law into {@code Drivetrain.setAimLock} so the driver can hold an aim while
 * translating. {@link #shootOne()} and {@link #shootAll()} never require the drivetrain, so driving
 * and aiming continue through a shot; {@link #aimAndShootAll} does both in sequence for autonomous.
 *
 * <p><b>Shooting without sensors</b> (the first-event robot) is metered by time: for each piece the
 * storage transport and the transfer run <em>together</em> for {@link #SENSORLESS_FEED_PULSE_MS},
 * then the macro waits for the flywheel to recover before the next piece. The side wheels never push
 * the queue into a stopped transfer, and {@link #getShotsFired()} is honestly a pulse count.
 *
 * <p><b>Without a storage-entrance sensor the count is unknowable in teleop</b> (it only rises on
 * that sensor's edge or through {@code setCount}), so {@link #piecesOnBoard()} treats a zero count as
 * "assume full" ({@link #ASSUME_FULL_WHEN_UNCOUNTED}): Shoot One always fires one pulse and Shoot
 * All fires {@code Storage.CAPACITY} of them (the operator cancels early with the stop button, or
 * sets the count by hand). A robot that refuses to shoot because it cannot count is useless at an
 * event; a spare pulse on an empty channel costs nothing.
 */
public class Macros {
    // ---- Timeouts and tolerances (plain statics; edit and redeploy) ----
    public static long INTAKE_TIMEOUT_MS = 8000;
    /**
     * Run the storage transport while intaking so pieces move rearward and make room. Off by
     * default: with the transfer stopped, the side wheels would push every stored piece against it
     * for the whole intake (fixthese C8). Flip it only once the mechanism proves it needs the help.
     */
    public static boolean INTAKE_RUNS_STORAGE = false;
    /**
     * A game decision, named so it can be found: when nothing can count (no trusted entrance
     * sensor and nothing set the count), treat the robot as holding {@code Storage.CAPACITY}
     * pieces, so Shoot One fires one pulse and Shoot All fires four. {@code false} makes an unknown
     * count NO_TARGET: a robot that cannot count refuses to shoot. Shown on the teleop card as
     * "assumes 4" (fixthese R2-B2).
     */
    public static boolean ASSUME_FULL_WHEN_UNCOUNTED = true;
    public static long SHOOT_ONE_TIMEOUT_MS = 6000;
    public static long SHOOT_ALL_TIMEOUT_MS = 20000;
    /**
     * Per piece, without an exit or feed sensor: how long the storage transport and the transfer
     * run together to move one piece into the flywheel. Measure with {@code Bench: Storage} and
     * {@code Bench: Transfer}; too short leaves the piece in the lift, too long feeds two.
     */
    public static long SENSORLESS_FEED_PULSE_MS = 600;
    public static long AIM_TIMEOUT_MS = 2500;
    /** The robot counts as aimed within this of the wanted heading. */
    public static double AIM_TOLERANCE_DEGREES = 2.0;
    /** A one-shot aim re-issues Pedro's hold when the wanted heading moves by more than this. */
    public static double AIM_REISSUE_DEGREES = 1.0;
    /** A seen piece closer than this to the crosshair counts as aligned. */
    public static double ALIGN_TOLERANCE_DEGREES = 1.5;
    public static long PIPELINE_WARMUP_MS = 250;
    public static long SEARCH_TIMEOUT_MS = 2000;
    public static long APPROACH_TIMEOUT_MS = 4000;
    public static long ALIGN_TIMEOUT_MS = 1500;
    public static long RELOCALIZE_TIMEOUT_MS = 1500;
    public static long SNAP_TIMEOUT_MS = 1500;
    public static double SNAP_TOLERANCE_DEGREES = 3.0;
    public static long DRIVE_TO_TIMEOUT_MS = 6000;
    /** A drive-to counts as arrived within this of the target on both axes. */
    public static double DRIVE_TO_TOLERANCE_INCHES = 3.0;
    /**
     * Shorter than this and no path is built: a line from the current pose to (almost) itself is
     * degenerate inside Pedro (a zero tangent, NaN powers that the motor layer drops on the floor),
     * so {@code followLazyCommand} is handed a null path and finishes at once, and the macro's own
     * position check reports the outcome.
     */
    public static double MIN_PATH_INCHES = 0.5;

    /** How the last macro finished. */
    public enum Outcome { IDLE, RUNNING, SUCCESS, TIMED_OUT, NO_TARGET, CANCELLED }

    private final Robot robot;
    private Outcome outcome = Outcome.IDLE;
    private String activeName = "idle";
    private int shotsFired = 0;

    public Macros(Robot robot) {
        this.robot = robot;
    }

    // ---- State ----

    public Outcome getOutcome() {
        return outcome;
    }

    /** The running macro's name, or {@code "idle"}. */
    public String getActiveName() {
        return activeName;
    }

    public boolean isRunning() {
        return outcome == Outcome.RUNNING;
    }

    /** Pieces the last shooting macro actually fired (per the sensors, or the pulse count). */
    public int getShotsFired() {
        return shotsFired;
    }

    /** Human-readable one-liner for telemetry. */
    public String getStatus() {
        return activeName + " : " + outcome;
    }

    /**
     * Marks a running macro CANCELLED. Every macro calls this itself when its group is ended from
     * outside ({@link #reporting}); {@code Robot.abortMacro()} calls it too, harmlessly, after the
     * follower and pipeline cleanup that Ivy cannot do.
     */
    public void markCancelled() {
        // Both inside the guard: a late callback from an already-finished macro must not blank the
        // name of one that is running.
        if (outcome == Outcome.RUNNING) {
            outcome = Outcome.CANCELLED;
            activeName = "idle";
        }
    }

    /**
     * Pieces currently in the robot: the storage queue plus one in the lift, if sensed. Without a
     * trusted storage-entrance sensor a zero count is "unknown", not "empty", and this returns
     * {@code Storage.CAPACITY} so the shooting macros fire blind, if
     * {@link #ASSUME_FULL_WHEN_UNCOUNTED} (see the class doc).
     */
    public int piecesOnBoard() {
        int known = robot.storage.count() + (robot.transfer.hasPieceInLift() ? 1 : 0);
        if (known == 0 && !isCountKnown() && ASSUME_FULL_WHEN_UNCOUNTED) return Storage.CAPACITY;
        return known;
    }

    /** False when the count is a guess: no entrance sensor and nothing set the count this OpMode. */
    public boolean isCountKnown() {
        return robot.storage.hasEntranceSensor() || robot.storage.count() > 0
                || robot.transfer.hasPieceInLift();
    }

    // ---- Plumbing shared by every macro ----

    /**
     * How every macro is built: {@code begin(name)}, the body, then the outcome. If the group is
     * ended before {@code finishWith} runs (a {@link Waits#bounded} timeout, {@code Scheduler.cancel},
     * an operator abort), {@code onInterrupt} marks it CANCELLED, so no caller has to remember to.
     *
     * <p>{@code deadline(body, onInterrupt(...))}, not {@code setEnd} on the sequential: Ivy's groups
     * are {@code CommandBuilder}s whose {@code setEnd} <em>replaces</em> the group's own end, the one
     * that ends its children (docs/01 B.5 trap 12). A natural finish costs one extra tick and fires
     * the callback too, which is harmless: a terminal outcome is never changed.
     *
     * <p>{@code alongside} commands run in parallel with the whole macro and are ended with it.
     * They start inside {@code Scheduler.schedule()}, before the first loop, which is what lets
     * {@link #heldFlywheel()} take the shooter over from a displaced hold with no gap.
     */
    private Command reporting(String name, Command body, Supplier<Outcome> result, Command... alongside) {
        Command[] children = new Command[alongside.length + 1];
        children[0] = onInterrupt(this::markCancelled);
        System.arraycopy(alongside, 0, children, 1, alongside.length);
        return deadline(sequential(begin(name), body, finishWith(result)), children);
    }

    private Command reporting(String name, Command body, Outcome success, Outcome failure, BooleanSupplier ok) {
        return reporting(name, body, () -> ok.getAsBoolean() ? success : failure);
    }

    private Command begin(String name) {
        return instant(() -> {
            activeName = name;
            outcome = Outcome.RUNNING;
        });
    }

    private Command finishWith(Supplier<Outcome> result) {
        return instant(() -> {
            outcome = result.get();
            activeName = "idle";
        });
    }

    private Command waitMs(long ms) {
        return Waits.waitMs(robot.getClock(), ms);
    }

    private Command bounded(Command work, long timeoutMs) {
        return Waits.bounded(robot.getClock(), work, timeoutMs);
    }

    private static Command noop() {
        return instant(() -> { });
    }

    /**
     * The flywheel hold that rides alongside a shooting macro from its first tick to its last. It
     * starts inside {@code Scheduler.schedule()}, right after the hold it displaces (Teleop's armed
     * flywheel, ended by Ivy's OVERRIDE with an {@code idle()}), so the wheel never sees a zero
     * target between the two holds and {@code Shooter.atSpeed()}'s latch is not reset. Before this
     * the macro's own hold started two hand-off ticks later and every armed shot began with one
     * loop of {@code setVelocity(0)} (review of 2026-09-16). With nothing on board it is a no-op:
     * NO_TARGET never touches the shooter.
     */
    private Command heldFlywheel() {
        return conditional(() -> piecesOnBoard() > 0, robot.shooter.holdSpeedCommand(), noop());
    }

    // ---- Collecting ----

    /**
     * Runs the intake (and, if {@link #INTAKE_RUNS_STORAGE}, the storage transport) until the storage
     * reports full or {@link #INTAKE_TIMEOUT_MS} passes. NO_TARGET when the storage was already
     * full; SUCCESS when it became full or the count rose. On a robot with no way to know it is full
     * (no entrance sensor, no full sensor: {@code Storage.canDetectFull()}) the timer ending
     * <em>is</em> the job, so that is SUCCESS too, under the name {@code "intake (timed)"} so the
     * card says what it did; the operator then sets the count by hand. Only a robot that could have
     * seen a piece and saw none reports TIMED_OUT (fixthese R2-A4).
     */
    public Command intakeUntilFull() {
        final int[] countAtStart = new int[1];
        final boolean[] alreadyFull = new boolean[1];
        final boolean canDetectFull = robot.storage.canDetectFull();
        Command transport = INTAKE_RUNS_STORAGE
                ? robot.storage.advanceUntilCommand(robot.storage::isFull, INTAKE_TIMEOUT_MS)
                : noop();
        Command work = race(
                parallel(robot.intake.intakeCommand(), transport),
                waitUntil(robot.storage::isFull),
                waitMs(INTAKE_TIMEOUT_MS));
        return reporting(canDetectFull ? "intake" : "intake (timed)",
                sequential(
                        instant(() -> {
                            countAtStart[0] = robot.storage.count();
                            alreadyFull[0] = robot.storage.isFull();
                        }),
                        conditional(() -> !alreadyFull[0], work, noop())),
                () -> {
                    if (alreadyFull[0]) return Outcome.NO_TARGET;
                    if (robot.storage.count() > countAtStart[0] || robot.storage.isFull()) return Outcome.SUCCESS;
                    return canDetectFull ? Outcome.TIMED_OUT : Outcome.SUCCESS;
                });
    }

    /**
     * Blob pipeline, wait for a stable detection, drive to the piece while capturing, then restore
     * the AprilTag pipeline. Success is a <em>new</em> piece: the storage count rose, or the intake
     * captured something it did not already hold. With no camera this times out in
     * {@link #PIPELINE_WARMUP_MS} + {@link #SEARCH_TIMEOUT_MS} without moving.
     */
    public Command collectPiece() {
        final int[] countAtStart = new int[1];
        final boolean[] hadPieceAtStart = new boolean[1];
        BooleanSupplier capturedNew = () -> robot.storage.count() > countAtStart[0]
                || (robot.intake.hasPiece() && !hadPieceAtStart[0]);
        return reporting("collect",
                sequential(
                        instant(() -> {
                            countAtStart[0] = robot.storage.count();
                            hadPieceAtStart[0] = robot.intake.hasPiece();
                        }),
                        instant(robot.limelight::activateBlobPipeline),
                        waitMs(PIPELINE_WARMUP_MS),
                        race(waitUntil(robot.limelight::hasStableBlob), waitMs(SEARCH_TIMEOUT_MS)),
                        // A race, not a deadline: race ends its losers INTERRUPTED, whereas deadline and
                        // parallel forward their own end condition, so a capture command inside a
                        // deadline that finished NATURALLY would be told it captured something.
                        race(
                                robot.drivetrain.followLazyCommand(this::approachPath, false),
                                robot.intake.captureCommand(),
                                waitUntil(capturedNew),
                                waitMs(APPROACH_TIMEOUT_MS)),
                        instant(robot.limelight::activateAprilTagPipeline)),
                Outcome.SUCCESS, Outcome.TIMED_OUT, capturedNew);
    }

    /**
     * Turns in place to face the seen piece (no path: a zero-length line is degenerate). NO_TARGET
     * unless a stable blob ends up within {@link #ALIGN_TOLERANCE_DEGREES} of the crosshair.
     */
    public Command alignToPiece() {
        Command turn = lazy(() -> {
            double heading = headingToBlob();
            return Double.isNaN(heading) ? null : robot.drivetrain.turnToCommand(heading);
        }).requiring(robot.drivetrain);   // lazy contributes no requirements of its own
        return reporting("align",
                sequential(
                        instant(robot.limelight::activateBlobPipeline),
                        waitMs(PIPELINE_WARMUP_MS),
                        race(waitUntil(robot.limelight::hasStableBlob), waitMs(SEARCH_TIMEOUT_MS)),
                        bounded(turn, ALIGN_TIMEOUT_MS),
                        instant(robot.limelight::activateAprilTagPipeline)),
                Outcome.SUCCESS, Outcome.NO_TARGET, this::alignedToBlob);
    }

    // ---- Shooting ----

    /**
     * Spins up, moves one piece storage → transfer → flywheel, and idles the wheel. The shooter is
     * held by {@link #heldFlywheel()} from the macro's first tick to its last, so the default
     * command cannot wind it down under the piece and an already-armed wheel is never told to stop
     * on the way in; the spin-up wait itself requires nothing. NO_TARGET with nothing on board.
     */
    public Command shootOne() {
        final int[] shots = new int[1];
        final boolean[] hadPieces = new boolean[1];
        Command work = bounded(
                sequential(robot.shooter.waitForSpeedCommand(), feedOneCore(shots, () -> false)),
                SHOOT_ONE_TIMEOUT_MS);
        return reporting("shootOne",
                sequential(
                        instant(() -> {
                            shots[0] = 0;
                            shotsFired = 0;
                            hadPieces[0] = piecesOnBoard() > 0;
                        }),
                        conditional(() -> hadPieces[0], work, noop())),
                () -> shots[0] >= 1 ? Outcome.SUCCESS
                        : hadPieces[0] ? Outcome.TIMED_OUT : Outcome.NO_TARGET,
                heldFlywheel());
    }

    /**
     * One spin-up, then every piece on board in turn. SUCCESS only when as many pieces were fired
     * as were on board at the start; {@link #getShotsFired()} reports a partial run.
     */
    public Command shootAll() {
        final int[] shots = new int[1];
        final int[] onBoard = new int[1];
        return reporting("shootAll",
                sequential(
                        instant(() -> {
                            shots[0] = 0;
                            shotsFired = 0;
                            onBoard[0] = piecesOnBoard();
                        }),
                        conditional(() -> onBoard[0] > 0, shootAllCore(shots, onBoard), noop())),
                () -> shootAllOutcome(shots[0], onBoard[0]),
                heldFlywheel());
    }

    /**
     * {@link #aimAt} then {@link #shootAll()} as one bounded, reporting macro: the autonomous
     * "empty the pre-loads into the up-CELL" move. Requires the drivetrain, storage, transfer and
     * shooter for the whole run, so it is autonomous's move, not a teleop button. The flywheel
     * spins up during the aim ({@link #heldFlywheel()}), not after it.
     */
    public Command aimAndShootAll(Pose target, int minTagId, int maxTagId) {
        final int[] shots = new int[1];
        final int[] onBoard = new int[1];
        return reporting("aimShootAll",
                sequential(
                        instant(() -> {
                            shots[0] = 0;
                            shotsFired = 0;
                            onBoard[0] = piecesOnBoard();
                        }),
                        // The aim is bounded like every other wait; if the robot cannot settle
                        // within AIM_TOLERANCE_DEGREES in time it shoots anyway: in autonomous a
                        // piece kept on board scores nothing, a near miss might.
                        conditional(() -> onBoard[0] > 0,
                                sequential(bounded(aimCore(target, minTagId, maxTagId), AIM_TIMEOUT_MS),
                                        shootAllCore(shots, onBoard)),
                                noop())),
                () -> shootAllOutcome(shots[0], onBoard[0]),
                heldFlywheel());
    }

    private Command shootAllCore(int[] shots, int[] onBoard) {
        // Unrolled to CAPACITY guarded steps rather than Ivy's repeat(): a sequential that is
        // interrupted (by the timeout) before it reaches a Repeat child calls end() on the
        // never-started Repeat, which NPEs on its null command list (docs/01 B.5 trap 8).
        // CAPACITY + 1 feed steps: the queue holds CAPACITY and the lift can hold one more.
        final int[] attempts = new int[1];
        final int feedSteps = Storage.CAPACITY + 1;
        Command[] steps = new Command[feedSteps + 1];
        steps[0] = sequential(instant(() -> attempts[0] = 0), robot.shooter.waitForSpeedCommand());
        for (int i = 1; i <= feedSteps; i++) {
            // attempts is bumped before the feed, so "more pieces" is judged after this one.
            steps[i] = conditional(() -> attempts[0] < onBoard[0],
                    sequential(instant(() -> attempts[0]++),
                            feedOneCore(shots, () -> attempts[0] < onBoard[0])),
                    noop());
        }
        // Nothing in here requires the shooter: heldFlywheel() in the enclosing reporting group owns
        // it for the whole macro, so no two siblings ever set its target (fixthese C7).
        return bounded(sequential(steps), SHOOT_ALL_TIMEOUT_MS);
    }

    private static Outcome shootAllOutcome(int shots, int onBoard) {
        if (onBoard == 0) return Outcome.NO_TARGET;
        if (shots == 0) return Outcome.TIMED_OUT;
        return shots >= onBoard ? Outcome.SUCCESS : Outcome.TIMED_OUT;
    }

    /**
     * The storage → transfer → flywheel hand-off for one piece, then the flywheel's recovery;
     * requires storage and transfer. The caller holds the shooter. Every per-shot flag is reset in
     * the leading instant so no step can inherit a previous piece's state.
     *
     * <p>With an exit sensor the advance ends on the edge that says the piece reached the transfer,
     * and the transfer leg follows. Without one there is no edge to end on, so the advance is a
     * timed pulse run <em>together</em> with the transfer leg: the side wheels never push the queue
     * into a stopped transfer for seconds at a time (fixthese B2). Either way the shot is counted
     * only when the sensors that exist agree, and without an exit sensor the storage count is
     * dead-reckoned down by one so the intake interlock releases.
     *
     * @param morePieces read after the hand-off: true when another piece follows, so the flywheel
     *                   recovery is waited for; false after the last piece, when only the short
     *                   dwell runs (nothing is fed into a slowed wheel, so nothing to wait for).
     */
    private Command feedOneCore(int[] shots, BooleanSupplier morePieces) {
        final int[] exitsAtStart = new int[1];
        final boolean[] preloaded = new boolean[1];
        final boolean[] staged = new boolean[1];
        Command handoff = robot.storage.hasExitSensor()
                ? sequential(
                        conditional(() -> preloaded[0], noop(), robot.storage.advanceOneCommand()),
                        transferLeg(staged))
                : parallel(
                        robot.storage.advanceForMsCommand(SENSORLESS_FEED_PULSE_MS),
                        transferLeg(staged));
        return sequential(
                instant(() -> {
                    exitsAtStart[0] = robot.storage.getExitEvents();
                    preloaded[0] = robot.transfer.hasPieceInLift() || robot.transfer.pieceAtFeed();
                    // Without a feed sensor the timed pulse is taken to have staged the piece; with
                    // one, the sensed leg records what the sensor saw after the lift.
                    staged[0] = !robot.transfer.hasFeedSensor();
                }),
                handoff,
                afterShot(morePieces),
                instant(() -> {
                    if (!pieceWasShot(exitsAtStart[0], preloaded[0], staged[0])) return;
                    shots[0]++;
                    shotsFired = shots[0];
                    if (!robot.storage.hasExitSensor()) robot.storage.markExited();
                }));
    }

    /**
     * One piece through the transfer into the flywheel. With a feed sensor: lift until staged, note
     * it, feed until clear. Without one: a single {@link #SENSORLESS_FEED_PULSE_MS} pulse at feed
     * speed ("lift" and "feed" are the same motor, so two timed speeds would be theatre). The
     * sensorless leg is the bare command, not a sequential: a sequential hands off one child per
     * tick, and the storage must start in the same tick as the transfer, never a loop before it.
     */
    private Command transferLeg(boolean[] staged) {
        if (robot.transfer.hasFeedSensor()) {
            return sequential(
                    robot.transfer.liftOneCommand(),
                    instant(() -> staged[0] = robot.transfer.pieceAtFeed()),
                    robot.transfer.feedCommand());
        }
        return robot.transfer.feedForMsCommand(SENSORLESS_FEED_PULSE_MS);
    }

    /**
     * After a piece has gone through the wheel. With another piece to come: a short minimum so the
     * speed dip has begun, then wait for the wheel to come back up, bounded (a second piece fed into
     * a slowed wheel is a short shot). After the last piece: only the minimum dwell, so the wheel is
     * never idled with the piece still in it, but no time is spent waiting for a recovery nobody
     * needs. Nothing to wait for without a shooter.
     */
    private Command afterShot(BooleanSupplier morePieces) {
        if (!robot.shooter.isAvailable()) return noop();
        return conditional(morePieces, flywheelRecovery(), waitMs(Shooter.SHOT_RECOVERY_MIN_MS));
    }

    private Command flywheelRecovery() {
        return sequential(
                waitMs(Shooter.SHOT_RECOVERY_MIN_MS),
                race(waitUntil(robot.shooter::atSpeed), waitMs(Shooter.SHOT_RECOVERY_TIMEOUT_MS)));
    }

    private boolean pieceWasShot(int exitsAtStart, boolean preloaded, boolean staged) {
        boolean left = !robot.storage.hasExitSensor() || preloaded
                || robot.storage.getExitEvents() > exitsAtStart;
        boolean fed = !robot.transfer.hasFeedSensor() || (staged && !robot.transfer.pieceAtFeed());
        return left && fed;
    }

    // ---- Aiming ----

    /**
     * The field heading the robot must hold so the shooter faces {@code target}. When a tag in
     * {@code [minTagId, maxTagId]} is visible the bearing comes from its {@code tx} (heading +
     * camera yaw - tx; tx is positive to the right), otherwise from odometry toward the point. The
     * shooter's firing direction ({@code Shooter.HEADING_OFFSET_RAD}) is subtracted, so a rear-firing
     * shooter turns its back to the target. NaN without a pose, or without a target and a tag.
     */
    public double aimHeading(Pose target, int minTagId, int maxTagId) {
        Pose pose = robot.drivetrain.getPose();
        if (pose == null) return Double.NaN;
        double tx = robot.limelight.getTagTx(minTagId, maxTagId);
        double bearing;
        if (!Double.isNaN(tx)) {
            bearing = pose.heading() + Math.toRadians(Limelight.CAMERA_YAW_OFFSET_DEGREES) - Math.toRadians(tx);
        } else {
            if (target == null) return Double.NaN;
            bearing = Math.atan2(target.y() - pose.y(), target.x() - pose.x());
        }
        return Angles.normalizeAngle(bearing - Shooter.HEADING_OFFSET_RAD);
    }

    /**
     * Turns in place until the shooter faces {@code target} (tag-refined when one is visible), then
     * hands the sticks back. SUCCESS within {@link #AIM_TOLERANCE_DEGREES}; TIMED_OUT after
     * {@link #AIM_TIMEOUT_MS}, or when no pose is available.
     */
    public Command aimAt(Pose target, int minTagId, int maxTagId) {
        return reporting("aim",
                bounded(aimCore(target, minTagId, maxTagId), AIM_TIMEOUT_MS),
                Outcome.SUCCESS, Outcome.TIMED_OUT, () -> aimed(target, minTagId, maxTagId));
    }

    /**
     * Pedro's hold as the turn primitive (as {@code Drivetrain.turnToCommand}), re-issued whenever
     * the wanted heading moves by more than {@link #AIM_REISSUE_DEGREES} so a tag can refine it.
     * Requires the drivetrain; {@code setEnd} hands control back either way.
     */
    private Command aimCore(Pose target, int minTagId, int maxTagId) {
        final long[] startedAt = new long[1];
        final double[] commanded = new double[1];
        return Command.build()
                .setStart(() -> {
                    startedAt[0] = robot.getClock().nowMs();
                    commanded[0] = Double.NaN;
                    reissueAim(target, minTagId, maxTagId, commanded);
                })
                .setExecute(() -> reissueAim(target, minTagId, maxTagId, commanded))
                .setDone(() -> robot.getClock().nowMs() - startedAt[0] >= Drivetrain.MIN_PATH_MS
                        && aimed(target, minTagId, maxTagId))
                .setEnd(ec -> robot.drivetrain.cancelPath())
                .requiring(robot.drivetrain);
    }

    private void reissueAim(Pose target, int minTagId, int maxTagId, double[] commanded) {
        double wanted = aimHeading(target, minTagId, maxTagId);
        if (Double.isNaN(wanted)) return;
        if (!Double.isNaN(commanded[0])
                && Math.abs(Angles.angleError(commanded[0], wanted)) < Math.toRadians(AIM_REISSUE_DEGREES)) {
            return;
        }
        commanded[0] = wanted;
        robot.drivetrain.holdHeading(wanted);
    }

    private boolean aimed(Pose target, int minTagId, int maxTagId) {
        double wanted = aimHeading(target, minTagId, maxTagId);
        return !Double.isNaN(wanted)
                && robot.drivetrain.atHeading(wanted, Math.toRadians(AIM_TOLERANCE_DEGREES));
    }

    // ---- Localization and heading ----

    /**
     * AprilTag pipeline, wait for a trustworthy botpose, apply it. In BIOBUZZ every tag rides on a
     * moving HIVE CELL, so this is expected to report NO_TARGET all season; it stays wired for a
     * future static reference and costs nothing.
     */
    public Command relocalize() {
        final boolean[] fixed = new boolean[1];
        return reporting("relocalize",
                sequential(
                        instant(() -> {
                            fixed[0] = false;
                            robot.limelight.activateAprilTagPipeline();
                        }),
                        waitMs(PIPELINE_WARMUP_MS),
                        race(waitUntil(() -> robot.limelight.getBotposeAsPedroPose() != null),
                                waitMs(RELOCALIZE_TIMEOUT_MS)),
                        // The only explicit requirement in this class: nothing else here drives, and a
                        // pose write must not race the driver-control default command.
                        instant(() -> fixed[0] = robot.tryLocalizeFromAprilTag()).requiring(robot.drivetrain)),
                Outcome.SUCCESS, Outcome.NO_TARGET, () -> fixed[0]);
    }

    /** Turns in place to an absolute heading; hands the follower back either way. */
    public Command snapToHeading(double headingRadians) {
        String name = String.format(Locale.US, "snapTo %.0f",
                Math.toDegrees(Angles.normalizeAngle(headingRadians)));
        return reporting(name,
                bounded(robot.drivetrain.turnToCommand(headingRadians), SNAP_TIMEOUT_MS),
                Outcome.SUCCESS, Outcome.TIMED_OUT, () -> atHeading(headingRadians));
    }

    /**
     * Drives a straight Pedro path from the current pose to {@code target}, then hands the follower
     * back. The path is built at start ({@code Paths.line(current, target)} with a linear heading
     * sweep), so the target may be computed any time earlier. {@code holdEnd} is false: in teleop
     * the driver-control default resumes the moment this releases the drivetrain, and in auto the
     * next leg follows.
     */
    public Command driveTo(Pose target) {
        return reporting("driveTo",
                bounded(robot.drivetrain.followLazyCommand(() -> lineTo(target), false), DRIVE_TO_TIMEOUT_MS),
                Outcome.SUCCESS, Outcome.TIMED_OUT, () -> robot.drivetrain.atPose(
                        target, DRIVE_TO_TOLERANCE_INCHES, DRIVE_TO_TOLERANCE_INCHES));
    }

    // ---- Private geometry helpers ----

    /**
     * Straight line from the current pose to {@code target}; null without a pose, and null when the
     * target is within {@link #MIN_PATH_INCHES} (a zero-length line is degenerate inside Pedro).
     */
    private Path lineTo(Pose target) {
        Pose current = robot.drivetrain.getPose();
        if (current == null || target == null) return null;
        if (current.distance(target) < MIN_PATH_INCHES) return null;
        return Paths.line(current, target).linear(current.heading(), target.heading());
    }

    /** Straight line from the current pose to the standoff pose in front of the seen piece. */
    private Path approachPath() {
        Pose current = robot.drivetrain.getPose();
        if (current == null || !robot.limelight.hasStableBlob()) return null;
        Pose target = robot.limelight.estimateBlobApproachPose(current);
        if (target == null) return null;
        if (current.distance(target) < MIN_PATH_INCHES) return null;
        return Paths.line(current, target).linear(current.heading(), target.heading());
    }

    private double headingToBlob() {
        Pose current = robot.drivetrain.getPose();
        if (current == null || !robot.limelight.hasStableBlob()) return Double.NaN;
        double[] robotFrame = robot.limelight.estimateBlobInRobotFrame();
        if (robotFrame == null) return Double.NaN;
        return Angles.headingToward(current.heading(), robotFrame[0], robotFrame[1]);
    }

    private boolean alignedToBlob() {
        return robot.limelight.hasStableBlob()
                && Math.abs(robot.limelight.getFilteredBlobTx()) <= ALIGN_TOLERANCE_DEGREES;
    }

    private boolean atHeading(double headingRadians) {
        Pose pose = robot.drivetrain.getPose();
        if (pose == null) return false;
        double errorDegrees = Math.toDegrees(Angles.angleError(pose.heading(), headingRadians));
        return Math.abs(errorDegrees) <= SNAP_TOLERANCE_DEGREES;
    }
}
