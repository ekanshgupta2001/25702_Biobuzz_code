package org.firstinspires.ftc.teamcode.subsystems;

import com.pedropathing.controllers.Controller;
import com.pedropathing.controllers.PIDController;
import com.pedropathing.drivetrain.DrivePowers;
import com.pedropathing.follower.Follower;
import com.pedropathing.follower.ManualDrive;
import com.pedropathing.ivy.Command;
import com.pedropathing.ivy.behaviors.BlockedBehavior;
import com.pedropathing.ivy.behaviors.EndCondition;
import com.pedropathing.ivy.behaviors.InterruptedBehavior;
import com.pedropathing.math.Pose;
import com.pedropathing.paths.Path;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.pedro.Constants;
import org.firstinspires.ftc.teamcode.util.hardware.Hardware;
import org.firstinspires.ftc.teamcode.util.math.Angles;
import org.firstinspires.ftc.teamcode.util.time.Clock;

import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

/**
 * Wraps the Pedro follower: manual driving, pose access, and path control.
 *
 * <p>Pedro 3.0.0 has one follower with four modes, implied by the last command it was given:
 * {@code follow} (drives a path), {@code hold} (station-keeps), {@code manual} (stick powers) and
 * {@code stop}. There is no "teleop mode" to engage and no flag to keep truthful; calling
 * {@link #drive} every loop is the whole teleop story, and {@link #cancelPath()} is one
 * {@code manual(0, 0, 0)}.
 *
 * <h2>Arbitration</h2>
 * This subsystem is an Ivy <em>resource</em>: every command that moves the robot declares
 * {@code requiring(drivetrain)}. {@link #driverControlCommand} sits at priority -1 with
 * {@link InterruptedBehavior#SUSPEND}, so scheduling any macro automatically parks driver control
 * and ending the macro automatically restores it. The one thing Ivy cannot do is stop the
 * follower, which drives itself once handed a path; every path command therefore hands control
 * back in its {@code setEnd}.
 *
 * <h2>Testability</h2>
 * All motion goes through {@link PathFollower} and all time through a {@link Clock}. Both are
 * injectable, so every command below runs on the JVM against a fake follower and a fake clock
 * ({@code DrivetrainCommandTest}). {@link #getFollower()} exposes the raw Pedro follower for path
 * introspection and the match logger; it is null under test.
 */
public class Drivetrain {
    public static int DRIVER_CONTROL_PRIORITY = -1;

    /**
     * Whether the robot holds its heading when the driver is not turning. Only as good as the
     * localizer's heading; {@link #resetHeading()} is the driver's escape hatch.
     */
    public static boolean HEADING_HOLD_ENABLED = true;
    /**
     * Turn-stick magnitude above which the driver is considered to be steering. The stick arrives
     * already shaped ({@code DriveScaling.shape}: 0.07 raw deadband, then square expo, then the slow
     * scale), so noise is already zero and any non-zero value is intent. A larger figure here is
     * compared against the <em>shaped</em> value: 0.05 meant a raw deflection up to ~0.28 (0.45 in
     * slow mode) was thrown away and the hold fought the driver's small corrections.
     */
    public static double HEADING_HOLD_STICK_DEADBAND = 0.001;
    /** Error is in RADIANS here, so a gain of 1.5 maps 10 degrees (0.17 rad) to ~0.26 turn power. */
    public static double HEADING_HOLD_P = 1.5;
    public static double HEADING_HOLD_I = 0.0;
    public static double HEADING_HOLD_D = 0.08;
    public static double HEADING_HOLD_MAX_TURN = 0.4;
    /** Below this error, stop correcting, otherwise the robot hunts around the setpoint. */
    public static double HEADING_HOLD_TOLERANCE_RAD = Math.toRadians(1.0);

    /**
     * Minimum time a started path or turn must run before it may report complete. A zero-length
     * path is at its parametric end on tick one and would otherwise read as a perfect zero-time run.
     */
    public static long MIN_PATH_MS = 60;
    /** A turn that has not settled by then hands the sticks back anyway. */
    public static long TURN_TIMEOUT_MS = 2500;
    /** A turn counts as arrived inside this error. */
    public static double TURN_TOLERANCE_RAD = Math.toRadians(2.0);
    /**
     * How long a freshly constructed Pinpoint localizer spends recalibrating its IMU. A pose written
     * before that is lost (docs/01 A.9 gotcha 6), so {@link #setPose} written inside this window is
     * repeated once by {@link #update()} after it.
     */
    public static long LOCALIZER_SETTLE_MS = 1000;

    /** Null when the drivetrain could not be built; every motion call then no-ops. */
    private final PathFollower follower;
    /** The raw Pedro follower for callers needing its full API. Null under test. */
    private final Follower pedro;
    private final Clock clock;

    private boolean fieldCentric = true;
    /**
     * Field heading the driver calls "forward", radians. Field-centric sticks are rotated by
     * (heading - this), so a blue driver, who stands behind the +X wall facing -X, pushes the stick
     * up and the robot drives -X. Season value from {@code game/Field.driverForwardHeading}; the aim
     * lock and every macro stay in the true field frame.
     */
    private double driverHeadingOffset = 0;

    private final long builtAtMs;
    /** The last pose written through {@link #setPose}, for the post-calibration repeat. */
    private Pose lastSetPose = null;
    private boolean reapplyWhenSettled = false;
    /** How many times the pose estimate has been rewritten; anything derived from an older frame is stale. */
    private int poseWrites = 0;

    private final PIDController headingController =
            Controller.pid(HEADING_HOLD_P, HEADING_HOLD_I, HEADING_HOLD_D);
    /** The heading being held, in radians, or null when the driver is steering. */
    private Double heldHeading = null;
    /** External setpoint for the hold (an aim lock), or null. NaN from it means no opinion this loop. */
    private DoubleSupplier aimLock = null;

    public Drivetrain(HardwareMap hardwareMap) {
        this(hardwareMap, Clock.system());
    }

    public Drivetrain(HardwareMap hardwareMap, Clock clock) {
        this.clock = clock;
        this.builtAtMs = clock.nowMs();
        Follower built = null;
        try {
            built = Constants.create(hardwareMap);
            if (built == null) {
                Hardware.recordFailure("drivetrain", "Constants.create returned null (not configured)");
            }
        } catch (RuntimeException e) {
            // create() resolves four drive motors plus the localizer. Any missing name would
            // otherwise throw straight out of Robot's constructor and kill the OpMode.
            Hardware.recordFailure("drivetrain", "Constants.create failed: " + e.getMessage());
        }
        pedro = built;
        follower = built == null ? null : new PedroPathFollower(built);
    }

    /** Builds on an already-constructed follower. Tests inject a fake here. */
    public Drivetrain(PathFollower follower, Clock clock) {
        this.clock = clock;
        this.builtAtMs = clock.nowMs();
        this.follower = follower;
        this.pedro = follower instanceof PedroPathFollower ? ((PedroPathFollower) follower).raw() : null;
    }

    /** False when the drivetrain could not be built. All motion calls then no-op. */
    public boolean isAvailable() {
        return follower != null;
    }

    /**
     * Call once from the OpMode's {@code start()} (MatchOpMode does): hands the follower to the
     * sticks, manual mode with zero power. A pose written during init is repeated by the first
     * {@link #update()} after {@link #LOCALIZER_SETTLE_MS}, whether START came early or late.
     */
    public void onStart() {
        if (follower != null) follower.manual(0, 0, 0);
    }

    /** True once the localizer's IMU calibration window has passed (always true without a follower). */
    public boolean isLocalizerSettled() {
        return follower == null || clock.nowMs() - builtAtMs >= LOCALIZER_SETTLE_MS;
    }

    /** True while a pose written during calibration is still waiting to be repeated. */
    public boolean isPoseReapplyPending() {
        return reapplyWhenSettled;
    }

    /** The field heading the driver calls "forward"; see {@link #setDriverHeadingOffset}. */
    public double getDriverHeadingOffset() {
        return driverHeadingOffset;
    }

    /**
     * Sets which field heading is "stick forward" for field-centric driving: {@code 0} for a driver
     * behind the -X wall, {@code Math.PI} for one behind the +X wall. {@link #resetHeading()} uses
     * the same value, since the driver presses it facing away from their own wall.
     */
    public void setDriverHeadingOffset(double fieldHeadingRad) {
        driverHeadingOffset = fieldHeadingRad;
    }

    /**
     * Applies stick powers. Field-centric input is rotated into the robot frame with Pedro's
     * {@link ManualDrive#fieldCentric}; robot-centric input goes straight through. Convention
     * (Pedro's): {@code +forward} ahead, {@code +strafe} left, {@code +turn} counter-clockwise.
     * Powers are latched by the follower, so this is called every loop while driving.
     */
    public void drive(double forward, double strafe, double turn) {
        if (follower == null) return;
        Pose pose = follower.pose();
        if (fieldCentric && pose != null) {
            // The driver's "forward" is field heading driverHeadingOffset, so the stick is rotated
            // by the robot's heading relative to that, not relative to +X.
            DrivePowers p = ManualDrive.fieldCentric(forward, strafe, turn,
                    pose.heading() - driverHeadingOffset);
            follower.manual(p.forward(), p.strafe(), p.turn());
        } else {
            follower.manual(forward, strafe, turn);
        }
    }

    public void setFieldCentric(boolean fieldCentric) {
        this.fieldCentric = fieldCentric;
    }

    public void toggleFieldCentric() {
        fieldCentric = !fieldCentric;
    }

    public boolean isFieldCentric() {
        return fieldCentric;
    }

    /**
     * Writes the pose estimate, e.g. at init or from an external fix.
     *
     * <p>Also drops the held heading. The hold's setpoint was captured in the old heading frame;
     * keeping it after the frame changes makes the controller chase a number that no longer means
     * anything, and the robot rotates by the size of the correction.
     *
     * <p>A write inside {@link #LOCALIZER_SETTLE_MS} of construction lands during the Pinpoint's IMU
     * calibration and is lost, so it is repeated once by {@link #update()} after the window. If the
     * robot has already started moving by then (START within a second of INIT), up to that second of
     * motion is discarded in exchange for a heading that is right for the rest of the match.
     */
    public void setPose(Pose pose) {
        if (follower != null && pose != null) {
            follower.setPose(pose);
            lastSetPose = pose;
            reapplyWhenSettled = !isLocalizerSettled();
            poseWrites++;
        }
        releaseHeadingHold();
    }

    /**
     * Count of pose rewrites ({@link #setPose}, {@link #resetHeading}, the post-calibration repeat).
     * A caller holding something computed against the pose ({@code Macros}' tag-derived aim
     * correction) compares this with the count it saw, and drops its value when they differ.
     */
    public int getPoseWrites() {
        return poseWrites;
    }

    public Pose getPose() {
        return follower == null ? null : follower.pose();
    }

    /**
     * The raw Pedro follower, for path introspection and diagnostics, or {@code null} when the
     * drivetrain is unavailable or was built on a non-Pedro {@link PathFollower}.
     */
    public Follower getFollower() {
        return pedro;
    }

    /** True while a path is actively being followed. Stick input is ignored during this. */
    public boolean isFollowingPath() {
        return follower != null && follower.mode() == Follower.Mode.FOLLOW;
    }

    /** True while station-keeping after a path or a hold command. */
    public boolean isHoldingPose() {
        return follower != null && follower.mode() == Follower.Mode.HOLD;
    }

    /** Whether the robot is within the given distance of a pose on each axis. False when unavailable. */
    public boolean atPose(Pose target, double xToleranceInches, double yToleranceInches) {
        if (follower == null || target == null) return false;
        Pose here = follower.pose();
        return here != null
                && Math.abs(target.x() - here.x()) <= xToleranceInches
                && Math.abs(target.y() - here.y()) <= yToleranceInches;
    }

    /** True when the heading is within {@code toleranceRad} of {@code headingRad}. */
    public boolean atHeading(double headingRad, double toleranceRad) {
        Pose here = getPose();
        return here != null && Math.abs(Angles.angleError(here.heading(), headingRad)) <= toleranceRad;
    }

    /**
     * Abandons the current path or hold immediately and hands control back to the driver.
     *
     * <p>This is the abort path: cancelling the Ivy command that started a path does not itself
     * stop the follower, because the follower drives itself. Without this call the robot keeps
     * going to its target with the sticks locked out.
     */
    public void cancelPath() {
        if (follower != null) follower.manual(0, 0, 0);
    }

    /**
     * Treats the robot's current facing as "away from the driver wall" (the driver-forward heading,
     * see {@link #setDriverHeadingOffset}), keeping its x/y position. The driver escape hatch for
     * field-centric drive when localisation has drifted: face away from your wall, press it.
     */
    public void resetHeading() {
        if (follower == null) return;
        Pose p = follower.pose();
        if (p == null) return;
        // setPose() releases the heading hold. Without that, the hold would still be aiming at the
        // heading this call just discarded and would spin the robot back toward it.
        setPose(new Pose(p.x(), p.y(), driverHeadingOffset));
    }

    // ---- Heading hold ----

    /**
     * Replaces a centred turn stick with a correction back toward the held heading.
     *
     * <p>A mecanum robot does not track straight on its own. <b>It must never fight the
     * driver:</b> any deliberate turn input hands control straight back and re-captures the heading
     * on release, so the robot holds wherever the driver left it.
     */
    private double applyHeadingHold(double turn) {
        if (!HEADING_HOLD_ENABLED || follower == null) {
            heldHeading = null;
            return turn;
        }
        if (Math.abs(turn) >= HEADING_HOLD_STICK_DEADBAND) {
            heldHeading = null;
            return turn;
        }
        Pose pose = follower.pose();
        if (pose == null) {
            heldHeading = null;
            return turn;
        }
        double locked = aimLock == null ? Double.NaN : aimLock.getAsDouble();
        if (!Double.isNaN(locked)) {
            // An aim lock supplies the setpoint every loop; the correction starts at once.
            if (heldHeading == null) resetHeadingController();
            heldHeading = Angles.normalizeAngle(locked);
        } else if (heldHeading == null) {
            heldHeading = pose.heading();
            resetHeadingController();
            return 0;
        }
        // Fed as an error rather than a position, so the controller never sees the raw angles and
        // the 0/2pi seam cannot produce a full-speed spin the short way round.
        double error = Angles.angleError(pose.heading(), heldHeading);
        if (Math.abs(error) <= HEADING_HOLD_TOLERANCE_RAD) return 0;
        double correction = headingController.calculate(0, error);
        return Math.max(-HEADING_HOLD_MAX_TURN, Math.min(HEADING_HOLD_MAX_TURN, correction));
    }

    private void resetHeadingController() {
        headingController.reset();
        headingController.kP = HEADING_HOLD_P;
        headingController.kI = HEADING_HOLD_I;
        headingController.kD = HEADING_HOLD_D;
    }

    /** Forgets the held heading, so the next centred-stick loop captures a fresh one. */
    public void releaseHeadingHold() {
        heldHeading = null;
    }

    /**
     * Locks the heading hold to an externally supplied field heading (radians): aiming a fixed
     * shooter while the driver keeps translating. The supplier is read on every centred-stick loop;
     * NaN means no opinion, which falls back to the ordinary capture-and-hold. A deliberate turn on
     * the stick still passes straight through (never fight the driver) and the lock resumes on
     * release. {@link #getHeldHeading()} reports the locked heading while active. Needs
     * {@link #HEADING_HOLD_ENABLED}.
     */
    public void setAimLock(DoubleSupplier fieldHeadingRad) {
        aimLock = fieldHeadingRad;
        heldHeading = null;
    }

    /** Back to the ordinary hold: the next centred-stick loop captures the current heading. */
    public void clearAimLock() {
        aimLock = null;
        heldHeading = null;
    }

    public boolean isAimLocked() {
        return aimLock != null;
    }

    /**
     * Station-keeps at the current position facing {@code headingRadians}: Pedro's hold as a turn
     * primitive. Prefer {@link #turnToCommand}, which also hands control back; this is for a
     * command that re-issues the target as it refines it, and must call {@link #cancelPath()} itself.
     */
    public void holdHeading(double headingRadians) {
        if (follower == null) return;
        Pose here = follower.pose();
        if (here != null) follower.hold(here.withHeading(headingRadians), false);
    }

    public boolean isHeadingHoldActive() {
        return heldHeading != null;
    }

    /** The heading being held in radians, or NaN when not holding. */
    public double getHeldHeading() {
        return heldHeading == null ? Double.NaN : heldHeading;
    }

    public void update() {
        if (follower == null) return;
        if (reapplyWhenSettled && isLocalizerSettled()) {
            reapplyWhenSettled = false;
            if (lastSetPose != null) {
                follower.setPose(lastSetPose);
                poseWrites++;
            }
        }
        follower.update();
    }

    // ---- Ivy commands ----

    /**
     * The default command: feeds stick values to the follower whenever nothing else owns the
     * drivetrain. Schedule once at OpMode init.
     *
     * <p>Priority -1 plus {@link InterruptedBehavior#SUSPEND} is what makes macros cancellable for
     * free; {@link BlockedBehavior#QUEUE} keeps it from being silently dropped if scheduled while
     * something else holds the resource. The behaviour lives in {@code setExecute} because the
     * Scheduler's resume path re-adds a suspended command without re-calling {@code start()}.
     * The following-path guard is a safety net: a manual write would abandon a path mid-flight.
     */
    public Command driverControlCommand(DoubleSupplier forward, DoubleSupplier strafe,
                                        DoubleSupplier turn) {
        return Command.build()
                .setExecute(() -> {
                    if (isFollowingPath()) return;
                    drive(forward.getAsDouble(), strafe.getAsDouble(),
                            applyHeadingHold(turn.getAsDouble()));
                })
                .setDone(() -> false)
                .setEnd(ec -> {
                    releaseHeadingHold();
                    drive(0, 0, 0);
                })
                .setPriority(DRIVER_CONTROL_PRIORITY)
                .setInterruptedBehavior(InterruptedBehavior.SUSPEND)
                .setBlockedBehavior(BlockedBehavior.QUEUE)
                .requiring(this);
    }

    /** Follows a pre-built path with the same guarantees as {@link #followLazyCommand}. */
    public Command followPathCommand(Path path, boolean holdEnd) {
        if (follower == null || path == null) return finishedCommand();
        return followLazyCommand(() -> path, holdEnd);
    }

    /**
     * Follows a path that is not known until the command actually starts.
     *
     * <p>Necessary for anything vision-driven or planned from the current pose: the supplier runs
     * in {@code start()}, so the path is built from where the robot is <em>then</em>. A supplier
     * returning {@code null} yields a command that finishes immediately rather than moving the
     * robot somewhere arbitrary.
     *
     * <h3>What {@code holdEnd} does</h3>
     * With {@code holdEnd = true} a natural end leaves the follower station-keeping at the path's
     * end pose (commanded explicitly, so the robot does not stop at the 97.5% parametric threshold
     * while Pedro's own tracker catches up). With {@code holdEnd = false}, or whenever the command
     * is interrupted, {@link #cancelPath()} runs and control goes straight back to the driver.
     * Pedro's {@code PedroCommands.follow} does neither, which is why this exists.
     */
    public Command followLazyCommand(Supplier<Path> pathSupplier, boolean holdEnd) {
        // Boxed so the lambdas below share one instance of each; the command may be built once and
        // run more than once, and setStart resets all three.
        final boolean[] started = new boolean[1];
        final long[] startedAt = new long[1];
        final Pose[] endPose = new Pose[1];

        return Command.build()
                .setStart(() -> {
                    startedAt[0] = clock.nowMs();
                    started[0] = false;
                    endPose[0] = null;
                    if (follower == null) return;
                    try {
                        Path path = pathSupplier.get();
                        if (path == null) return;
                        endPose[0] = path.endPose();     // throws if the path has no heading
                        follower.follow(path);
                        started[0] = true;
                    } catch (RuntimeException e) {
                        // A path without a heading interpolator, or a supplier that failed. Better
                        // a command that finishes at once than an exception out of the OpMode loop.
                        Hardware.recordFailure("drivetrain", "path rejected: " + e.getMessage());
                    }
                })
                .setDone(() -> {
                    if (!started[0]) return true;
                    if (clock.nowMs() - startedAt[0] < MIN_PATH_MS) return false;
                    return follower.atParametricEnd();
                })
                .setEnd(ec -> {
                    if (!started[0]) return;
                    if (ec == EndCondition.NATURALLY && holdEnd) {
                        if (endPose[0] != null) follower.hold(endPose[0], false);
                    } else {
                        cancelPath();
                    }
                })
                .requiring(this);
    }

    /**
     * Turns in place to an absolute field heading, then hands control back to the driver.
     *
     * <p>Pedro 3 has no turn primitive; a hold at the current position with the new heading is
     * the same thing. Done when within {@link #TURN_TOLERANCE_RAD}, or after
     * {@link #TURN_TIMEOUT_MS}; either way {@code setEnd} releases the hold, because a snap is
     * something a driver does mid-drive and wants the sticks back from immediately.
     */
    public Command turnToCommand(double headingRadians) {
        if (follower == null) return finishedCommand();
        final long[] startedAt = new long[1];
        return Command.build()
                .setStart(() -> {
                    startedAt[0] = clock.nowMs();
                    Pose here = follower.pose();
                    if (here != null) follower.hold(here.withHeading(headingRadians), false);
                })
                .setDone(() -> {
                    long elapsed = clock.nowMs() - startedAt[0];
                    if (elapsed < MIN_PATH_MS) return false;
                    return atHeading(headingRadians, TURN_TOLERANCE_RAD) || elapsed >= TURN_TIMEOUT_MS;
                })
                .setEnd(ec -> cancelPath())
                .requiring(this);
    }

    /**
     * Holds the current position against pushing until interrupted. Deliberately never finishes on
     * its own: "hold" means "until something else wants the drivetrain". Pedro's own hold command
     * is an instant that releases the resource on tick one.
     */
    public Command holdCommand() {
        if (follower == null) return finishedCommand();
        return Command.build()
                .setStart(() -> {
                    Pose here = follower.pose();
                    if (here != null) follower.hold(here, false);
                })
                .setDone(() -> false)
                .setEnd(ec -> cancelPath())
                .requiring(this);
    }

    /**
     * Drives at fixed robot-frame powers for {@code ms} on the injected clock, then hands the
     * follower back ({@code manual(0, 0, 0)}). The tuned twin of
     * {@code OpenLoopDrive.driveForMsCommand}: the hardcoded auto's LEAVE goes through whichever
     * motor layer exists. Bypasses field-centric mixing and the heading hold on purpose. Finishes at
     * once without a follower.
     */
    public Command driveForMsCommand(double forward, double strafe, double turn, long ms) {
        if (follower == null) return finishedCommand();
        final long[] startedAt = new long[1];
        return Command.build()
                .setStart(() -> {
                    startedAt[0] = clock.nowMs();
                    follower.manual(forward, strafe, turn);
                })
                .setExecute(() -> follower.manual(forward, strafe, turn))
                .setDone(() -> clock.nowMs() - startedAt[0] >= ms)
                .setEnd(ec -> cancelPath())
                .requiring(this);
    }

    /** A command that completes on its first tick. Returned when there is no drivetrain to move. */
    private static Command finishedCommand() {
        return Command.build().setDone(() -> true);
    }
}
