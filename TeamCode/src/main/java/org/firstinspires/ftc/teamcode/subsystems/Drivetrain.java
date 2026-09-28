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
import com.pedropathing.revhub.drivetrains.Mecanum;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.pedro.Constants;
import org.firstinspires.ftc.teamcode.util.math.Angles;

import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

/**
 * The mecanum drivetrain: stick driving, pose, heading hold, the aim lock, and Pedro paths.
 *
 * <h2>One motor layer</h2>
 * This class owns exactly one {@link Mecanum}, built from names and directions alone, and hands that
 * same instance to the {@link Follower} when the tuning configs exist. So:
 * <ul>
 *   <li><b>Sticks always work.</b> With a follower they go through {@code follower.manual(...)},
 *       field-centric against the Pinpoint's heading. Without one they go straight to
 *       {@code mecanum.drive(...)}, robot-centric, because there is no pose to rotate against.</li>
 *   <li><b>Paths, the heading hold and the aim lock need the follower</b> and are inert until
 *       AutoTune has filled {@code Constants.localizerConfig} and {@code foresightConfig}.</li>
 * </ul>
 * There is deliberately no second drivetrain class for the untuned case. Two {@code Mecanum} objects
 * over four motors keep two independent {@code CachedMotor} power caches and fight; one instance is
 * the whole fix, and it is why this code no longer needs a rule about which layer may exist.
 *
 * <h2>Arbitration</h2>
 * This subsystem is an Ivy <em>resource</em>: every command that moves the robot declares
 * {@code requiring(drivetrain)}. {@link #driverControlCommand} sits at priority -1 with
 * {@link InterruptedBehavior#SUSPEND}, so scheduling a macro parks driver control and ending the
 * macro restores it. The one thing Ivy cannot do is stop the follower, which drives itself once
 * handed a path, so every path command hands control back in its {@code setEnd}.
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
     * already shaped ({@code DriveScaling.shape}: raw deadband, then expo, then the slow scale), so
     * noise is already zero and any non-zero value is intent. This is compared against the
     * <em>shaped</em> value: 0.05 here threw away raw deflections up to ~0.28 and the hold fought the
     * driver's small corrections.
     */
    public static double HEADING_HOLD_STICK_DEADBAND = 0.001;
    /** Error is in RADIANS, so a gain of 1.5 maps 10 degrees (0.17 rad) to ~0.26 turn power. */
    public static double HEADING_HOLD_P = 1.5;
    public static double HEADING_HOLD_I = 0.0;
    public static double HEADING_HOLD_D = 0.08;
    public static double HEADING_HOLD_MAX_TURN = 0.4;
    /** Below this error, stop correcting, or the robot hunts around the setpoint. */
    public static double HEADING_HOLD_TOLERANCE_RAD = Math.toRadians(1.0);

    /**
     * Minimum time a started path or turn must run before it may report complete. A zero-length path
     * is at its parametric end on tick one and would otherwise read as a perfect zero-time run.
     */
    public static long MIN_PATH_MS = 60;
    /** A turn that has not settled by then hands the sticks back anyway. */
    public static long TURN_TIMEOUT_MS = 2500;
    public static double TURN_TOLERANCE_RAD = Math.toRadians(2.0);
    /**
     * How long a freshly constructed Pinpoint spends recalibrating its IMU. A pose written before
     * that is lost (docs/01 A.9 gotcha 6), so a {@link #setPose} inside the window is repeated once by
     * {@link #update()} afterwards.
     */
    public static long LOCALIZER_SETTLE_MS = 1000;

    /** The motor layer. Never null: four names and four directions are all it needs. */
    private final Mecanum mecanum;
    /** Null until the tuning configs are filled. Drives {@link #mecanum}, never its own copy. */
    private final Follower follower;

    private boolean fieldCentric = true;
    /**
     * Field heading the driver calls "forward", radians. Field-centric sticks are rotated by
     * (heading - this), so a blue driver standing behind the +X wall facing -X pushes the stick up
     * and the robot drives -X. Season value from {@code game/Field.driverForwardHeading}; the aim lock
     * and every macro stay in the true field frame.
     */
    private double driverHeadingOffset = 0;

    private final long builtAtMs;
    private Pose lastSetPose = null;
    private boolean reapplyWhenSettled = false;
    /** How many times the pose estimate has been rewritten; anything derived from an older frame is stale. */
    private int poseWrites = 0;

    private final PIDController headingController =
            Controller.pid(HEADING_HOLD_P, HEADING_HOLD_I, HEADING_HOLD_D);
    /** The heading being held, radians, or null while the driver is steering. */
    private Double heldHeading = null;
    /** External setpoint for the hold (an aim lock), or null. NaN from it means no opinion this loop. */
    private DoubleSupplier aimLock = null;

    public Drivetrain(HardwareMap hardwareMap) {
        this.builtAtMs = System.currentTimeMillis();
        this.mecanum = Constants.createMecanum(hardwareMap);
        this.follower = Constants.create(hardwareMap, mecanum);
    }

    /** True once AutoTune's configs exist, so paths, the heading hold and the aim lock are live. */
    public boolean hasFollower() {
        return follower != null;
    }

    /**
     * The raw Pedro follower for path introspection, or {@code null} before tuning.
     */
    public Follower getFollower() {
        return follower;
    }

    /**
     * Call once from the OpMode's {@code start()}: hands the follower to the sticks, manual mode at
     * zero power. A pose written during init is repeated by the first {@link #update()} after
     * {@link #LOCALIZER_SETTLE_MS}, whether START came early or late.
     */
    public void onStart() {
        if (follower != null) follower.manual(0, 0, 0);
        else mecanum.stop();
    }

    /** True once the localizer's IMU calibration window has passed (always true without a follower). */
    public boolean isLocalizerSettled() {
        return follower == null || System.currentTimeMillis() - builtAtMs >= LOCALIZER_SETTLE_MS;
    }

    /**
     * Sets which field heading is "stick forward" for field-centric driving: {@code 0} for a driver
     * behind the -X wall, {@code Math.PI} for one behind the +X wall. {@link #resetHeading()} uses the
     * same value, since the driver presses it facing away from their own wall.
     */
    public void setDriverHeadingOffset(double fieldHeadingRad) {
        driverHeadingOffset = fieldHeadingRad;
    }

    /**
     * Applies stick powers. Convention (Pedro's): {@code +forward} ahead, {@code +strafe} left,
     * {@code +turn} counter-clockwise. Powers are latched, so this is called every loop while driving.
     *
     * <p>Field-centric needs a pose, so it only applies once there is a follower; before that the
     * sticks are robot-centric and the drive-frame toggle has no effect.
     */
    public void drive(double forward, double strafe, double turn) {
        if (follower == null) {
            mecanum.drive(new DrivePowers(forward, strafe, turn), true);
            return;
        }
        Pose pose = follower.pose();
        if (fieldCentric && pose != null) {
            // The driver's "forward" is field heading driverHeadingOffset, so the stick is rotated by
            // the robot's heading relative to that, not relative to +X.
            DrivePowers p = ManualDrive.fieldCentric(forward, strafe, turn,
                    pose.heading() - driverHeadingOffset);
            follower.manual(p.forward(), p.strafe(), p.turn());
        } else {
            follower.manual(forward, strafe, turn);
        }
    }

    public void toggleFieldCentric() {
        fieldCentric = !fieldCentric;
    }

    /** True when sticks are interpreted in the field frame. Always false without a follower. */
    public boolean isFieldCentric() {
        return fieldCentric && follower != null;
    }

    /**
     * Writes the pose estimate, at init or from an external fix.
     *
     * <p>Also drops the held heading: the hold's setpoint was captured in the old heading frame, and
     * keeping it across a frame change makes the controller chase a number that no longer means
     * anything, rotating the robot by the size of the correction.
     *
     * <p>A write inside {@link #LOCALIZER_SETTLE_MS} of construction lands during the Pinpoint's IMU
     * calibration and is lost, so it is repeated once by {@link #update()} after the window. If the
     * robot has already moved by then (START within a second of INIT), that second of motion is
     * discarded in exchange for a heading that is right for the rest of the match.
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
     * Count of pose rewrites. A caller holding something computed against the pose ({@code Macros}'
     * tag-derived aim correction) compares this with the count it saw and drops its value when they
     * differ.
     */
    public int getPoseWrites() {
        return poseWrites;
    }

    /** The pose estimate, or {@code null} before tuning (there is no localizer). */
    public Pose getPose() {
        return follower == null ? null : follower.pose();
    }

    public boolean isFollowingPath() {
        return follower != null && follower.mode() == Follower.Mode.FOLLOW;
    }

    public boolean isHoldingPose() {
        return follower != null && follower.mode() == Follower.Mode.HOLD;
    }

    /** Whether the robot is within the given distance of a pose on each axis. False without a pose. */
    public boolean atPose(Pose target, double xToleranceInches, double yToleranceInches) {
        Pose here = getPose();
        return here != null && target != null
                && Math.abs(target.x() - here.x()) <= xToleranceInches
                && Math.abs(target.y() - here.y()) <= yToleranceInches;
    }

    public boolean atHeading(double headingRad, double toleranceRad) {
        Pose here = getPose();
        return here != null && Math.abs(Angles.angleError(here.heading(), headingRad)) <= toleranceRad;
    }

    /**
     * Abandons the current path or hold immediately and hands control back to the driver.
     *
     * <p>This is the abort path. Cancelling the Ivy command that started a path does not stop the
     * follower, because the follower drives itself; without this the robot keeps going to its target
     * with the sticks locked out.
     */
    public void cancelPath() {
        if (follower != null) follower.manual(0, 0, 0);
        else mecanum.stop();
    }

    /**
     * Treats the robot's current facing as "away from the driver wall", keeping its x/y. The escape
     * hatch when odometry has drifted: face away from your wall and press it. Also the one-button
     * recovery after a hard collision, since the aim law and the shooter's distance lookup are both
     * pose-derived.
     */
    public void resetHeading() {
        Pose p = getPose();
        // setPose() releases the heading hold; without that the hold would still aim at the heading
        // this call just discarded and would spin the robot back toward it.
        if (p != null) setPose(new Pose(p.x(), p.y(), driverHeadingOffset));
    }

    // ---- Heading hold ----

    /**
     * Replaces a centred turn stick with a correction back toward the held heading.
     *
     * <p>A mecanum robot does not track straight on its own. <b>It must never fight the driver:</b>
     * any deliberate turn input hands control straight back and re-captures the heading on release,
     * so the robot holds wherever the driver left it.
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
        // Fed as an error rather than a position, so the controller never sees the raw angles and the
        // 0/2pi seam cannot produce a full-speed spin the short way round.
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
     * shooter while the driver keeps translating. Read on every centred-stick loop; NaN means no
     * opinion, which falls back to capture-and-hold. A deliberate turn still passes straight through
     * and the lock resumes on release.
     */
    public void setAimLock(DoubleSupplier fieldHeadingRad) {
        aimLock = fieldHeadingRad;
        heldHeading = null;
    }

    public void clearAimLock() {
        aimLock = null;
        heldHeading = null;
    }

    public boolean isAimLocked() {
        return aimLock != null;
    }

    /**
     * Station-keeps at the current position facing {@code headingRadians}: Pedro's hold as a turn
     * primitive. For a command that re-issues the target as it refines it; such a caller must call
     * {@link #cancelPath()} itself.
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

    /**
     * The one {@code follower.update()} per loop. Without a follower there is nothing to tick:
     * {@link #drive} wrote the motors directly.
     */
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
     * The default command: feeds stick values to the drivetrain whenever nothing else owns it.
     * Schedule once at OpMode init.
     *
     * <p>Priority -1 plus {@link InterruptedBehavior#SUSPEND} is what makes macros cancellable for
     * free; {@link BlockedBehavior#QUEUE} keeps it from being silently dropped if something already
     * holds the resource. The behaviour lives in {@code setExecute} because the Scheduler's resume
     * path re-adds a suspended command without re-calling {@code start()}. The following-path guard
     * is a safety net: a manual write would abandon a path mid-flight.
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

    /**
     * Follows a path that is not known until the command starts.
     *
     * <p>The supplier runs in {@code start()}, so the path is built from where the robot is
     * <em>then</em> — necessary for anything planned from the current pose. A supplier returning
     * {@code null} yields a command that finishes immediately rather than driving somewhere
     * arbitrary.
     *
     * <h3>What {@code holdEnd} does</h3>
     * With {@code holdEnd = true} a natural end leaves the follower station-keeping at the path's end
     * pose, commanded explicitly so the robot does not stop at the 97.5% parametric threshold while
     * Pedro's tracker catches up. With {@code holdEnd = false}, or on any interruption,
     * {@link #cancelPath()} runs and control goes back to the driver. Pedro's own
     * {@code PedroCommands.follow} does neither, which is why this exists.
     */
    public Command followLazyCommand(Supplier<Path> pathSupplier, boolean holdEnd) {
        if (follower == null) return finishedCommand();
        // Boxed so the lambdas share one instance each; the command may be built once and run twice.
        final boolean[] started = new boolean[1];
        final long[] startedAt = new long[1];
        final Pose[] endPose = new Pose[1];

        return Command.build()
                .setStart(() -> {
                    startedAt[0] = System.currentTimeMillis();
                    started[0] = false;
                    endPose[0] = null;
                    Path path = pathSupplier.get();
                    if (path == null) return;
                    // endPose() throws if the path carries no heading interpolator. Better a command
                    // that finishes at once than an exception out of the OpMode loop.
                    try {
                        endPose[0] = path.endPose();
                    } catch (RuntimeException e) {
                        return;
                    }
                    follower.follow(path);
                    started[0] = true;
                })
                .setDone(() -> {
                    if (!started[0]) return true;
                    if (System.currentTimeMillis() - startedAt[0] < MIN_PATH_MS) return false;
                    return follower.atParametricEnd();
                })
                .setEnd(ec -> {
                    if (!started[0]) return;
                    if (ec == EndCondition.NATURALLY && holdEnd && endPose[0] != null) {
                        follower.hold(endPose[0], false);
                    } else {
                        cancelPath();
                    }
                })
                .requiring(this);
    }

    /**
     * Turns in place to an absolute field heading, then hands control back.
     *
     * <p>Pedro 3 has no turn primitive; a hold at the current position with the new heading is the
     * same thing. Done within {@link #TURN_TOLERANCE_RAD} or after {@link #TURN_TIMEOUT_MS}; either
     * way {@code setEnd} releases the hold, because a snap is something a driver does mid-drive and
     * wants the sticks back from immediately.
     */
    public Command turnToCommand(double headingRadians) {
        if (follower == null) return finishedCommand();
        final long[] startedAt = new long[1];
        return Command.build()
                .setStart(() -> {
                    startedAt[0] = System.currentTimeMillis();
                    holdHeading(headingRadians);
                })
                .setDone(() -> {
                    long elapsed = System.currentTimeMillis() - startedAt[0];
                    if (elapsed < MIN_PATH_MS) return false;
                    return atHeading(headingRadians, TURN_TOLERANCE_RAD) || elapsed >= TURN_TIMEOUT_MS;
                })
                .setEnd(ec -> cancelPath())
                .requiring(this);
    }

    /**
     * Drives at fixed robot-frame powers for {@code ms}, then hands control back. Bypasses
     * field-centric mixing and the heading hold on purpose, and works with or without a follower,
     * which is what lets the first-event autonomous leave the wall before anything is tuned.
     */
    public Command driveForMsCommand(double forward, double strafe, double turn, long ms) {
        final long[] startedAt = new long[1];
        return Command.build()
                .setStart(() -> {
                    startedAt[0] = System.currentTimeMillis();
                    drive(forward, strafe, turn);
                })
                .setExecute(() -> drive(forward, strafe, turn))
                .setDone(() -> System.currentTimeMillis() - startedAt[0] >= ms)
                .setEnd(ec -> cancelPath())
                .requiring(this);
    }

    /** A command that completes on its first tick. Returned when there is no follower to drive. */
    private static Command finishedCommand() {
        return Command.build().setDone(() -> true);
    }
}
