package org.firstinspires.ftc.teamcode.commands;

import static com.pedropathing.ivy.commands.Commands.instant;
import static com.pedropathing.ivy.commands.Commands.onInterrupt;
import static com.pedropathing.ivy.groups.Groups.deadline;
import static com.pedropathing.ivy.groups.Groups.sequential;

import com.pedropathing.api.Paths;
import com.pedropathing.ivy.Command;
import com.pedropathing.math.Pose;
import com.pedropathing.paths.Path;

import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.subsystems.Drivetrain;
import org.firstinspires.ftc.teamcode.subsystems.Limelight;
import org.firstinspires.ftc.teamcode.subsystems.Shooter;
import org.firstinspires.ftc.teamcode.util.math.Angles;

import java.util.Locale;
import java.util.function.BooleanSupplier;
import java.util.function.Supplier;

/**
 * The one-button actions that span more than one mechanism: shooting, aiming, and driving to a pose.
 *
 * <h2>Every macro is bounded and reports an outcome</h2>
 * Each is built through {@link #reporting}, so a timeout or an abort always leaves a terminal
 * {@link Outcome} rather than {@code RUNNING} for ever. The drivers read the outcome through the
 * rumble patterns, which is the only channel that works mid-match.
 *
 * <h2>The aim law, and why a front camera helps a rear shooter</h2>
 * The flywheel is bolted to the chassis firing out the rear ({@link Shooter#HEADING_OFFSET_RAD}), so
 * <em>aiming is turning the robot's back to the CELL</em>. The camera faces front, which means it sees
 * the HIVE's AprilTags only while the robot is facing the HIVE — exactly when the shooter is pointed
 * the wrong way. So {@link #aimHeading} records how far the tags disagree with the odometry bearing
 * while they are visible, and keeps applying that correction for {@link #AIM_BIAS_MAX_AGE_MS} after
 * they leave view, or until the pose is rewritten. Drive up facing the HIVE, take the correction,
 * turn, and shoot by it.
 *
 * <h2>Nothing counts pieces</h2>
 * There is no sensor in the intake or the tunnel, so "shoot all" means <b>fire
 * {@link #PIECES_PER_LOAD} pulses of the tunnel</b>, not "empty the robot". {@link #getShotsFired()}
 * is a pulse count. A pulse on an empty tunnel costs nothing; a robot that refused to shoot because
 * it could not count would be useless at an event.
 */
public class Macros {
    /**
     * How many tunnel pulses "shoot all" fires. Four, because BIOBUZZ G407 caps a robot at 4
     * controlled scoring elements and the robot pre-loads exactly that many.
     */
    public static int PIECES_PER_LOAD = 4;

    public static long SHOOT_ONE_TIMEOUT_MS = 4000;
    public static long SHOOT_ALL_TIMEOUT_MS = 12000;

    public static long AIM_TIMEOUT_MS = 2500;
    /** The robot counts as aimed within this of the wanted heading. */
    public static double AIM_TOLERANCE_DEGREES = 2.0;
    /** A one-shot aim re-issues Pedro's hold when the wanted heading moves by more than this. */
    public static double AIM_REISSUE_DEGREES = 1.0;
    /**
     * How long a tag-derived aim correction is kept after the tags leave view. Odometry heading
     * drifts little in a few seconds, and the target's placement error does not move at all, so the
     * correction stays good for about as long as it takes to turn and shoot. Five seconds is a
     * drive-up-and-turn; it is a strategy call, not a measurement.
     */
    public static long AIM_BIAS_MAX_AGE_MS = 5000;

    public static long SNAP_TIMEOUT_MS = 1500;
    public static double SNAP_TOLERANCE_DEGREES = 3.0;
    public static long DRIVE_TO_TIMEOUT_MS = 6000;
    public static double DRIVE_TO_TOLERANCE_INCHES = 3.0;
    /**
     * Shorter than this and no path is built. A line from the current pose to (almost) itself is
     * degenerate inside Pedro — a zero tangent, NaN powers that the motor layer drops on the floor —
     * so {@code followLazyCommand} is handed a null path, finishes at once, and the macro's own
     * position check reports the outcome.
     */
    public static double MIN_PATH_INCHES = 0.5;

    /** How the last macro finished. */
    public enum Outcome { IDLE, RUNNING, SUCCESS, TIMED_OUT, CANCELLED }

    private final Robot robot;
    private Outcome outcome = Outcome.IDLE;
    private String activeName = "idle";
    /** The cycle the last shooting macro ran, for its pulse count. */
    private Shoot lastShoot = null;

    /** The tag-derived aim correction, radians, and what it was taken against. NaN when none. */
    private double aimBias = Double.NaN;
    private long aimBiasAtMs = 0;
    private int aimBiasPoseWrites = -1;

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

    /** Tunnel pulses the last shooting macro ran. Not a piece count; nothing can know that. */
    public int getShotsFired() {
        return lastShoot == null ? 0 : lastShoot.getShotsFired();
    }

    public String getStatus() {
        return activeName + " : " + outcome;
    }

    /**
     * Marks a running macro CANCELLED. Every macro calls this itself when its group is ended from
     * outside ({@link #reporting}); {@code Robot.abortMacro()} calls it too, harmlessly, after the
     * follower cleanup that Ivy cannot do.
     */
    public void markCancelled() {
        // Both assignments inside the guard: a late callback from an already-finished macro must not
        // blank the name of one that is now running.
        if (outcome == Outcome.RUNNING) {
            outcome = Outcome.CANCELLED;
            activeName = "idle";
        }
    }

    // ---- Shooting ----

    /** One tunnel pulse into a spun-up flywheel. */
    public Command shootOne() {
        return shoot("shootOne", 1, SHOOT_ONE_TIMEOUT_MS);
    }

    /** {@link #PIECES_PER_LOAD} pulses, one spin-up, the flywheel recovering between each. */
    public Command shootAll() {
        return shoot("shootAll", PIECES_PER_LOAD, SHOOT_ALL_TIMEOUT_MS);
    }

    /**
     * The shooting macro. The flywheel hold rides <em>alongside</em> the cycle rather than inside it,
     * so it starts within {@code Scheduler.schedule()} — the same tick Ivy's OVERRIDE ends whatever
     * hold it displaced (the operator's armed flywheel). Without that the wheel would be told to stop
     * for a loop or two at the start of every shot.
     *
     * <p>The caller sets the flywheel target first, from the distance table or the manual override.
     */
    private Command shoot(String name, int pieces, long timeoutMs) {
        final Shoot cycle = new Shoot(robot.intake, robot.shooter, pieces);
        return reporting(name,
                Waits.bounded(cycle.build(), timeoutMs),
                () -> cycle.completed() ? Outcome.SUCCESS : Outcome.TIMED_OUT,
                instant(() -> lastShoot = cycle),
                robot.shooter.armedCommand());
    }

    /**
     * Aim, then shoot everything: autonomous's move. The flywheel spins up <em>during</em> the aim
     * rather than after it, and the aim is bounded like every other wait — if the robot cannot settle
     * within {@link #AIM_TOLERANCE_DEGREES} in time it shoots anyway, because in autonomous a piece
     * kept on board scores nothing and a near miss might.
     */
    public Command aimAndShootAll(Pose target, int minTagId, int maxTagId) {
        final Shoot cycle = new Shoot(robot.intake, robot.shooter, PIECES_PER_LOAD);
        return reporting("aimShootAll",
                sequential(
                        Waits.bounded(aimCore(target, minTagId, maxTagId), AIM_TIMEOUT_MS),
                        Waits.bounded(cycle.build(), SHOOT_ALL_TIMEOUT_MS)),
                () -> cycle.completed() ? Outcome.SUCCESS : Outcome.TIMED_OUT,
                instant(() -> lastShoot = cycle),
                robot.shooter.armedCommand());
    }

    // ---- Aiming ----

    /**
     * The field heading the robot must hold so the shooter faces {@code target}.
     *
     * <p>When a tag in {@code [minTagId, maxTagId]} is visible the bearing comes from its {@code tx}
     * (heading + camera yaw - tx; tx is positive to the right), and the difference between that and
     * the odometry bearing is remembered. When no tag is visible the odometry bearing is used, plus
     * that remembered correction while it is younger than {@link #AIM_BIAS_MAX_AGE_MS} and the pose
     * has not been rewritten since. {@link Shooter#HEADING_OFFSET_RAD} is then subtracted, so a
     * rear-firing shooter turns its back to the target. NaN without a pose, or with neither a target
     * nor a tag.
     */
    public double aimHeading(Pose target, int minTagId, int maxTagId) {
        Pose pose = robot.drivetrain.getPose();
        if (pose == null) return Double.NaN;
        double tx = robot.limelight.getTagTx(minTagId, maxTagId);
        double odometry = target == null ? Double.NaN
                : Math.atan2(target.y() - pose.y(), target.x() - pose.x());
        double bearing;
        if (!Double.isNaN(tx)) {
            bearing = pose.heading() + Math.toRadians(Limelight.CAMERA_YAW_OFFSET_DEGREES)
                    - Math.toRadians(tx);
            if (!Double.isNaN(odometry)) {
                aimBias = Angles.angleError(odometry, bearing);
                aimBiasAtMs = System.currentTimeMillis();
                aimBiasPoseWrites = robot.drivetrain.getPoseWrites();
            }
        } else {
            if (Double.isNaN(odometry)) return Double.NaN;
            bearing = odometry + (hasAimBias() ? aimBias : 0);
        }
        return Angles.normalizeAngle(bearing - Shooter.HEADING_OFFSET_RAD);
    }

    /** True while a tag-derived correction is in force: young enough, and the pose not rewritten since. */
    public boolean hasAimBias() {
        return !Double.isNaN(aimBias)
                && System.currentTimeMillis() - aimBiasAtMs <= AIM_BIAS_MAX_AGE_MS
                && robot.drivetrain.getPoseWrites() == aimBiasPoseWrites;
    }

    /** The correction in force, degrees (positive = the tags say the target is further CCW), or NaN. */
    public double getAimBiasDegrees() {
        return hasAimBias() ? Math.toDegrees(aimBias) : Double.NaN;
    }

    /**
     * Turns in place until the shooter faces {@code target} (tag-refined when one is visible), then
     * hands the sticks back.
     */
    public Command aimAt(Pose target, int minTagId, int maxTagId) {
        return reporting("aim",
                Waits.bounded(aimCore(target, minTagId, maxTagId), AIM_TIMEOUT_MS),
                Outcome.SUCCESS, Outcome.TIMED_OUT, () -> aimed(target, minTagId, maxTagId));
    }

    /**
     * Pedro's hold as the turn primitive, re-issued whenever the wanted heading moves by more than
     * {@link #AIM_REISSUE_DEGREES} so a newly visible tag can refine it. {@code setEnd} hands control
     * back either way.
     */
    private Command aimCore(Pose target, int minTagId, int maxTagId) {
        final long[] startedAt = new long[1];
        final double[] commanded = new double[1];
        return Command.build()
                .setStart(() -> {
                    startedAt[0] = System.currentTimeMillis();
                    commanded[0] = Double.NaN;
                    reissueAim(target, minTagId, maxTagId, commanded);
                })
                .setExecute(() -> reissueAim(target, minTagId, maxTagId, commanded))
                .setDone(() -> System.currentTimeMillis() - startedAt[0] >= Drivetrain.MIN_PATH_MS
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

    // ---- Heading and position ----

    /** Turns in place to an absolute heading; hands the follower back either way. */
    public Command snapToHeading(double headingRadians) {
        String name = String.format(Locale.US, "snapTo %.0f",
                Math.toDegrees(Angles.normalizeAngle(headingRadians)));
        return reporting(name,
                Waits.bounded(robot.drivetrain.turnToCommand(headingRadians), SNAP_TIMEOUT_MS),
                Outcome.SUCCESS, Outcome.TIMED_OUT,
                () -> robot.drivetrain.atHeading(headingRadians, Math.toRadians(SNAP_TOLERANCE_DEGREES)));
    }

    /**
     * Drives a straight Pedro path from the current pose to {@code target}, then hands the follower
     * back. The path is built at start, so the target may be computed any time earlier.
     * {@code holdEnd} is false: in teleop the driver-control default resumes the moment this releases
     * the drivetrain, and in auto the next leg follows.
     */
    public Command driveTo(Pose target) {
        return reporting("driveTo",
                Waits.bounded(robot.drivetrain.followLazyCommand(() -> lineTo(target), false),
                        DRIVE_TO_TIMEOUT_MS),
                Outcome.SUCCESS, Outcome.TIMED_OUT,
                () -> robot.drivetrain.atPose(target, DRIVE_TO_TOLERANCE_INCHES,
                        DRIVE_TO_TOLERANCE_INCHES));
    }

    // ---- Plumbing shared by every macro ----

    /**
     * How every macro is built: {@code begin(name)}, the body, then the outcome. If the group is ended
     * before {@code finishWith} runs (a {@link Waits#bounded} timeout, {@code Scheduler.cancel}, an
     * operator abort), {@code onInterrupt} marks it CANCELLED so no caller has to remember to.
     *
     * <p>It is {@code deadline(body, onInterrupt(...))} and not {@code setEnd} on the sequential: Ivy's
     * groups are {@code CommandBuilder}s whose {@code setEnd} <em>replaces</em> the group's own end —
     * the one that ends its children (docs/01 B.5 trap 12). A natural finish fires the callback too,
     * which is harmless, because a terminal outcome is never overwritten.
     *
     * <p>{@code alongside} commands run in parallel with the whole macro and are ended with it. They
     * start inside {@code Scheduler.schedule()}, before the first loop, which is what lets the
     * flywheel hold take over from a displaced hold with no gap.
     */
    private Command reporting(String name, Command body, Supplier<Outcome> result, Command... alongside) {
        Command[] children = new Command[alongside.length + 1];
        children[0] = onInterrupt(this::markCancelled);
        System.arraycopy(alongside, 0, children, 1, alongside.length);
        return deadline(sequential(begin(name), body, finishWith(result)), children);
    }

    private Command reporting(String name, Command body, Outcome success, Outcome failure,
                              BooleanSupplier ok) {
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

    /**
     * Straight line from the current pose to {@code target}; null without a pose, and null when the
     * target is within {@link #MIN_PATH_INCHES} (see that constant).
     */
    private Path lineTo(Pose target) {
        Pose current = robot.drivetrain.getPose();
        if (current == null || target == null) return null;
        if (current.distance(target) < MIN_PATH_INCHES) return null;
        return Paths.line(current, target).linear(current.heading(), target.heading());
    }
}
