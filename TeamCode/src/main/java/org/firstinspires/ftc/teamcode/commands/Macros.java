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
import org.firstinspires.ftc.teamcode.subsystems.Shooter;
import org.firstinspires.ftc.teamcode.util.math.Angles;

import java.util.function.BooleanSupplier;
import java.util.function.Supplier;

/**
 * The one-button actions that span more than one mechanism: shooting, aiming, and driving to a pose.
 *
 * <h2>Every macro is bounded and reports an outcome</h2>
 * Each is built through {@link #reporting}, so a timeout or an abort always leaves a terminal
 * {@link Outcome} rather than {@code RUNNING} for ever, shown on the telemetry card by
 * {@link #getStatus()}.
 *
 * <h2>Aiming</h2>
 * The flywheel is bolted to the chassis firing out the rear ({@link Shooter#HEADING_OFFSET_RAD}), so
 * <em>aiming is turning the robot's back to the CELL</em>. {@link #aimHeading} takes the bearing from
 * odometry alone; the Limelight is not used.
 *
 * <h2>Nothing counts pieces</h2>
 * There is no sensor in the intake or the tunnel, so a shot is <b>a length of time feeding</b>, not
 * "empty the robot": Teleop's {@link #shoot()} feeds until the driver stops it, autonomous's
 * {@link #shootAll()} for {@link Shoot#AUTO_FEED_MS}. Feeding an empty tunnel costs nothing.
 */
public class Macros {
    /**
     * Teleop's shot stops by itself after this if X never comes. A backstop for a jam: anti-jam is off
     * while feeding, so a jammed ball would otherwise stall the intake until someone notices.
     */
    public static long SHOOT_TIMEOUT_MS = 12000;
    /** Bound on autonomous's timed shot: spin-up + {@link Shoot#AUTO_FEED_MS} + dwell fit well inside it. */
    public static long SHOOT_ALL_TIMEOUT_MS = 12000;

    public static long AIM_TIMEOUT_MS = 2500;
    /** The robot counts as aimed within this of the wanted heading. */
    public static double AIM_TOLERANCE_DEGREES = 2.0;
    /** A one-shot aim re-issues Pedro's hold when the wanted heading moves by more than this. */
    public static double AIM_REISSUE_DEGREES = 1.0;

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

    /** The step the last shooting macro is on (SPIN_UP, FEED, DWELL, DONE), or "-". */
    public String getShotState() {
        return lastShoot == null ? "-" : lastShoot.getStateName();
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

    /**
     * Teleop's shot: spin up once, then feed nonstop until the command is cancelled (X), or until
     * {@link #SHOOT_TIMEOUT_MS} as a jam backstop. Cancelled = CANCELLED, backstop = TIMED_OUT.
     */
    public Command shoot() {
        return shoot("shoot", Shoot.UNTIL_CANCELLED, SHOOT_TIMEOUT_MS);
    }

    /** Autonomous's shot: spin up once, feed nonstop for {@link Shoot#AUTO_FEED_MS}, done. */
    public Command shootAll() {
        return shoot("shootAll", Shoot.AUTO_FEED_MS, SHOOT_ALL_TIMEOUT_MS);
    }

    /**
     * The shooting macro. The flywheel hold rides <em>alongside</em> the cycle rather than inside it,
     * so it starts within {@code Scheduler.schedule()} — the same tick Ivy's OVERRIDE ends whatever
     * hold it displaced (the operator's armed flywheel). Without that the wheel would be told to stop
     * for a loop or two at the start of every shot.
     *
     * <p>The wheels run at the Shooter's fixed speeds ({@code Shooter.TARGET_TICKS_PER_SEC} and
     * {@code ROLLER_TARGET_TICKS_PER_SEC}).
     */
    private Command shoot(String name, long feedMs, long timeoutMs) {
        final Shoot cycle = new Shoot(robot.intake, robot.shooter, feedMs);
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
    public Command aimAndShootAll(Pose target) {
        final Shoot cycle = new Shoot(robot.intake, robot.shooter, Shoot.AUTO_FEED_MS);
        return reporting("aimShootAll",
                sequential(
                        Waits.bounded(aimCore(target), AIM_TIMEOUT_MS),
                        Waits.bounded(cycle.build(), SHOOT_ALL_TIMEOUT_MS)),
                () -> cycle.completed() ? Outcome.SUCCESS : Outcome.TIMED_OUT,
                instant(() -> lastShoot = cycle),
                robot.shooter.armedCommand());
    }

    // ---- Aiming ----

    /**
     * The field heading the robot must hold so the shooter faces {@code target}: the odometry bearing
     * to it, minus {@link Shooter#HEADING_OFFSET_RAD} so a rear-firing shooter turns its back to the
     * target. NaN without a pose or a target.
     */
    public double aimHeading(Pose target) {
        Pose pose = robot.drivetrain.getPose();
        if (pose == null || target == null) return Double.NaN;
        double bearing = Math.atan2(target.y() - pose.y(), target.x() - pose.x());
        return Angles.normalizeAngle(bearing - Shooter.HEADING_OFFSET_RAD);
    }

    /**
     * Pedro's hold as the turn primitive, re-issued whenever the wanted heading moves by more than
     * {@link #AIM_REISSUE_DEGREES}. {@code setEnd} hands control back either way.
     */
    private Command aimCore(Pose target) {
        final long[] startedAt = new long[1];
        final double[] commanded = new double[1];
        return Command.build()
                .setStart(() -> {
                    startedAt[0] = System.currentTimeMillis();
                    commanded[0] = Double.NaN;
                    reissueAim(target, commanded);
                })
                .setExecute(() -> reissueAim(target, commanded))
                .setDone(() -> System.currentTimeMillis() - startedAt[0] >= Drivetrain.MIN_PATH_MS
                        && aimed(target))
                .setEnd(ec -> robot.drivetrain.cancelPath())
                .requiring(robot.drivetrain);
    }

    private void reissueAim(Pose target, double[] commanded) {
        double wanted = aimHeading(target);
        if (Double.isNaN(wanted)) return;
        if (!Double.isNaN(commanded[0])
                && Math.abs(Angles.angleError(commanded[0], wanted)) < Math.toRadians(AIM_REISSUE_DEGREES)) {
            return;
        }
        commanded[0] = wanted;
        robot.drivetrain.holdHeading(wanted);
    }

    private boolean aimed(Pose target) {
        double wanted = aimHeading(target);
        return !Double.isNaN(wanted)
                && robot.drivetrain.atHeading(wanted, Math.toRadians(AIM_TOLERANCE_DEGREES));
    }

    // ---- Position ----

    /**
     * Drives a straight Pedro path from the current pose to {@code target}, then hands the follower
     * back. The path is built at start, so the target may be computed any time earlier.
     * {@code holdEnd} is false: in auto the next leg follows straight on.
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
