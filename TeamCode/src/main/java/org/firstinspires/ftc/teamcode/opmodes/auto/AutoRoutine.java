package org.firstinspires.ftc.teamcode.opmodes.auto;

import static com.pedropathing.ivy.commands.Commands.instant;
import static com.pedropathing.ivy.groups.Groups.sequential;

import com.pedropathing.ivy.Command;

import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.commands.Waits;
import org.firstinspires.ftc.teamcode.game.Field;

import java.util.ArrayList;
import java.util.Collections;
import java.util.List;

/**
 * The first-competition autonomous: shoot the four pre-loaded POLLEN into the up-facing CELL, then
 * drive off the wall for LEAVE. Hardcoded and open-loop on purpose: it runs before the Pedro
 * follower is tuned, so there is no localizer, no path and no aiming. The robot is placed by hand
 * touching its wall with the shooter (the rear) toward the CELL.
 *
 * <p>Built as one Ivy {@code sequential} that requires every mechanism for the whole run (docs/03
 * section 5), separate from the OpMode so {@code AutoRoutineTest} can run it on the JVM. Every step
 * is bounded on the injected clock. LEAVE happens whether or not the shooting succeeded: a robot
 * that left with the balls still on board beats one stuck against the wall.
 *
 * <p>The Pedro-path version (drive to the shooting spot, {@code aimAndShootAll}, park) replaces
 * this once the drivetrain is tuned; see HANDOFF.
 */
public final class AutoRoutine {
    /** Which way "off the wall" is, in the robot's frame. */
    public enum LeaveDirection { FORWARD, BACKWARD, LEFT, RIGHT }

    /** Upper bound on the shooting phase; {@code Macros.SHOOT_ALL_TIMEOUT_MS} bounds it from inside too. */
    public static long SHOOT_BUDGET_MS = 20000;
    /** Pause after the last shot before moving, so the last piece clears the flywheel. */
    public static long SETTLE_MS = 250;
    /**
     * The front is against the wall (the shooter fires out the rear), so away from the wall is
     * BACKWARD: toward, but well short of, the HIVE structure. Change to a strafe if the placement
     * differs.
     */
    public static LeaveDirection LEAVE_DIRECTION = LeaveDirection.BACKWARD;
    public static double LEAVE_POWER = 0.3;
    /** Long enough to clearly stop touching the wall; not long enough to reach anything. */
    public static long LEAVE_MS = 800;

    private final Robot robot;
    private String phase = "not started";
    private final List<String> log = new ArrayList<>();

    public AutoRoutine(Robot robot) {
        this.robot = robot;
    }

    /** The routine. Schedule it once from the OpMode's {@code start()}. */
    public Command build() {
        double[] leave = leavePowers(LEAVE_DIRECTION, LEAVE_POWER);
        return sequential(
                instant(() -> {
                    setPhase("shoot");
                    robot.storage.setCount(Field.PRELOADED_POLLEN_PER_ROBOT);
                }),
                Waits.bounded(robot.getClock(), robot.macros.shootAll(), SHOOT_BUDGET_MS),
                instant(() -> {
                    if (robot.macros.isRunning()) robot.macros.markCancelled();   // the budget cut it
                    log.add("shots " + robot.macros.getShotsFired() + " : " + robot.macros.getOutcome());
                }),
                Waits.waitMs(robot.getClock(), SETTLE_MS),
                instant(() -> setPhase("leave")),
                leaveCommand(leave),
                instant(() -> {
                    robot.stopMechanisms();
                    setPhase("done");
                }))
                .requiring(robot.intake, robot.storage, robot.transfer, robot.shooter,
                        robot.openLoopDrive, robot.drivetrain);
    }

    /**
     * The LEAVE move through whichever motor layer exists: the tuned follower once
     * {@code Constants.create()} returns one, otherwise the open-loop drive. Decided when the routine
     * is built (in {@code start()}), when availability is known. {@code Robot} guarantees only one
     * of the two is fitted.
     */
    private Command leaveCommand(double[] powers) {
        return robot.drivetrain.isAvailable()
                ? robot.drivetrain.driveForMsCommand(powers[0], powers[1], powers[2], LEAVE_MS)
                : robot.openLoopDrive.driveForMsCommand(powers[0], powers[1], powers[2], LEAVE_MS);
    }

    /** {@code {forward, strafe, turn}} for a leave direction, in Pedro's robot-frame convention. */
    public static double[] leavePowers(LeaveDirection direction, double power) {
        switch (direction) {
            case FORWARD:
                return new double[] {power, 0, 0};
            case BACKWARD:
                return new double[] {-power, 0, 0};
            case LEFT:
                return new double[] {0, power, 0};
            case RIGHT:
            default:
                return new double[] {0, -power, 0};
        }
    }

    private void setPhase(String name) {
        phase = name;
        log.add(name);
    }

    public String getPhase() {
        return phase;
    }

    public List<String> getLog() {
        return Collections.unmodifiableList(log);
    }
}
