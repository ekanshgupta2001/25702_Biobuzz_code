package org.firstinspires.ftc.teamcode.opmodes.auto;

import static com.pedropathing.ivy.commands.Commands.instant;
import static com.pedropathing.ivy.groups.Groups.sequential;

import com.pedropathing.ivy.Scheduler;
import com.pedropathing.math.Pose;

import org.firstinspires.ftc.teamcode.commands.Waits;
import org.firstinspires.ftc.teamcode.game.Field;
import org.firstinspires.ftc.teamcode.game.FieldPoses;
import org.firstinspires.ftc.teamcode.opmodes.MatchOpMode;
import org.firstinspires.ftc.teamcode.util.field.Alliance;
import org.firstinspires.ftc.teamcode.util.field.FieldConstants;
import org.firstinspires.ftc.teamcode.util.field.PoseStorage;
import org.firstinspires.ftc.teamcode.util.time.MatchClock;

/**
 * The 30-second autonomous: shoot the pre-loads into the up-CELL, then get off the wall.
 *
 * <h2>Two routines, chosen by whether Pedro is tuned</h2>
 * <ul>
 *   <li><b>Tuned</b> — aim at the CELL with the tag-refined aim law, shoot, then drive a path to
 *       park in the LOADING ZONE. 20 (TIP) + 3 (LEAVE) + 5 (PARK) are all on the table.</li>
 *   <li><b>Not tuned</b> — shoot from where the robot was placed (rear toward the CELL), then back
 *       off the wall on a timer. Worth 20 + 3 and needs nothing measured. This is what runs at the
 *       first event if AutoTune has not happened yet.</li>
 * </ul>
 * There is no alliance selector and no lock: {@code BlueAuto} and {@code RedAuto} carry the side in
 * the OpMode name, so there is no way to start the wrong route and no "started UNLOCKED" state to
 * warn about.
 *
 * <h2>The first TIP only needs three POLLEN</h2>
 * The up-CELL starts with 3 NECTAR and the robot pre-loads 4 POLLEN, and a HIVE tips on 3 POLLEN + 3
 * NECTAR. Three shots that land is a TIP, which is why this routine spends its budget on shooting
 * rather than on driving.
 */
public abstract class Auto extends MatchOpMode {
    /** Cap on the shooting phase, so a jam cannot eat the whole period. */
    public static long SHOOT_BUDGET_MS = 15000;
    /** Pause after shooting, so the robot is still before it drives. */
    public static long SETTLE_MS = 250;
    /**
     * Backwards off the wall. Negative is away from the wall, given the robot starts with its
     * rear-firing shooter pointed at the HIVE. Set these on the practice field: the robot must clearly
     * stop touching the wall and must never reach the HIVE structure.
     */
    public static double LEAVE_POWER = -0.4;
    public static long LEAVE_MS = 700;

    private final Alliance side;

    protected Auto(Alliance side) {
        this.side = side;
    }

    @Override
    protected final Alliance alliance() {
        return side;
    }

    @Override
    protected final MatchClock.Period matchPeriod() {
        return MatchClock.Period.AUTONOMOUS;
    }

    @Override
    protected void onInit() {
        Scheduler.schedule(robot.intake.defaultIdleCommand(), robot.shooter.defaultIdleCommand());
    }

    @Override
    protected void onInitLoop() {
        telemetry.addLine(side + " autonomous: shoot the pre-loads, then leave");
        telemetry.addLine();
        telemetry.addLine("PLACE: touching your wall, REAR (shooter) toward the up-facing CELL.");
        telemetry.addData("Route", robot.drivetrain.hasFollower()
                ? "tuned: aim, shoot, path to park"
                : "NOT TUNED: shoot in place, then back off the wall on a timer");
        telemetry.addData("Flywheel", robot.shooter.getStatusLine());
        if (!robot.limelight.isConnected()) telemetry.addLine("!! no Limelight: odometry aim only");
    }

    @Override
    protected void onStart() {
        Field.CellSide up = Field.startingUpCellSide(side);
        Pose target = Field.cell(side, up);
        int[] tags = Field.tagRange(side, up);

        if (robot.drivetrain.hasFollower()) {
            Pose start = FieldConstants.forAlliance(FieldPoses.BLUE_START_FACING_HIVE, side);
            robot.drivetrain.setPose(start);
            robot.shooter.setTargetForDistance(start.distance(target));
            Scheduler.schedule(sequential(
                    Waits.bounded(robot.macros.aimAndShootAll(target, tags[0], tags[1]), SHOOT_BUDGET_MS),
                    Waits.waitMs(SETTLE_MS),
                    robot.macros.driveTo(FieldConstants.forAlliance(FieldPoses.BLUE_PARK, side)),
                    instant(robot::stopMechanisms)));
        } else {
            // No localizer, so no distance to look up: the manual speed is the only number available.
            robot.shooter.setManualTarget();
            Scheduler.schedule(sequential(
                    Waits.bounded(robot.macros.shootAll(), SHOOT_BUDGET_MS),
                    Waits.waitMs(SETTLE_MS),
                    robot.drivetrain.driveForMsCommand(LEAVE_POWER, 0, 0, LEAVE_MS),
                    instant(robot::stopMechanisms)));
        }
    }

    @Override
    protected void onDecide() {
        // The buzzer safety net. BIOBUZZ G403 forbids powered movement after the period ends, and a
        // macro still inside its timeout does not know the match is over.
        MatchClock clock = robot.getMatchClock();
        if (clock != null && clock.isExpired()) {
            robot.abortMacro();
            robot.stopMechanisms();
        }
    }

    @Override
    protected void onAfterAct() {
        // Written every loop, not once at the end: a cut or disabled autonomous still hands teleop a
        // pose, so field-centric drive and the aim law work from the first second.
        Pose pose = robot.drivetrain.getPose();
        if (pose != null) PoseStorage.save(pose);
    }

    @Override
    protected void onTelemetry() {
        MatchClock clock = robot.getMatchClock();
        telemetry.addData("Loop", "%.0f Hz", robot.getLoopHz());
        telemetry.addData("Time", clock == null ? "-" : clock.getStatus());
        telemetry.addData("Macro", "%s  (%d pulses)", robot.macros.getStatus(),
                robot.macros.getShotsFired());
        telemetry.addData("Flywheel", robot.shooter.getStatusLine());
        Pose pose = robot.drivetrain.getPose();
        telemetry.addData("Pose", pose == null ? "none (not tuned)"
                : String.format("%.1f, %.1f @ %.0f deg", pose.x(), pose.y(),
                        Math.toDegrees(pose.heading())));
    }
}
