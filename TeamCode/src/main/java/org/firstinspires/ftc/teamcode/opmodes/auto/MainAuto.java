package org.firstinspires.ftc.teamcode.opmodes.auto;

import com.pedropathing.ivy.Command;
import com.pedropathing.ivy.Scheduler;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.game.Field;
import org.firstinspires.ftc.teamcode.game.FieldPoses;
import org.firstinspires.ftc.teamcode.opmodes.MatchOpMode;
import org.firstinspires.ftc.teamcode.util.field.FieldConstants;
import org.firstinspires.ftc.teamcode.util.field.PoseStorage;
import org.firstinspires.ftc.teamcode.util.time.MatchClock;

/**
 * The autonomous OpMode. Runs {@link AutoSelector} during init, schedules {@link AutoRoutine} on
 * start, writes {@link PoseStorage} every loop so teleop inherits the alliance, the piece count
 * (and the pose, once there is one), and stops everything at the buzzer.
 *
 * <p>This is the hardcoded first-competition version: no localizer, no paths. The robot is placed
 * touching its wall with the shooter (rear) toward the up-facing CELL; the routine shoots the four
 * pre-loads and drives off the wall for LEAVE.
 */
@Autonomous(name = "Auto: shoot 4 + leave", group = "Main", preselectTeleOp = "Teleop")
public class MainAuto extends MatchOpMode {
    private final AutoSelector selector = new AutoSelector();
    private AutoRoutine routine;
    private Command routineCommand;
    private boolean stoppedAtBuzzer = false;

    @Override
    protected String logTag() {
        return "auto";
    }

    @Override
    protected MatchClock.Period matchPeriod() {
        return MatchClock.Period.AUTONOMOUS;
    }

    @Override
    protected void onInit() {
        robot.intake.defaultIdleCommand().schedule();
        robot.storage.defaultIdleCommand().schedule();
        robot.transfer.defaultIdleCommand().schedule();
        robot.shooter.defaultIdleCommand().schedule();
        robot.openLoopDrive.defaultStopCommand().schedule();
        robot.storage.setCount(Field.PRELOADED_POLLEN_PER_ROBOT);
    }

    @Override
    protected void onInitLoop() {
        selector.poll(gamepad1);
        robot.setAlliance(selector.getAlliance());      // G408: which NECTAR is the opponent's
        reportMissingHardware();
        if (robot.drivetrain.isAvailable() && !robot.drivetrain.isLocalizerSettled()) {
            telemetry.addLine("!! Localizer calibrating: wait a second before START");
        }
        telemetry.addData("Setup", selector.status());
        telemetry.addLine("Place the robot touching the wall, REAR (shooter) toward the up CELL.");
        telemetry.addData("Pre-loads", robot.storage.count() + " POLLEN");
        telemetry.addData("Sensors", robot.sensingSummary());
        telemetry.addData("Plan", "shoot all, settle, leave " + AutoRoutine.LEAVE_DIRECTION
                + " for " + AutoRoutine.LEAVE_MS + " ms at " + AutoRoutine.LEAVE_POWER);
        telemetry.addData("Drive", robot.drivetrain.isAvailable() ? "Pedro follower"
                : (robot.openLoopDrive.isAvailable() ? "open loop (Pedro not tuned)" : "!! NO DRIVE MOTORS"));
        if (loggerError != null) telemetry.addData("!! Logger FAILED", loggerError);
    }

    @Override
    protected void onStart() {
        // Where the robot was placed, in the true field frame. A no-op without a follower; once
        // Constants.create() is real this is what makes teleop's inherited pose mean something.
        robot.drivetrain.setPose(FieldConstants.forAlliance(
                FieldPoses.startPose(selector.getStart()), selector.getAlliance()));
        routine = new AutoRoutine(robot);
        routineCommand = routine.build();
        routineCommand.schedule();
    }

    @Override
    protected void onDecide() {
        // G403: nothing may move in the transition. The routine is budgeted to finish well before
        // 30 s; this is the safety net if a tunable ever pushes it over.
        MatchClock clock = robot.getMatchClock();
        if (clock != null && clock.isExpired() && routineCommand != null
                && Scheduler.isScheduled(routineCommand)) {
            Scheduler.cancel(routineCommand);
            robot.abortMacro();
            robot.stopMechanisms();
            stoppedAtBuzzer = true;
        }
    }

    @Override
    protected void onAfterAct() {
        // Every loop, not once at the end: if this OpMode is stopped early, teleop still inherits.
        PoseStorage.save(robot.drivetrain.getPose(), selector.getAlliance(), selector.getStart(),
                robot.storage.count());
    }

    @Override
    protected void onStop() {
        PoseStorage.save(robot.drivetrain.getPose(), selector.getAlliance(), selector.getStart(),
                robot.storage.count());
    }

    @Override
    protected void onTelemetry() {
        MatchClock clock = robot.getMatchClock();
        telemetry.addData("Time", clock == null ? "-" : clock.getStatus());
        telemetry.addData("Phase", routine == null ? "-" : routine.getPhase());
        telemetry.addData("Shots", robot.macros.getShotsFired() + "  (" + robot.macros.getStatus() + ")");
        telemetry.addData("Pieces left", robot.storage.count());
        if (stoppedAtBuzzer) telemetry.addLine("!! STOPPED at the buzzer before the routine finished");
        for (String line : robot.getMissingHardware()) telemetry.addData("!! MISSING", line);
    }
}
