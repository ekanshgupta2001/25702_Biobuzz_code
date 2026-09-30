package org.firstinspires.ftc.teamcode.opmodes.test;

import com.pedropathing.ivy.Scheduler;
import com.pedropathing.math.Pose;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.game.Field;
import org.firstinspires.ftc.teamcode.opmodes.MatchOpMode;
import org.firstinspires.ftc.teamcode.subsystems.Drivetrain;
import org.firstinspires.ftc.teamcode.util.field.Alliance;
import org.firstinspires.ftc.teamcode.util.math.DriveScaling;
import org.firstinspires.ftc.teamcode.util.time.MatchClock;

/**
 * Tries out the driver's side of the drivetrain: stick shaping (deadband, expo, slow mode),
 * field-centric against robot-centric, and the heading hold.
 *
 * <p>It drives through the same default command {@code Teleop} schedules, with the same sticks, so
 * what feels right here is what the match OpMode will do. The only difference is that the shaping
 * numbers can be changed on the pad and are shown on the card at every stage: raw stick, after
 * deadband and expo, after the slow-mode scale.
 *
 * <p>Field-centric needs a pose, so before AutoTune has produced the follower the robot is
 * robot-centric only and the heading hold is off; the card says so. The alliance matters only for
 * which way "forward" points in field-centric: pick it with dpad left (RED) / right (BLUE) during init.
 *
 * <p>Every static this bench changes is put back in {@link #onStop()}, so a bench session cannot carry
 * into a match run on the same Robot Controller (docs/03 §18).
 */
@TeleOp(name = "Bench: Drive", group = "Bench")
public class DriveBench extends MatchOpMode {
    public static double DEADBAND_STEP = 0.01;
    public static double EXPO_STEP = 0.25;

    private Alliance side = Alliance.BLUE;

    private double deadbandAtStart;
    private double expoAtStart;
    private boolean headingHoldAtStart;

    @Override
    protected Alliance alliance() {
        return Alliance.BLUE;   // the Robot's own; this bench keeps its choice in `side`
    }

    @Override
    protected MatchClock.Period matchPeriod() {
        return MatchClock.Period.TELEOP;
    }

    @Override
    protected void onInit() {
        deadbandAtStart = DriveScaling.DEFAULT_DEADBAND;
        expoAtStart = DriveScaling.DEFAULT_EXPO;
        headingHoldAtStart = Drivetrain.HEADING_HOLD_ENABLED;

        // Exactly Teleop's drive default: shaped, slow-scaled sticks, heading hold inside Drivetrain.
        Scheduler.schedule(robot.drivetrain.driverControlCommand(
                () -> scaled(forwardRaw()), () -> scaled(strafeRaw()), () -> scaled(turnRaw())));
        robot.drivetrain.setDriverHeadingOffset(Field.driverForwardHeading(side));
    }

    @Override
    protected void onInitLoop() {
        if (gamepad1.dpadLeftWasPressed()) side = Alliance.RED;
        if (gamepad1.dpadRightWasPressed()) side = Alliance.BLUE;
        robot.drivetrain.setDriverHeadingOffset(Field.driverForwardHeading(side));

        telemetry.addData("Alliance", "%s   (dpad left = RED, right = BLUE)", side);
        telemetry.addLine("Only sets which way is 'forward' in field-centric.");
        telemetry.addLine(robot.drivetrain.hasFollower()
                ? "Pedro tuned: field-centric and heading hold available."
                : "!! Not tuned: robot-centric only, no heading hold.");
    }

    // Sticks as Controls defines them: the SDK reports stick-up as negative, and Pedro's +strafe is
    // left and +turn is counter-clockwise, so all three are negated.
    private double forwardRaw() { return -gamepad1.left_stick_y; }
    private double strafeRaw()  { return -gamepad1.left_stick_x; }
    private double turnRaw()    { return -gamepad1.right_stick_x; }

    private double scaled(double raw) {
        return DriveScaling.shape(raw) * DriveScaling.slowScale(gamepad1.left_trigger);
    }

    @Override
    protected void onDecide() {
        if (gamepad1.leftBumperWasPressed()) robot.drivetrain.toggleFieldCentric();
        if (gamepad1.yWasPressed()) robot.drivetrain.resetHeading();
        if (gamepad1.aWasPressed()) {
            Drivetrain.HEADING_HOLD_ENABLED = !Drivetrain.HEADING_HOLD_ENABLED;
            robot.drivetrain.releaseHeadingHold();
        }
        if (gamepad1.xWasPressed()) {
            DriveScaling.DEFAULT_DEADBAND = deadbandAtStart;
            DriveScaling.DEFAULT_EXPO = expoAtStart;
        }
        if (gamepad1.dpadUpWasPressed()) {
            DriveScaling.DEFAULT_DEADBAND = Math.min(0.5, DriveScaling.DEFAULT_DEADBAND + DEADBAND_STEP);
        }
        if (gamepad1.dpadDownWasPressed()) {
            DriveScaling.DEFAULT_DEADBAND = Math.max(0, DriveScaling.DEFAULT_DEADBAND - DEADBAND_STEP);
        }
        if (gamepad1.dpadRightWasPressed()) DriveScaling.DEFAULT_EXPO += EXPO_STEP;
        if (gamepad1.dpadLeftWasPressed()) {
            DriveScaling.DEFAULT_EXPO = Math.max(1, DriveScaling.DEFAULT_EXPO - EXPO_STEP);
        }
    }

    @Override
    protected void onTelemetry() {
        telemetry.addData("Loop", "%.0f Hz", robot.getLoopHz());
        telemetry.addLine("LB=field/robot  Y=re-zero heading  A=heading hold  L-trig=slow");
        telemetry.addLine("dpad up/down=deadband  left/right=expo  X=reset scaling");
        telemetry.addLine();

        telemetry.addData("Frame", robot.drivetrain.isFieldCentric() ? "FIELD centric (" + side + ")"
                : robot.drivetrain.hasFollower() ? "ROBOT centric" : "ROBOT centric (not tuned)");
        telemetry.addData("Scaling", "deadband %.2f   expo %.2f   slow x%.2f",
                DriveScaling.DEFAULT_DEADBAND, DriveScaling.DEFAULT_EXPO,
                DriveScaling.slowScale(gamepad1.left_trigger));
        axisRow("forward", forwardRaw());
        axisRow("strafe", strafeRaw());
        axisRow("turn", turnRaw());
        telemetry.addLine();

        Pose pose = robot.drivetrain.getPose();
        if (pose == null) {
            telemetry.addData("Pose", "none (not tuned)");
        } else {
            telemetry.addData("Pose", "x %.1f  y %.1f  heading %.1f deg",
                    pose.x(), pose.y(), Math.toDegrees(pose.heading()));
        }
        double held = robot.drivetrain.getHeldHeading();
        telemetry.addData("Heading hold", "%s   %s",
                Drivetrain.HEADING_HOLD_ENABLED && robot.drivetrain.hasFollower() ? "ON" : "OFF",
                Double.isNaN(held) ? "not holding" : String.format("holding %.1f deg", Math.toDegrees(held)));
        telemetry.addData("battery", "%.1f V", robot.getBatteryVolts());
    }

    private void axisRow(String name, double raw) {
        double shaped = DriveScaling.shape(raw);
        telemetry.addData("  " + name, "raw %+.2f -> shaped %+.2f -> out %+.2f",
                raw, shaped, shaped * DriveScaling.slowScale(gamepad1.left_trigger));
    }

    @Override
    protected void onStop() {
        // No motor writes here: the SDK rejects them from stop() and zeroes the motors itself.
        DriveScaling.DEFAULT_DEADBAND = deadbandAtStart;
        DriveScaling.DEFAULT_EXPO = expoAtStart;
        Drivetrain.HEADING_HOLD_ENABLED = headingHoldAtStart;
    }
}
