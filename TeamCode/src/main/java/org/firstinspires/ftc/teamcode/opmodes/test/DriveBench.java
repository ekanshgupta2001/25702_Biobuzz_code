package org.firstinspires.ftc.teamcode.opmodes.test;

import com.pedropathing.math.Pose;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.subsystems.Drivetrain;

/**
 * Drives the robot with Pedro, the same way Teleop does, and shows the localization.
 *
 * <p>Gamepad 1: sticks drive, OPTIONS switches field / robot centric, Y zeroes the pose.
 * Field centric and the pose need Pedro tuned (Pinpoint + Foresight configs in {@code Constants});
 * before that it drives robot centric only. The pose starts at 0, 0, 0 where the robot sits at
 * init, so field-centric "forward" is the way it faced then (or when Y was last pressed).
 */
@TeleOp(name = "Bench: Drive", group = "Bench")
public class DriveBench extends OpMode {
    private Drivetrain drivetrain;

    @Override
    public void init() {
        drivetrain = new Drivetrain(hardwareMap);
        // The Pinpoint keeps its pose between OpModes; start this bench from a known zero.
        drivetrain.setPose(new Pose(0, 0, 0));
    }

    @Override
    public void start() {
        drivetrain.onStart();
    }

    @Override
    public void loop() {
        if (gamepad1.optionsWasPressed()) drivetrain.toggleFieldCentric();
        if (gamepad1.yWasPressed()) drivetrain.setPose(new Pose(0, 0, 0));

        // The SDK reports stick-up as negative; Pedro wants +forward, +strafe left, +turn CCW.
        drivetrain.drive(-gamepad1.left_stick_y, -gamepad1.left_stick_x, -gamepad1.right_stick_x);
        drivetrain.update();

        telemetry.addLine("OPTIONS = field/robot centric   Y = zero the pose");
        telemetry.addData("Mode", drivetrain.isFieldCentric() ? "FIELD centric" : "ROBOT centric");
        Pose pose = drivetrain.getPose();
        if (pose == null) {
            telemetry.addLine("Pedro not tuned: no localization yet (run the Pinpoint Tuner)");
        } else {
            telemetry.addData("x", "%.2f in", pose.x());
            telemetry.addData("y", "%.2f in", pose.y());
            telemetry.addData("heading", "%.1f deg", Math.toDegrees(pose.heading()));
        }
    }
}
