package org.firstinspires.ftc.teamcode.pedro;

import com.pedropathing.follower.Follower;
import com.pedropathing.revhub.drivetrains.MecanumConfig;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.util.hardware.HardwareNames;

/**
 * Pedro 3.0.0 configuration for the V1 drivetrain.
 *
 * <p>Only the motor layer is filled in. {@link #drivetrainConfig} needs nothing from AutoTune:
 * four config names (from {@link HardwareNames}) and four directions. It is what
 * {@code subsystems/OpenLoopDrive} drives through before the follower is tuned, and it is the same
 * object the tuned follower will use, so directions verified now carry over.
 *
 * <p>{@link #create} stays {@code null} until the Pinpoint and Foresight configs come out of the
 * AutoTune web UI (docs/01 section A.7, worked example in section A.1). A null follower keeps
 * {@code Drivetrain.isAvailable()} false and everything else running.
 */
public class Constants {
    /**
     * Motor names and directions. The left side reversed / right side forward is the usual mecanum
     * wiring; confirm with the Mecanum Tuner (each wheel should drive the robot forward) and flip
     * the offending direction here.
     */
    public static MecanumConfig drivetrainConfig = new MecanumConfig(c -> {
        c.frontLeftName.set(HardwareNames.FRONT_LEFT_MOTOR);
        c.backLeftName.set(HardwareNames.BACK_LEFT_MOTOR);
        c.frontRightName.set(HardwareNames.FRONT_RIGHT_MOTOR);
        c.backRightName.set(HardwareNames.BACK_RIGHT_MOTOR);
        c.frontLeftDirection.set(DcMotorSimple.Direction.REVERSE);
        c.backLeftDirection.set(DcMotorSimple.Direction.REVERSE);
        c.frontRightDirection.set(DcMotorSimple.Direction.FORWARD);
        c.backRightDirection.set(DcMotorSimple.Direction.FORWARD);
        c.manualBrakeMode.set(true);
    });

    /**
     * The tuned follower: {@code new Follower(new PinpointLocalizer(h, localizerConfig),
     * new Mecanum(h, drivetrainConfig), new Foresight(foresightConfig))} once those two configs
     * exist. Until then null, on purpose.
     */
    public static Follower create(HardwareMap h) {
        return null;
    }
}
