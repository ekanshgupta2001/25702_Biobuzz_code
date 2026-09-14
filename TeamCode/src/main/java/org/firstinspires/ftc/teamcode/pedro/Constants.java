package org.firstinspires.ftc.teamcode.pedro;

import com.pedropathing.algorithm.Foresight;
import com.pedropathing.algorithm.ForesightConfig;
import com.pedropathing.follower.Follower;
import com.pedropathing.revhub.drivetrains.Mecanum;
import com.pedropathing.revhub.drivetrains.MecanumConfig;
import com.pedropathing.revhub.localizers.PinpointConfig;
import com.pedropathing.revhub.localizers.PinpointLocalizer;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.util.hardware.HardwareNames;

/**
 * Pedro 3.0.0 configuration for the V1 drivetrain.
 *
 * <p>Three configs, filled in the order AutoTune produces them (docs/01 section A.7, registration in
 * {@code Tuning}): {@link #drivetrainConfig} needs nothing from AutoTune beyond a direction check
 * (Mecanum Tuner, or the {@code Bench: Drive} OpMode); {@link #localizerConfig} is the Pinpoint
 * Tuner's output; {@link #foresightConfig} is the Foresight Tuner's. {@link #create} returns the
 * follower only once the last two exist. Until then {@code Drivetrain.isAvailable()} is false, teleop
 * drives open loop through {@code OpenLoopDrive}, the hardcoded auto leaves through it, and
 * everything else runs.
 */
public class Constants {
    /**
     * Motor names and directions. The left side reversed / right side forward is the usual mecanum
     * wiring; confirm with the Mecanum Tuner or {@code Bench: Drive} (each wheel must push the robot
     * forward) and flip the offending direction here. Shared by {@code OpenLoopDrive} now and by the
     * tuned follower later, so a direction verified once carries over.
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
     * The Pinpoint localizer, from the Pinpoint Tuner. {@code null} until its numbers are pasted in.
     * Template (replace every placeholder with the tuner's output; the name is already the one
     * the Robot Controller configuration must use):
     * <pre>
     * localizerConfig = new PinpointConfig(c -> {
     *     c.name.set(HardwareNames.PINPOINT);
     *     c.xPodDirection.set(GoBildaPinpointDriver.EncoderDirection.FORWARD);   // tuner
     *     c.yPodDirection.set(GoBildaPinpointDriver.EncoderDirection.FORWARD);   // tuner
     *     c.xPodOffset.set(0.0);                                                  // tuner, inches
     *     c.yPodOffset.set(0.0);                                                  // tuner, inches
     *     c.podType.set(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD);
     * });
     * </pre>
     * Constructing a {@code PinpointLocalizer} starts an IMU calibration that takes about a second;
     * {@code Drivetrain.onStart()} re-applies the init pose once it has settled (docs/01 A.9 gotcha 6).
     */
    public static PinpointConfig localizerConfig = null;

    /** The Foresight Tuner's block (its twelve required fields). {@code null} until pasted in. */
    public static ForesightConfig foresightConfig = null;

    /** True once both tuning configs are pasted in, so an init card can say why the drive is open loop. */
    public static boolean isTuned() {
        return localizerConfig != null && foresightConfig != null;
    }

    /**
     * The tuned follower, or {@code null} while either tuning config is missing. A null follower keeps
     * {@code Drivetrain.isAvailable()} false and everything else running. Constructor order is
     * (localizer, drivetrain, algorithm) (docs/01 A.9 gotcha 1).
     */
    public static Follower create(HardwareMap h) {
        if (!isTuned()) return null;
        return new Follower(
                new PinpointLocalizer(h, localizerConfig),
                new Mecanum(h, drivetrainConfig),
                new Foresight(foresightConfig));
    }
}
