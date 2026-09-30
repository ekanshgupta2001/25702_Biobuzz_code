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
 * Pedro 3.0.0 configuration for the mecanum drivetrain.
 *
 * <h2>One Mecanum, built once, shared</h2>
 * {@link #createMecanum} resolves the four drive motors; {@link #create} wraps <em>that same
 * instance</em> in a {@link Follower}. This matters: {@code Mecanum}'s constructor calls
 * {@code hardwareMap.get(DcMotorEx.class, name)} itself and wraps each motor in its own
 * {@code CachedMotor}, so two {@code Mecanum} objects over the same four motors hold two independent
 * power caches and fight each other. Sharing one instance is what lets stick driving and path
 * following coexist in a single {@code Drivetrain} instead of the two parallel motor layers this
 * code used to carry.
 *
 * <h2>Filling the three configs</h2>
 * In the order AutoTune produces them, (docs/01 section A.7):
 * {@link #drivetrainConfig} needs only a direction check (Mecanum Tuner, or the SDK's TestHardware
 * utility); {@link #localizerConfig} is the Pinpoint Tuner's output; {@link #foresightConfig} is the
 * Foresight Tuner's. Until the last two exist {@link #create} returns {@code null} and the robot
 * drives robot-centric straight through the {@code Mecanum}, with no heading hold, no aim lock and
 * no paths. Everything else runs normally.
 */
public class Constants {
    /**
     * Motor names and directions. Left reversed / right forward is the usual mecanum wiring; confirm
     * each wheel pushes the robot forward with the SDK's TestHardware utility or the Mecanum Tuner,
     * and flip the offending direction here. A direction verified once carries into the tuned
     * follower, because the follower drives this very config.
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
     * Template (the name is already the one the Robot Controller configuration must use):
     * <pre>
     * localizerConfig = new PinpointConfig(c -&gt; {
     *     c.name.set(HardwareNames.PINPOINT);
     *     c.xPodDirection.set(GoBildaPinpointDriver.EncoderDirection.FORWARD);   // tuner
     *     c.yPodDirection.set(GoBildaPinpointDriver.EncoderDirection.FORWARD);   // tuner
     *     c.xPodOffset.set(0.0);                                                 // tuner, inches
     *     c.yPodOffset.set(0.0);                                                 // tuner, inches
     *     c.podType.set(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD);
     * });
     * </pre>
     * Constructing a {@code PinpointLocalizer} starts an IMU calibration of about a second, so a pose
     * written during init is lost; {@code Drivetrain} repeats it once the window has passed
     * (docs/01 A.9 gotcha 6).
     */
    public static PinpointConfig localizerConfig = null;

    /** The Foresight Tuner's block, all twelve required fields. {@code null} until pasted in. */
    public static ForesightConfig foresightConfig = null;

    /** True once both tuning configs are pasted in, so an init card can say why paths are off. */
    public static boolean isTuned() {
        return localizerConfig != null && foresightConfig != null;
    }

    /**
     * The motor layer. Always constructible: it needs only names and directions, so stick driving
     * works before anything is tuned. Throws if a drive motor name is missing from the
     * configuration, which is the intended loud failure.
     */
    public static Mecanum createMecanum(HardwareMap h) {
        return new Mecanum(h, drivetrainConfig);
    }

    /**
     * The follower over an existing {@link Mecanum}, or {@code null} while either tuning config is
     * missing. Constructor order is (localizer, drivetrain, algorithm) — the Quickstart's own comment
     * has it wrong (docs/01 A.9 gotcha 1).
     */
    public static Follower create(HardwareMap h, Mecanum mecanum) {
        if (!isTuned()) return null;
        return new Follower(
                new PinpointLocalizer(h, localizerConfig),
                mecanum,
                new Foresight(foresightConfig));
    }
}
