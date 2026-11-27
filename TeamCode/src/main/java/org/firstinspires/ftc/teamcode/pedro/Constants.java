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
 * <h2>One Mecanum and one localizer, built once, shared</h2>
 * {@link #createMecanum} resolves the four drive motors and {@link #createLocalizer} the Pinpoint;
 * {@link #create} wraps <em>those same instances</em> in a {@link Follower}. This matters:
 * {@code Mecanum}'s constructor calls {@code hardwareMap.get(DcMotorEx.class, name)} itself and wraps
 * each motor in its own {@code CachedMotor}, so two {@code Mecanum} objects over the same four motors
 * hold two independent power caches and fight each other. A second {@code PinpointLocalizer} would
 * restart the Pinpoint's IMU calibration. Sharing one of each is what lets stick driving, the pose
 * and path following coexist in a single {@code Drivetrain}.
 *
 * <h2>The three configs</h2>
 * {@link #drivetrainConfig} is filled and checked ({@code Bench: Motors}). {@link #localizerConfig}
 * (the Pinpoint) and {@link #foresightConfig} (path following) are {@code null}: the robot drives
 * robot-centric straight through the {@code Mecanum} and has no pose. Both come from Pedro's AutoTune
 * tuners, which were removed from this build because BIOBUZZ R704 bans them in matches. To tune
 * after the event, restore {@code pedro/Tuning.java}, {@code pedro/procedures/} and the
 * {@code com.pedropathing:tuning} line in {@code build.dependencies.gradle} from git commit 6834106.
 */
public class Constants {
    /**
     * Motor names and directions. Left reversed / right forward is the usual mecanum wiring; confirm
     * each wheel pushes the robot forward with {@code Bench: Motors} and flip the offending direction
     * here. {@code HardcodedAuto} has its own copy of these four directions: change both.
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
     * The Pinpoint localizer on its own, or {@code null} until {@link #localizerConfig} is pasted in.
     * Needs no Foresight tuning, so the pose works as soon as the pods are tuned. Build it once: the
     * follower must be handed this same instance.
     */
    public static PinpointLocalizer createLocalizer(HardwareMap h) {
        if (localizerConfig == null) return null;
        return new PinpointLocalizer(h, localizerConfig);
    }

    /**
     * The follower over an existing {@link Mecanum} and localizer, or {@code null} without a localizer
     * or {@link #foresightConfig}. Constructor order is (localizer, drivetrain, algorithm) — the
     * Quickstart's own comment has it wrong (docs/01 A.9 gotcha 1).
     */
    public static Follower create(Mecanum mecanum, PinpointLocalizer localizer) {
        if (localizer == null || foresightConfig == null) return null;
        return new Follower(localizer, mecanum, new Foresight(foresightConfig));
    }
}
