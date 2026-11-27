package org.firstinspires.ftc.teamcode.subsystems;

import com.pedropathing.drivetrain.DrivePowers;
import com.pedropathing.follower.Follower;
import com.pedropathing.follower.ManualDrive;
import com.pedropathing.math.Pose;
import com.pedropathing.revhub.drivetrains.Mecanum;
import com.pedropathing.revhub.localizers.PinpointLocalizer;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.pedro.Constants;

/**
 * The mecanum drivetrain: stick driving and the pose.
 *
 * <h2>One motor layer, one localizer</h2>
 * This class owns exactly one {@link Mecanum}, built from names and directions alone, and at most one
 * {@link PinpointLocalizer}, and hands those same instances to the {@link Follower} when the tuning
 * configs exist. So:
 * <ul>
 *   <li><b>Sticks always work, robot-centric by default.</b> With a follower they go through
 *       {@code follower.manual(...)} (field-centric only after {@link #toggleFieldCentric()}).
 *       Without one they go straight to {@code mecanum.drive(...)}, robot-centric.</li>
 *   <li><b>The pose needs only the localizer</b>: once {@code Constants.localizerConfig} is filled,
 *       {@link #getPose()} reads the Pinpoint, with or without a follower.</li>
 * </ul>
 * Today both configs are null, so the sticks go straight to the motors and there is no pose. Never
 * build a second {@code Mecanum} over the same four motors: each keeps its own power cache and they
 * fight.
 *
 * <h2>Who drives it</h2>
 * Teleop calls {@link #drive} every loop. The hardcoded autos do not use this class at all.
 */
public class Drivetrain {
    /**
     * How long a freshly constructed Pinpoint spends recalibrating its IMU. A pose written before
     * that is lost (docs/01 A.9 gotcha 6), so a {@link #setPose} inside the window is repeated once by
     * {@link #update()} afterwards.
     */
    public static long LOCALIZER_SETTLE_MS = 1000;

    /** The motor layer. Never null: four names and four directions are all it needs. */
    private final Mecanum mecanum;
    /** The Pinpoint. Null until {@code Constants.localizerConfig} is filled. */
    private final PinpointLocalizer localizer;
    /**
     * Null until both tuning configs are filled. Drives {@link #mecanum} and reads {@link #localizer},
     * never its own copies.
     */
    private final Follower follower;

    /** Robot-centric unless {@link #toggleFieldCentric()} is called; field-centric needs the follower. */
    private boolean fieldCentric = false;

    private final long builtAtMs;
    private Pose lastSetPose = null;
    private boolean reapplyWhenSettled = false;

    public Drivetrain(HardwareMap hardwareMap) {
        this.builtAtMs = System.currentTimeMillis();
        this.mecanum = Constants.createMecanum(hardwareMap);
        this.localizer = Constants.createLocalizer(hardwareMap);
        this.follower = Constants.create(mecanum, localizer);   // the same localizer, not a second one
    }

    /** True once both tuning configs exist, so field-centric driving is available. */
    public boolean hasFollower() {
        return follower != null;
    }

    /** True once {@code Constants.localizerConfig} exists, so {@link #getPose()} has a pose. */
    public boolean hasLocalizer() {
        return localizer != null;
    }

    /**
     * Call once from the OpMode's {@code start()}: hands the follower to the sticks, manual mode at
     * zero power. A pose written during init is repeated by the first {@link #update()} after
     * {@link #LOCALIZER_SETTLE_MS}, whether START came early or late.
     */
    public void onStart() {
        if (follower != null) follower.manual(0, 0, 0);
        else mecanum.stop();
    }

    /** True once the localizer's IMU calibration window has passed (always true without a localizer). */
    public boolean isLocalizerSettled() {
        return localizer == null || System.currentTimeMillis() - builtAtMs >= LOCALIZER_SETTLE_MS;
    }

    /**
     * Applies stick powers. Convention (Pedro's): {@code +forward} ahead, {@code +strafe} left,
     * {@code +turn} counter-clockwise. Powers are latched, so this is called every loop while driving.
     *
     * <p>Field-centric needs a pose, so it only applies once there is a follower; before that the
     * sticks are robot-centric and the drive-frame toggle has no effect.
     */
    public void drive(double forward, double strafe, double turn) {
        if (follower == null) {
            mecanum.drive(new DrivePowers(forward, strafe, turn), true);
            return;
        }
        Pose pose = follower.pose();
        if (fieldCentric && pose != null) {
            DrivePowers p = ManualDrive.fieldCentric(forward, strafe, turn, pose.heading());
            follower.manual(p.forward(), p.strafe(), p.turn());
        } else {
            follower.manual(forward, strafe, turn);
        }
    }

    public void toggleFieldCentric() {
        fieldCentric = !fieldCentric;
    }

    /** True when sticks are interpreted in the field frame. Always false without a follower. */
    public boolean isFieldCentric() {
        return fieldCentric && follower != null;
    }

    /**
     * Writes the pose estimate, at init or from an external fix.
     *
     * <p>A write inside {@link #LOCALIZER_SETTLE_MS} of construction lands during the Pinpoint's IMU
     * calibration and is lost, so it is repeated once by {@link #update()} after the window. If the
     * robot has already moved by then (START within a second of INIT), that second of motion is
     * discarded in exchange for a heading that is right for the rest of the match.
     */
    public void setPose(Pose pose) {
        if (localizer == null || pose == null) return;
        writePose(pose);
        lastSetPose = pose;
        reapplyWhenSettled = !isLocalizerSettled();
    }

    private void writePose(Pose pose) {
        if (follower != null) follower.setPose(pose);
        else localizer.setPose(pose);
    }

    /** The pose estimate, or {@code null} until {@code Constants.localizerConfig} is filled. */
    public Pose getPose() {
        if (follower != null) return follower.pose();
        if (localizer != null) return localizer.pose();
        return null;
    }

    /**
     * Ticks odometry once per loop: {@code follower.update()}, which ticks the localizer itself, or
     * the localizer alone when there is no follower. Without a follower {@link #drive} has already
     * written the motors directly.
     */
    public void update() {
        if (localizer == null) return;   // no localizer means no follower either: nothing to tick
        if (reapplyWhenSettled && isLocalizerSettled()) {
            reapplyWhenSettled = false;
            if (lastSetPose != null) writePose(lastSetPose);
        }
        if (follower != null) follower.update();
        else localizer.update();   // never both: that would tick the Pinpoint twice a loop
    }
}
