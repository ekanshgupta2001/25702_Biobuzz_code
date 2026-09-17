package org.firstinspires.ftc.teamcode.subsystems;

import com.pedropathing.follower.Follower;
import com.pedropathing.math.Pose;
import com.pedropathing.paths.Path;

/**
 * The slice of Pedro 3.0.0's {@link Follower} that {@link Drivetrain} actually drives.
 *
 * <p>Exists so the drivetrain's command logic can run on the JVM against a fake. A real
 * {@code Follower} needs motors and a localizer to construct, which would put every path command,
 * the hold semantics and the heading hold beyond the reach of a unit test. Everything the
 * drivetrain calls goes through this interface; {@link PedroPathFollower} is the production
 * implementation, and callers that need Pedro's full API (path introspection, error readouts) get
 * the raw follower from {@link Drivetrain#getFollower()}.
 *
 * <p>The contract mirrors Pedro exactly, including its surprises: {@link #atParametricEnd()} is
 * the "path finished" signal and is true whenever the follower is not following;
 * {@link #follow} always restarts from the start of the path; and the mode is implied by the last
 * command ({@code follow}, {@code hold}, {@code manual}).
 */
public interface PathFollower {
    /** Advances the follower one loop. Exactly once per loop; it also ticks the localizer. */
    void update();

    /** Inches and radians, heading normalised to {@code [0, 2pi)}. */
    Pose pose();

    /** Writes the localizer's estimate. There is no separate "starting pose" in Pedro 3. */
    void setPose(Pose pose);

    /**
     * Drives with robot-frame powers and puts the follower in MANUAL mode. Pedro's convention:
     * {@code +strafe} is robot-left and {@code +turn} is counter-clockwise. Powers are latched, so
     * call it every loop while driving.
     */
    void manual(double forward, double strafe, double turn);

    /** Starts following from {@code t = 0}. The path must carry a heading interpolator. */
    void follow(Path path);

    /** True once the path geometry is finished, and whenever the follower is not following. */
    boolean atParametricEnd();

    /** Station-keeps at a pose. {@code scaled} applies the gentler hold gains. */
    void hold(Pose pose, boolean scaled);

    Follower.Mode mode();
}
