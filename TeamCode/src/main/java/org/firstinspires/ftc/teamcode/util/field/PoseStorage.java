package org.firstinspires.ftc.teamcode.util.field;

import com.pedropathing.math.Pose;

/**
 * Carries the robot's pose from autonomous into teleop.
 *
 * <p>Without this, teleop starts with no idea where the robot is — and it defaults to field-centric
 * drive, which is the mode that depends most on a correct heading. The driver's first stick input
 * then sends the robot in an arbitrary direction.
 *
 * <h2>The pose is the only thing that crosses the gap</h2>
 * This used to carry an alliance and a piece count as well. Neither has anywhere to come from now:
 *
 * <ul>
 *   <li>The alliance is baked into the OpMode class — {@code BlueTeleop} and {@code RedTeleop} are
 *       separate entries on the Driver Station — so it is chosen by the driver at the moment of
 *       selection and cannot be inherited stale from whatever ran last. That is strictly safer than
 *       surviving an OpMode switch.</li>
 *   <li>There are no game-piece sensors on the robot and nothing counts pieces, so there is no
 *       count to hand over.</li>
 * </ul>
 *
 * <p>It also carried a start position that was written on every save and read by nothing: no getter
 * for it ever existed. Handoff state that no one reads is a lie about what the handoff contains, so
 * it is gone with the rest.
 *
 * <h2>Lifetime</h2>
 * Static state, so it survives between OpMode runs but <em>not</em> a Robot Controller restart or an
 * app crash. Teleop must treat a missing pose as normal and fall back to a manual heading reset —
 * never assume this is populated. The front camera aims at tags; it does not localise, so there is
 * no vision fallback to lean on.
 *
 * <p>Autonomous should call {@link #save} continuously rather than once at the end, so the handoff
 * still works if the OpMode is stopped early — which is exactly when a match is most chaotic.
 */
public final class PoseStorage {
    private static volatile Pose pose = null;

    private PoseStorage() {}

    /** Records the current pose. Safe to call every loop. A null pose is ignored, never stored. */
    public static void save(Pose currentPose) {
        if (currentPose != null) pose = currentPose;
    }

    /** The pose left by the previous OpMode, or {@code null} if there isn't one. */
    public static Pose getPose() {
        return pose;
    }

    /** True when a previous OpMode left a usable pose behind. */
    public static boolean hasPose() {
        return pose != null;
    }

    /** Forgets the stored pose. Call when starting a genuinely new match. */
    public static void clear() {
        pose = null;
    }
}
