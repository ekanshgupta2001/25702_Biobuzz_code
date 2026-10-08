package org.firstinspires.ftc.teamcode.util.field;

import com.pedropathing.math.Pose;

/**
 * Carries the robot's pose from autonomous into teleop.
 *
 * <p>Without this, teleop starts with no idea where the robot is — and it defaults to field-centric
 * drive, which is the mode that depends most on a correct heading. The driver's first stick input
 * then sends the robot in an arbitrary direction.
 *
 * <h2>The alliance rides along, but only as a default</h2>
 * Autonomous records its side so Teleop's init can start its dpad alliance pick on it. The driver
 * still sees the alliance on the init card and changes it with the dpad, so a stale value from an
 * earlier run is one button press away from fixed.
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
    private static volatile Alliance alliance = null;

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

    /** Forgets the pose only. Teleop calls this once it has used it. */
    public static void clearPose() {
        pose = null;
    }

    /** Records which side autonomous ran on. */
    public static void saveAlliance(Alliance side) {
        alliance = side;
    }

    /** The side the last autonomous ran on, or {@code fallback} if none has run. */
    public static Alliance getAlliance(Alliance fallback) {
        return alliance == null ? fallback : alliance;
    }

    /** Forgets the stored pose and alliance. Call when starting a genuinely new match. */
    public static void clear() {
        pose = null;
        alliance = null;
    }
}
