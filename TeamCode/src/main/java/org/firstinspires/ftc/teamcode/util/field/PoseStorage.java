package org.firstinspires.ftc.teamcode.util.field;

import com.pedropathing.math.Pose;

/**
 * Carries the robot's pose, alliance and piece count from autonomous into teleop.
 *
 * <p>Without this, teleop starts with no idea where the robot is — and it defaults to field-centric
 * drive, which is the mode that depends most on a correct heading. The driver's first stick input
 * then sends the robot in an arbitrary direction.
 *
 * <p><b>Lifetime:</b> static state, so it survives between OpMode runs but <em>not</em> a Robot
 * Controller restart or an app crash. Teleop must therefore treat a missing value as normal and fall
 * back to AprilTag localisation or a manual heading reset — never assume this is populated.
 *
 * <p>Autonomous should call {@link #save} continuously rather than once at the end, so the handoff
 * still works if the OpMode is stopped early — which is exactly when a match is most chaotic.
 */
public final class PoseStorage {
    private static volatile Pose pose = null;
    private static volatile Alliance alliance = null;
    private static volatile StartPosition startPosition = null;
    /** Pieces still in the robot when the previous OpMode ended; {@code -1} = never recorded. */
    private static volatile int pieceCount = -1;

    private PoseStorage() {}

    /** Records the current pose and match setup. Safe to call every loop. */
    public static void save(Pose currentPose, Alliance currentAlliance, StartPosition start) {
        if (currentPose != null) pose = currentPose;
        if (currentAlliance != null) alliance = currentAlliance;
        if (start != null) startPosition = start;
    }

    /**
     * As {@link #save(Pose, Alliance, StartPosition)}, plus how many pieces the robot still holds.
     * Teleop seeds its storage count from it: a cut autonomous that shot two of four must not start
     * teleop believing the robot is empty (G407's interlock would then be two pieces off), and a
     * robot without an entrance sensor has no other way to know.
     */
    public static void save(Pose currentPose, Alliance currentAlliance, StartPosition start, int pieces) {
        save(currentPose, currentAlliance, start);
        if (pieces >= 0) pieceCount = pieces;
    }

    /** The pose left by the previous OpMode, or {@code null} if there isn't one. */
    public static Pose getPose() {
        return pose;
    }

    /** True when a previous OpMode left a usable pose behind. */
    public static boolean hasPose() {
        return pose != null;
    }

    /** True when autonomous ran and recorded which alliance we are on. */
    public static boolean hasAlliance() {
        return alliance != null;
    }

    /** The alliance chosen in autonomous, or {@code null} if autonomous never ran. */
    public static Alliance getAlliance() {
        return alliance;
    }

    /** True when a previous OpMode recorded how many pieces the robot holds. */
    public static boolean hasPieceCount() {
        return pieceCount >= 0;
    }

    /** Pieces left on board by the previous OpMode, or {@code -1} if never recorded. */
    public static int getPieceCount() {
        return pieceCount;
    }

    /** Forgets everything. Call when starting a genuinely new match. */
    public static void clear() {
        pose = null;
        alliance = null;
        startPosition = null;
        pieceCount = -1;
    }
}
