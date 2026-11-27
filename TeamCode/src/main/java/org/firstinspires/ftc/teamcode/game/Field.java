package org.firstinspires.ftc.teamcode.game;

import com.pedropathing.math.Pose;

import org.firstinspires.ftc.teamcode.util.field.Alliance;
import org.firstinspires.ftc.teamcode.util.field.FieldConstants;

/**
 * The BIOBUZZ field, in the robot's coordinate frame: only what the robot code uses. Sourced from
 * {@code docs/04-biobuzz-season-analysis.md} (Competition Manual V1, Event Field Setup Guide V1.0,
 * SDK v12.0 notes); every number tagged INFERRED there is tagged INFERRED here and listed in
 * HANDOFF section 9 to be confirmed against the Onshape field CAD. Figure-read dimensions carry the
 * manual's +/-1 in caveat.
 *
 * <h2>Frame (this is the one place it is written down)</h2>
 * Origin at the <b>A1 corner: the audience-side red corner</b>. +X runs along the audience wall
 * toward column F (the audience's right, the blue alliance wall); +Y runs away from the audience
 * toward row 6 (the far wall). Inches. Heading in radians, counter-clockwise from +X, normalised
 * to {@code [0, 2pi)} by Pedro. So red territory is low X, blue is high X; the audience is low Y,
 * the far wall is high Y; tile (column, row) has its centre at
 * {@code (12 + 24 * col, 12 + 24 * (row - 1))} with A = 0.
 *
 * <p>The check that pins the origin: the field is 180-degree rotationally symmetric, and
 * {@code (x, y) -> (144 - x, 144 - y)} must map the red LOADING ZONE on A5 onto the blue one on F2.
 * {@link FieldConstants#forAlliance} does that conversion for every blue-authored value here, and
 * {@code FieldConstants} carries the evidence for the rotation.
 *
 * <p>Everything alliance-specific is authored for BLUE and converted on demand.
 *
 * <h2>Why so little of the field is described here</h2>
 * The code needs two things from the field: where each CELL sits (for the position distance, once
 * the Pinpoint is set up) and which AprilTag IDs are on it (so the camera's tag can be matched to a
 * CELL). Nothing else is read, so nothing else is here.
 */
public final class Field {
    private Field() {}

    // ---- Frame and tiles ----

    public static final double FIELD_SIZE_INCHES = FieldConstants.FIELD_SIZE_INCHES;
    public static final double TILE_INCHES = 24.0;

    // ---- HIVE structure (manual section 9.6, figures 9-15 and 10-2) ----

    /** The HIVE structure sits on the centre four tiles C3, C4, D3, D4. */
    public static final Pose HIVE_CENTER = new Pose(FIELD_SIZE_INCHES / 2, FIELD_SIZE_INCHES / 2);
    /** Distance between the two CELLs of one HIVE, along Y (audience side to far side). */
    public static final double CELL_SPACING_INCHES = 18.8;
    /**
     * INFERRED: each HIVE's centre sits this far from the field centre along X (half of one HIVE's
     * share of the 49.46 in frame). Red HIVE on the column-C side (low X), blue on column D.
     */
    public static double HIVE_LATERAL_OFFSET_INCHES = 12.4;

    /** Which of a HIVE's two CELLs. */
    public enum CellSide {
        /** The CELL nearer the audience wall (low Y). */
        AUDIENCE,
        /** The CELL nearer the far wall (high Y). */
        FAR;

        public CellSide opposite() {
            return this == AUDIENCE ? FAR : AUDIENCE;
        }
    }

    /** Centre of the blue HIVE's pivot, on the column-D side of the field centre. */
    public static Pose blueHiveCenter() {
        return new Pose(HIVE_CENTER.x() + HIVE_LATERAL_OFFSET_INCHES, HIVE_CENTER.y());
    }

    /** Ground-plane centre of one blue CELL opening (heading 0). */
    public static Pose blueCell(CellSide side) {
        double dy = CELL_SPACING_INCHES / 2;
        Pose hive = blueHiveCenter();
        return new Pose(hive.x(), side == CellSide.FAR ? hive.y() + dy : hive.y() - dy);
    }

    /**
     * Ground-plane centre of a CELL opening for either alliance. Under the 180-degree field
     * rotation the blue FAR CELL becomes the red AUDIENCE CELL, so the side is flipped before
     * converting.
     */
    public static Pose cell(Alliance alliance, CellSide side) {
        if (alliance == Alliance.BLUE) return blueCell(side);
        return FieldConstants.forAlliance(blueCell(side.opposite()), alliance);
    }

    /** The CELL that starts the match up-facing: red audience-side, blue far-side (figure 10-2). */
    public static CellSide startingUpCellSide(Alliance alliance) {
        return alliance == Alliance.RED ? CellSide.AUDIENCE : CellSide.FAR;
    }

    // ---- AprilTags (manual section 9.9, figures 9-15 and 9-17; SDK v12.0 notes) ----

    /**
     * Inclusive tag ID range on the given CELL: red audience-side 34-37, red far-side 30-33, blue
     * audience-side 38-41, blue far-side 42-45. 36h11 tags, in clusters of four on the underside of
     * each CELL, facing down. The cluster origin is the centre of the CELL opening, so a detection's
     * bearing points straight at the mouth.
     *
     * <p>{@code Robot} uses it to tell which of our two CELLs the camera's tag is on, and to ignore
     * a tag that is on neither.
     */
    public static int[] tagRange(Alliance alliance, CellSide side) {
        if (alliance == Alliance.RED) {
            return side == CellSide.AUDIENCE ? new int[] {34, 37} : new int[] {30, 33};
        }
        return side == CellSide.AUDIENCE ? new int[] {38, 41} : new int[] {42, 45};
    }
}
