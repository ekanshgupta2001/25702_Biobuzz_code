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
 * The shooter is bolted to the chassis and fires out the rear, so the drivetrain heading <em>is</em>
 * the aim. What the aim law needs from the field is therefore small: where the target CELL opening
 * sits, and which AprilTag IDs are stuck underneath it. The robot has no game-piece sensors and
 * counts nothing, does not visit the GARDEN, and does not reason about FLOWERS — so those numbers
 * left with the code that used them. Adding a member back is cheap; leaving an unread constant in a
 * file whose whole job is to be the trusted source of field truth is not.
 */
public final class Field {
    private Field() {}

    // ---- Frame and tiles ----

    public static final double FIELD_SIZE_INCHES = FieldConstants.FIELD_SIZE_INCHES;
    public static final double TILE_INCHES = 24.0;
    public static final int TILES_PER_SIDE = 6;

    /**
     * Centre of a tile by column letter A-F (left to right from the audience) and row 1-6 (from
     * the audience wall to the far wall). Heading 0.
     */
    public static Pose tileCenter(char column, int row) {
        int col = Character.toUpperCase(column) - 'A';
        if (col < 0 || col >= TILES_PER_SIDE || row < 1 || row > TILES_PER_SIDE) {
            throw new IllegalArgumentException("no such tile: " + column + row);
        }
        return new Pose(TILE_INCHES / 2 + TILE_INCHES * col, TILE_INCHES / 2 + TILE_INCHES * (row - 1));
    }

    /**
     * The field heading a driver at the alliance station calls "forward": straight away from their
     * own wall. The blue wall is the +X (column F) wall, so blue drivers face -X and red drivers +X.
     * Feeds {@code Drivetrain.setDriverHeadingOffset}; without it a blue driver's stick-up drove the
     * robot toward themselves once the pose was in the true field frame (fixthese B3).
     */
    public static double driverForwardHeading(Alliance alliance) {
        return alliance == Alliance.BLUE ? Math.PI : 0;
    }

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

    /** The up-facing CELL after {@code tipsSoFar} TIPs of that alliance's HIVE: it flips each time. */
    public static CellSide upCellSide(Alliance alliance, int tipsSoFar) {
        CellSide start = startingUpCellSide(alliance);
        return tipsSoFar % 2 == 0 ? start : start.opposite();
    }

    // ---- AprilTags (manual section 9.9, figures 9-15 and 9-17; SDK v12.0 notes) ----

    /**
     * Inclusive tag ID range on the given CELL: red audience-side 34-37, red far-side 30-33, blue
     * audience-side 38-41, blue far-side 42-45. 36h11 tags, in clusters of four on the underside of
     * each CELL, facing down. The cluster origin is the centre of the CELL opening, so a detection's
     * bearing points straight at the mouth.
     *
     * <p>This is the only tag fact the robot needs: the Limelight is told which IDs belong to the
     * CELL being shot at and ignores every other detection. Nothing localises from a tag, so there
     * is no need for the full ID span of the field.
     */
    public static int[] tagRange(Alliance alliance, CellSide side) {
        if (alliance == Alliance.RED) {
            return side == CellSide.AUDIENCE ? new int[] {34, 37} : new int[] {30, 33};
        }
        return side == CellSide.AUDIENCE ? new int[] {38, 41} : new int[] {42, 45};
    }

    // ---- LOADING ZONE (manual section 9.3, Setup Guide section 8): where PARK is ----

    public static final double LOADING_ZONE_DEPTH_INCHES = 11.0;

    /**
     * Centre of the blue LOADING ZONE on tile F2. INFERRED placement: against the blue wall,
     * centred on the tile's Y span. PARK is at least partially inside it.
     */
    public static Pose blueLoadingZoneCenter() {
        return new Pose(FIELD_SIZE_INCHES - LOADING_ZONE_DEPTH_INCHES / 2, tileCenter('F', 2).y());
    }
}
