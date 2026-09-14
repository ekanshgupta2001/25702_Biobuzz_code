package org.firstinspires.ftc.teamcode.game;

import com.pedropathing.math.Pose;

import org.firstinspires.ftc.teamcode.util.field.Alliance;
import org.firstinspires.ftc.teamcode.util.field.FieldConstants;

/**
 * The BIOBUZZ field, in the robot's coordinate frame. Sourced from
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
 * <p>The check that pins the origin: the field is 180-degree rotationally symmetric
 * ({@link FieldConstants#SYMMETRY}), and {@code (x, y) -> (144 - x, 144 - y)} must map the red
 * LOADING ZONE on A5 onto the blue one on F2 and the red GARDEN on A1 onto the blue one on F6.
 * {@link FieldConstants#forAlliance} does that conversion for every blue-authored value here.
 *
 * <p>Everything alliance-specific is authored for BLUE and converted on demand. Fixed elements
 * (HIVE centre, the four FLOWERs) are given explicitly.
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

    /** An axis-aligned rectangle on the field, inches, corners inclusive. Immutable. */
    public static final class Zone {
        public final double xMin;
        public final double yMin;
        public final double xMax;
        public final double yMax;

        public Zone(double x1, double y1, double x2, double y2) {
            xMin = Math.min(x1, x2);
            xMax = Math.max(x1, x2);
            yMin = Math.min(y1, y2);
            yMax = Math.max(y1, y2);
        }

        public boolean contains(double x, double y) {
            return x >= xMin && x <= xMax && y >= yMin && y <= yMax;
        }

        public boolean contains(Pose pose) {
            return pose != null && contains(pose.x(), pose.y());
        }

        /** True when a square of the given half-size centred on {@code pose} overlaps this zone. */
        public boolean overlapsSquare(Pose pose, double halfSizeInches) {
            if (pose == null) return false;
            return pose.x() + halfSizeInches >= xMin && pose.x() - halfSizeInches <= xMax
                    && pose.y() + halfSizeInches >= yMin && pose.y() - halfSizeInches <= yMax;
        }

        public Pose center() {
            return new Pose((xMin + xMax) / 2, (yMin + yMax) / 2);
        }

        public double width() {
            return xMax - xMin;
        }

        public double height() {
            return yMax - yMin;
        }

        /** This blue-authored zone for the given alliance, via {@link FieldConstants#forAlliance}. */
        public Zone forAlliance(Alliance alliance) {
            Pose a = FieldConstants.forAlliance(new Pose(xMin, yMin), alliance);
            Pose b = FieldConstants.forAlliance(new Pose(xMax, yMax), alliance);
            return new Zone(a.x(), a.y(), b.x(), b.y());
        }

        @Override
        public String toString() {
            return "Zone[" + xMin + ".." + xMax + ", " + yMin + ".." + yMax + "]";
        }
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
    /** Frame footprint: width across X (alliance to alliance), depth across Y. */
    public static final double HIVE_FRAME_WIDTH_INCHES = 49.46;
    public static final double HIVE_FRAME_DEPTH_INCHES = 38.95;
    /** Pivot axis height above the tiles. */
    public static final double HIVE_PIVOT_HEIGHT_INCHES = 43.95;
    /** Distance between the two CELLs of one HIVE, along Y (audience side to far side). */
    public static final double CELL_SPACING_INCHES = 18.8;
    public static final double CELL_OPENING_WIDTH_INCHES = 20.0;
    public static final double CELL_OPENING_HEIGHT_INCHES = 14.0;
    public static final double CELL_OPENING_DEPTH_INCHES = 12.0;
    /**
     * INFERRED: height of the up-facing CELL's opening centre above the tiles, pivot plus about
     * half the CELL spacing. Confirm in the Onshape CAD before fixing shooter geometry.
     */
    public static double UP_CELL_OPENING_HEIGHT_INCHES = 53.0;
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

    /** Ground-plane centre of the CELL the alliance shoots into first. */
    public static Pose firstTargetCell(Alliance alliance) {
        return cell(alliance, startingUpCellSide(alliance));
    }

    // ---- AprilTags (manual section 9.9, figures 9-15 and 9-17; SDK v12.0 notes) ----

    /** 36h11, 3.25 in, in clusters of four on the underside of each CELL, facing down. */
    public static final double TAG_SIZE_INCHES = 3.25;
    public static final int MIN_TAG_ID = 30;
    public static final int MAX_TAG_ID = 45;
    /** Cluster geometry (figure 9-15): tag offsets from the cluster centreline. */
    public static final double CLUSTER_INNER_TAG_OFFSET_INCHES = 2.75;
    public static final double CLUSTER_OUTER_TAG_OFFSET_INCHES = 6.5;
    public static final double CLUSTER_REFERENCE_HOLE_OFFSET_INCHES = 7.0;
    public static final double CLUSTER_CENTRELINE_ABOVE_HOLES_INCHES = 2.75;
    public static final double CELL_FRONT_TO_HOLE_CENTRELINE_INCHES = 9.938;

    /**
     * Inclusive tag ID range on the given CELL: red audience-side 34-37, red far-side 30-33, blue
     * audience-side 38-41, blue far-side 42-45. The cluster origin is the centre of the CELL
     * opening, so a detection's bearing points straight at the mouth.
     */
    public static int[] tagRange(Alliance alliance, CellSide side) {
        if (alliance == Alliance.RED) {
            return side == CellSide.AUDIENCE ? new int[] {34, 37} : new int[] {30, 33};
        }
        return side == CellSide.AUDIENCE ? new int[] {38, 41} : new int[] {42, 45};
    }

    /** Tag range on the CELL the alliance shoots into first. */
    public static int[] firstTargetTags(Alliance alliance) {
        return tagRange(alliance, startingUpCellSide(alliance));
    }

    // ---- FLOWERs (manual section 9.7; positions read from figures, INFERRED, +/-1 in) ----

    /** INFERRED: how far the FLOWER's top opening centre sits inside the wall it is mounted on. */
    public static double FLOWER_INSET_INCHES = 3.0;
    public static final double FLOWER_TOP_OPENING_DIAMETER_INCHES = 4.0;
    public static final double FLOWER_TOP_OPENING_HEIGHT_INCHES = 21.5;
    public static final double FLOWER_BACKSTOP_INCHES = 1.25;
    /** Bottom retrieval opening: only POLLEN (2.8 in) fits; NECTAR (3.6 in) does not. */
    public static final double FLOWER_RETRIEVAL_OPENING_HEIGHT_INCHES = 3.55;
    public static final double FLOWER_RETRIEVAL_OPENING_DEPTH_INCHES = 3.57;
    public static final double FLOWER_FLOOR_RING_HEIGHT_INCHES = 0.4;
    public static final double FLOWER_FLOOR_RING_HOLE_INCHES = 2.79;

    /** Far (row-6) wall, at the B/C tile seam. */
    public static Pose flowerFarWall() {
        return new Pose(2 * TILE_INCHES, FIELD_SIZE_INCHES - FLOWER_INSET_INCHES);
    }

    /** Audience (row-1) wall, at the D/E tile seam. */
    public static Pose flowerAudienceWall() {
        return new Pose(4 * TILE_INCHES, FLOWER_INSET_INCHES);
    }

    /** Blue (column-F) wall, at the row 4/5 seam. */
    public static Pose flowerBlueWall() {
        return new Pose(FIELD_SIZE_INCHES - FLOWER_INSET_INCHES, 4 * TILE_INCHES);
    }

    /** Red (column-A) wall, at the row 2/3 seam. */
    public static Pose flowerRedWall() {
        return new Pose(FLOWER_INSET_INCHES, 2 * TILE_INCHES);
    }

    /** The FLOWER on the alliance's own wall. */
    public static Pose allianceFlower(Alliance alliance) {
        return alliance == Alliance.BLUE ? flowerBlueWall() : flowerRedWall();
    }

    public static Pose[] flowers() {
        return new Pose[] {flowerFarWall(), flowerAudienceWall(), flowerBlueWall(), flowerRedWall()};
    }

    // ---- Zones (manual section 9.3, Setup Guide section 8) ----

    public static final double LOADING_ZONE_LENGTH_INCHES = 23.0;
    public static final double LOADING_ZONE_DEPTH_INCHES = 11.0;
    public static final double GARDEN_LENGTH_INCHES = 23.0;
    public static final double GARDEN_DEPTH_INCHES = 2.0;
    /** ALLIANCE AREA outside the field, against the alliance wall (not on the tiles). */
    public static final double ALLIANCE_AREA_WIDTH_INCHES = 97.0;
    public static final double ALLIANCE_AREA_DEPTH_INCHES = 54.0;

    /**
     * Blue LOADING ZONE on tile F2. INFERRED placement: against the blue wall, centred on the
     * tile's Y span. PARK is at least partially inside it.
     */
    public static Zone blueLoadingZone() {
        Pose tile = tileCenter('F', 2);
        return new Zone(FIELD_SIZE_INCHES - LOADING_ZONE_DEPTH_INCHES, tile.y() - LOADING_ZONE_LENGTH_INCHES / 2,
                FIELD_SIZE_INCHES, tile.y() + LOADING_ZONE_LENGTH_INCHES / 2);
    }

    /** Blue GARDEN on tile F6. INFERRED placement: a strip along the blue wall. */
    public static Zone blueGarden() {
        Pose tile = tileCenter('F', 6);
        return new Zone(FIELD_SIZE_INCHES - GARDEN_DEPTH_INCHES, tile.y() - GARDEN_LENGTH_INCHES / 2,
                FIELD_SIZE_INCHES, tile.y() + GARDEN_LENGTH_INCHES / 2);
    }

    public static Zone loadingZone(Alliance alliance) {
        return blueLoadingZone().forAlliance(alliance);
    }

    public static Zone garden(Alliance alliance) {
        return blueGarden().forAlliance(alliance);
    }

    // ---- Scoring elements at the start of a match (manual sections 9.8, 10.3.1, 10.5.1) ----

    public static final int POLLEN_COUNT = 40;
    public static final int NECTAR_PER_ALLIANCE = 8;
    public static final int PRELOADED_POLLEN_PER_ROBOT = 4;
    public static final int POLLEN_PER_FLOWER_AT_START = 4;
    public static final int POLLEN_PER_GARDEN_AT_START = 4;
    public static final int NECTAR_IN_UP_CELL_AT_START = 3;
    public static final int NECTAR_PER_ALLIANCE_AREA_AT_START = 5;
    /** A HIVE is calibrated to TIP on 8 POLLEN, or on 3 POLLEN plus 3 NECTAR. */
    public static final int TIP_POLLEN = 8;
    public static final int TIP_MIXED_POLLEN = 3;
    public static final int TIP_MIXED_NECTAR = 3;

    /**
     * POLLEN needed for the first TIP: the up-CELL already holds enough NECTAR for the mixed
     * threshold, so three shots do it. Every later TIP starts from an empty CELL.
     */
    public static int pollenNeededForFirstTip() {
        return NECTAR_IN_UP_CELL_AT_START >= TIP_MIXED_NECTAR ? TIP_MIXED_POLLEN : TIP_POLLEN;
    }
}
