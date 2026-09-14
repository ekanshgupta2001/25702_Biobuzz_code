package org.firstinspires.ftc.teamcode.util.field;

/**
 * Where on the alliance side the robot starts autonomous.
 *
 * <p>BIOBUZZ rule G304 requires the robot to start fully on its own side (red: tile columns A-C,
 * blue: D-F), touching the perimeter wall, outside the LOADING ZONE (red A5, blue F2), outside every
 * FLOWER scoring volume, and contacting exactly 4 pre-loaded POLLEN. The manual names no start
 * tiles, so these are the two legal placements the team plans routines from. Positions are written
 * for BLUE in {@code FieldPoses.startPose(StartPosition)} and rotated for RED with
 * {@link FieldConstants#forAlliance} (the field is 180-degree symmetric), so red's "facing HIVE"
 * start on the audience wall is blue's on the far wall. See
 * {@code docs/04-biobuzz-season-analysis.md} sections 2 and 9.
 *
 * <p>Selected during init by {@code AutoSelector}, which cycles whatever values exist; adding a
 * position needs a matching pose in {@code FieldPoses} and nothing else. {@link #label()} is what
 * the driver sees on the Driver Station.
 */
public enum StartPosition {
    /**
     * Against the wall the starting up-facing CELL points toward (red: audience wall, tiles B1/C1;
     * blue: far wall, tiles D6/E6). Shortest path to the first three shots.
     */
    FACING_HIVE("Facing HIVE"),
    /**
     * Against the alliance wall between its FLOWER and its LOADING ZONE (red: tiles A3/A4; blue:
     * tiles F3/F4). Clear of the GARDEN corner and the FLOWER scoring volume.
     */
    ALLIANCE_WALL("Alliance wall");

    private final String label;

    StartPosition(String label) {
        this.label = label;
    }

    /** Driver-facing name for the init menu. */
    public String label() {
        return label;
    }
}
