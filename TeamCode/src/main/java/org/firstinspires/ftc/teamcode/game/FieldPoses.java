package org.firstinspires.ftc.teamcode.game;

import com.pedropathing.math.Pose;

import org.firstinspires.ftc.teamcode.util.field.FieldConstants;
import org.firstinspires.ftc.teamcode.util.field.StartPosition;

/**
 * Where the robot goes on this season's field, in the frame documented on {@link Field}.
 *
 * <p>Every pose is written for <b>BLUE</b>. Ask for the red version with
 * {@link FieldConstants#forAlliance} rather than writing a second copy: one source of truth per
 * location means a measurement correction is a one-line change instead of a hunt for the rotated
 * twin someone forgot to update. The field is 180-degree symmetric, so the red twin of
 * {@code (x, y, h)} is {@code (144 - x, 144 - y, h + pi)}; {@code FieldPosesTest} checks that the
 * rotated starts land on the tiles the manual names.
 *
 * <p>Fields are non-final statics so a value can be corrected from an OpMode on a practice field.
 *
 * <h2>These numbers are placeholders</h2>
 * Wall standoffs use the V1 CAD footprint; the shooting spot, park and garden approach are
 * reasonable first guesses. Measure the real field and replace them before trusting any path.
 */
public final class FieldPoses {
    private FieldPoses() {}

    /** Half the V1 footprint (17.5 in square), for poses that put a wall at the robot's back. */
    public static double ROBOT_HALF_LENGTH_INCHES = 8.75;
    /** Gap left between the bumper and a wall for poses that only need to be near it. */
    public static double WALL_CLEARANCE_INCHES = 2.0;

    /**
     * Start against the far wall at the D6/E6 seam with the shooter (the rear) toward the HIVE:
     * heading 90, front to the wall, rear toward -Y. This is the wall blue's starting up-CELL
     * points toward, so the first three shots need no move. Red's twin is the audience wall at
     * B1/C1, heading 270.
     */
    public static Pose BLUE_START_FACING_HIVE =
            new Pose(4 * Field.TILE_INCHES, Field.FIELD_SIZE_INCHES - ROBOT_HALF_LENGTH_INCHES,
                    Math.toRadians(90));

    /**
     * Start against the blue alliance wall at the F3/F4 seam with the shooter toward the HIVE:
     * heading 0, front to the wall, rear toward -X. Clear of the GARDEN corner and the FLOWER
     * volume. Red's twin is A3/A4, heading 180.
     */
    public static Pose BLUE_START_ALLIANCE_WALL =
            new Pose(Field.FIELD_SIZE_INCHES - ROBOT_HALF_LENGTH_INCHES, 3 * Field.TILE_INCHES,
                    Math.toRadians(0));

    /**
     * A shooting position in front of the blue far-side CELL (the one that starts up), with the
     * shooter toward it (heading 90: the rear faces -Y). There is no launch zone and no velocity
     * cap, so this is a range and line-of-sight choice.
     */
    public static Pose BLUE_SHOOTING_SPOT =
            new Pose(Field.HIVE_CENTER.x() + Field.HIVE_LATERAL_OFFSET_INCHES, 5 * Field.TILE_INCHES,
                    Math.toRadians(90));

    /**
     * PARK: robot centre just inside the blue LOADING ZONE on F2, front toward the wall so a
     * NECTAR can be loaded straight into the funnel. Only partial overlap is required.
     */
    public static Pose BLUE_PARK =
            new Pose(Field.FIELD_SIZE_INCHES - ROBOT_HALF_LENGTH_INCHES - WALL_CLEARANCE_INCHES,
                    Field.blueLoadingZone().center().y(), 0);

    /** Intake facing the blue GARDEN strip on F6, to collect its four POLLEN. */
    public static Pose BLUE_GARDEN_APPROACH =
            new Pose(Field.FIELD_SIZE_INCHES - Field.GARDEN_DEPTH_INCHES - ROBOT_HALF_LENGTH_INCHES
                    - WALL_CLEARANCE_INCHES, Field.blueGarden().center().y(), 0);

    /** Blue-side start pose for a start position. Adding a position needs a case here and nothing else. */
    public static Pose startPose(StartPosition position) {
        switch (position) {
            case FACING_HIVE:
                return BLUE_START_FACING_HIVE;
            case ALLIANCE_WALL:
                return BLUE_START_ALLIANCE_WALL;
            default:
                throw new IllegalArgumentException("no pose for " + position);
        }
    }

    /** Every blue pose, for sanity checks. */
    public static Pose[] all() {
        return new Pose[] {
                BLUE_START_FACING_HIVE, BLUE_START_ALLIANCE_WALL, BLUE_SHOOTING_SPOT, BLUE_PARK,
                BLUE_GARDEN_APPROACH,
        };
    }
}
