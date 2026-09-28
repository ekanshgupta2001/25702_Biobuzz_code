package org.firstinspires.ftc.teamcode.game;

import com.pedropathing.math.Pose;

import org.firstinspires.ftc.teamcode.util.field.FieldConstants;

/**
 * Where the robot goes on this season's field, in the frame documented on {@link Field}.
 *
 * <p>Every pose is written for <b>BLUE</b>. Ask for the red version with
 * {@link FieldConstants#forAlliance} rather than writing a second copy: one source of truth per
 * location means a measurement correction is a one-line change instead of a hunt for the rotated
 * twin someone forgot to update. The field is 180-degree symmetric, so the red twin of
 * {@code (x, y, h)} is {@code (144 - x, 144 - y, h + pi)}.
 *
 * <p>Three poses, because the robot has three places it goes on purpose: where it starts, where it
 * shoots from, and where it parks. There is one start pose because there is one autonomous — the
 * start-position enum and the selector that chose between two starts are gone, and a second start
 * belongs here again only when an autonomous actually drives from it.
 *
 * <p>Fields are non-final statics so a value can be corrected from an OpMode on a practice field.
 *
 * <h2>Every number below is a placeholder</h2>
 * None of this has been measured on a field. The wall standoffs come from the V1 CAD footprint; the
 * shooting spot and the park are first guesses built on {@link Field} geometry that is itself marked
 * INFERRED. Measure the real field and replace them. Until then, treat any path or autonomous that
 * depends on these as untested — because it is.
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
     * points toward, so the first shots need no move — which is the whole reason this is the start.
     * Red's twin is the audience wall at B1/C1, heading 270.
     */
    public static Pose BLUE_START_FACING_HIVE =
            new Pose(4 * Field.TILE_INCHES, Field.FIELD_SIZE_INCHES - ROBOT_HALF_LENGTH_INCHES,
                    Math.toRadians(90));

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
                    Field.blueLoadingZoneCenter().y(), 0);
}
