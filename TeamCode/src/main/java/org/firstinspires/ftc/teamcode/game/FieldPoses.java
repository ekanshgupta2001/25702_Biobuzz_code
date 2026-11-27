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
 * <p>One pose: where the robot starts. Teleop uses it as the starting pose once the Pinpoint is set
 * up.
 *
 * <p>Fields are non-final statics so a value can be corrected from an OpMode on a practice field.
 *
 * <h2>Every number below is a placeholder</h2>
 * None of this has been measured on a field. Measure the real field and replace it.
 */
public final class FieldPoses {
    private FieldPoses() {}

    /** Half the V1 footprint (17.5 in square), for poses that put a wall at the robot's back. */
    public static double ROBOT_HALF_LENGTH_INCHES = 8.75;
    /**
     * Start against the far wall at the D6/E6 seam with the shooter (the front) toward the HIVE:
     * heading 270, rear to the wall, front toward -Y. This is the wall blue's starting up-CELL
     * points toward, so the first shots need no move — which is the whole reason this is the start.
     * Red's twin is the audience wall at B1/C1, heading 90.
     */
    public static Pose BLUE_START_FACING_HIVE =
            new Pose(4 * Field.TILE_INCHES, Field.FIELD_SIZE_INCHES - ROBOT_HALF_LENGTH_INCHES,
                    Math.toRadians(270));
}
