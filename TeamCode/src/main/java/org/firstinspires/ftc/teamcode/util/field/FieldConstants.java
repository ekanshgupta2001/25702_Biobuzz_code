package org.firstinspires.ftc.teamcode.util.field;

import com.pedropathing.math.Pose;

import org.firstinspires.ftc.teamcode.util.math.Angles;

/**
 * Field geometry and alliance symmetry, in Pedro coordinates. Game-independent.
 *
 * <h2>Coordinate system</h2>
 * Origin at a field corner, 144" square, X right, Y up, headings in radians counter-clockwise from
 * +X. Pedro 3.0.0's {@link Pose} normalises headings to {@code [0, 2pi)} itself; the transform here
 * does the same, so a converted heading is never handed back negative.
 *
 * <h2>Where the season's locations live</h2>
 * Nothing here knows what is on the field. Start poses, scoring poses and parking spots are in
 * {@code game/FieldPoses}, written once for <b>BLUE</b> and converted on demand with
 * {@link #forAlliance(Pose, Alliance)}:
 *
 * <pre>
 *   Pose start = FieldConstants.forAlliance(FieldPoses.BLUE_START_FACING_HIVE, alliance);
 * </pre>
 *
 * One source of truth per location means a measurement correction is a one-line change instead of
 * a hunt for the red twin someone forgot to update.
 *
 * <h2>Rotation, not mirroring — and no switch to get it wrong with</h2>
 * An FTC field relates its two alliances either by a mirror across a centre line or by a 180-degree
 * rotation about the centre, and the difference shows up in the heading: a mirror flips it
 * ({@code pi - h} or {@code -h}), a rotation adds {@code pi}. Choose wrong and you get an
 * autonomous that scores on one alliance and drives into a wall on the other.
 *
 * <p>The BIOBUZZ field is <b>180-degree rotationally symmetric</b>. The evidence is the diagonal
 * pairing of the alliance-specific zones: the red GARDEN is on tile A1 and the blue GARDEN on F6,
 * and the LOADING ZONES are A5 and F2. A mirror across the C/D centre line would put the blue
 * GARDEN on F1 and the blue LOADING ZONE on F5, which is not where the field puts them. The two
 * HIVEs likewise start with opposite CELLs facing up. See
 * {@code docs/04-biobuzz-season-analysis.md}, section 2.5.
 *
 * <p>So {@link #rotate180} is the only transform in this file and {@link #forAlliance} calls it
 * directly. There used to be a {@code Symmetry} enum with three arms and a {@code SYMMETRY} field
 * selecting between them; it was permanently {@code ROTATE_180}. A run-time switch over a fact the
 * field cannot change is only a place for a wrong value to hide — and an override that silently
 * mirrors every blue auto is a bad trade for a practice-field convenience nobody used.
 */
public final class FieldConstants {
    private FieldConstants() {}

    /** Standard FTC field: 144 inches on a side. */
    public static final double FIELD_SIZE_INCHES = 144.0;

    // ---- Alliance conversion ----

    /**
     * Converts a blue-side pose to the given alliance.
     *
     * <p>Blue is the identity, so blue poses can be written directly as measured.
     */
    public static Pose forAlliance(Pose bluePose, Alliance alliance) {
        if (bluePose == null) return null;
        if (alliance == Alliance.BLUE) return bluePose;
        return rotate180(bluePose);
    }

    /**
     * Rotates 180 degrees about the field centre: both coordinates reflect and the heading gains
     * {@code pi}. Reflecting the position while leaving the heading alone is the classic way to end
     * up facing backwards on one alliance only.
     */
    public static Pose rotate180(Pose pose) {
        if (pose == null) return null;
        return new Pose(
                FIELD_SIZE_INCHES - pose.x(),
                FIELD_SIZE_INCHES - pose.y(),
                Angles.normalizeAngle(pose.heading() + Math.PI));
    }
}
