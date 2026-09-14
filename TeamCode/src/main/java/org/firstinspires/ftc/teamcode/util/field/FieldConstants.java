package org.firstinspires.ftc.teamcode.util.field;

import com.pedropathing.math.Pose;

import org.firstinspires.ftc.teamcode.util.math.Angles;

/**
 * Field geometry and alliance symmetry, in Pedro coordinates. Game-independent.
 *
 * <h2>Coordinate system</h2>
 * Origin at a field corner, 144" square, X right, Y up, headings in radians counter-clockwise from
 * +X. Pedro 3.0.0's {@link Pose} normalises headings to {@code [0, 2pi)} itself; the helpers here
 * do the same so a mirrored heading is never negative.
 *
 * <h2>Where the season's locations live</h2>
 * Nothing here knows what is on the field. Start poses, scoring poses and parking spots are in
 * {@code game/FieldPoses}, written once for <b>BLUE</b> and converted on demand with
 * {@link #forAlliance(Pose, Alliance)}:
 *
 * <pre>
 *   Pose start = FieldConstants.forAlliance(FieldPoses.startPose(pos), alliance);
 * </pre>
 *
 * One source of truth per location means a measurement correction is a one-line change instead of
 * a hunt for the red twin someone forgot to update. The conversion math is unit-tested in
 * {@code FieldConstantsTest}.
 *
 * <h2>Which symmetry</h2>
 * FTC fields relate the two alliances in one of three ways: a mirror across the vertical centre
 * line, a mirror across the horizontal centre line, or a 180-degree rotation about the centre. A
 * mirror flips headings ({@code pi - h} or {@code -h}); a rotation adds {@code pi}. Choosing the
 * wrong one produces an autonomous that works on one alliance and drives into a wall on the
 * other, so {@link #SYMMETRY} is set from the season analysis in
 * {@code docs/04-biobuzz-season-analysis.md} and never guessed.
 */
public final class FieldConstants {
    private FieldConstants() {}

    /** Standard FTC field: 144 inches on a side. */
    public static final double FIELD_SIZE_INCHES = 144.0;
    public static final double FIELD_CENTER_INCHES = FIELD_SIZE_INCHES / 2;

    /** How a BLUE-side pose maps onto the RED side. */
    public enum Symmetry {
        /** Reflect across the vertical centre line {@code x = 72}. */
        MIRROR_X,
        /** Reflect across the horizontal centre line {@code y = 72}. */
        MIRROR_Y,
        /** Rotate 180 degrees about the field centre. */
        ROTATE_180
    }

    /**
     * The BIOBUZZ field symmetry: <b>180-degree rotation</b>. The red GARDEN is on tile A1 and the
     * blue GARDEN on F6, the LOADING ZONES are A5 and F2, the four FLOWERS sit one per wall at
     * rotated positions, and the two HIVEs start with opposite CELLS up. A mirror across the C/D
     * centre line would put the blue zones on F1 and F5, which is wrong. See
     * {@code docs/04-biobuzz-season-analysis.md}, section 2.5. A plain static so a practice field
     * laid out differently can override it from an OpMode.
     */
    public static Symmetry SYMMETRY = Symmetry.ROTATE_180;

    // ---- Alliance conversion ----

    /**
     * Converts a blue-side pose to the given alliance using {@link #SYMMETRY}.
     *
     * <p>Blue is the identity, so blue poses can be written directly as measured.
     */
    public static Pose forAlliance(Pose bluePose, Alliance alliance) {
        if (bluePose == null) return null;
        if (alliance == Alliance.BLUE) return bluePose;
        switch (SYMMETRY) {
            case MIRROR_Y:
                return mirrorAcrossY(bluePose);
            case ROTATE_180:
                return rotate180(bluePose);
            case MIRROR_X:
            default:
                return mirrorAcrossX(bluePose);
        }
    }

    /**
     * Reflects across the vertical centre line {@code x = 72}.
     *
     * <p>A direction {@code (cos h, sin h)} becomes {@code (-cos h, sin h)}, which is heading
     * {@code pi - h}. Mirroring the position without mirroring the heading is the classic way to
     * end up facing backwards on one alliance only.
     */
    public static Pose mirrorAcrossX(Pose pose) {
        if (pose == null) return null;
        return new Pose(
                FIELD_SIZE_INCHES - pose.x(),
                pose.y(),
                Angles.normalizeAngle(Math.PI - pose.heading()));
    }

    /** Reflects across the horizontal centre line {@code y = 72}; heading becomes {@code -h}. */
    public static Pose mirrorAcrossY(Pose pose) {
        if (pose == null) return null;
        return new Pose(
                pose.x(),
                FIELD_SIZE_INCHES - pose.y(),
                Angles.normalizeAngle(-pose.heading()));
    }

    /**
     * Rotates 180 degrees about the field centre: both coordinates reflect and the heading gains
     * {@code pi}. This is the conversion for a field whose red side is the blue side turned around,
     * which is what most "diagonal" FTC layouts are.
     */
    public static Pose rotate180(Pose pose) {
        if (pose == null) return null;
        return new Pose(
                FIELD_SIZE_INCHES - pose.x(),
                FIELD_SIZE_INCHES - pose.y(),
                Angles.normalizeAngle(pose.heading() + Math.PI));
    }

    // ---- Bounds ----

    /** True when a coordinate pair lies on the field. Used to reject impossible vision fixes. */
    public static boolean isInsideField(double x, double y) {
        return x >= 0 && x <= FIELD_SIZE_INCHES && y >= 0 && y <= FIELD_SIZE_INCHES;
    }

    /** True when a pose lies on the field. */
    public static boolean isInsideField(Pose pose) {
        return pose != null && isInsideField(pose.x(), pose.y());
    }
}
