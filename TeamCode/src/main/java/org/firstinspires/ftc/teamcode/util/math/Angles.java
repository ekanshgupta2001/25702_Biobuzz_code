package org.firstinspires.ftc.teamcode.util.math;

/**
 * Angle arithmetic that respects the wrap at 0/2pi.
 *
 * <h2>Why this is its own class</h2>
 * Field geometry, heading hold and macro aiming all need the same two operations, and none of them
 * has anything to do with a camera, a drivetrain or a field. Kept apart, there is one place where
 * the wrap is handled and one place to look when a heading comes out backwards.
 *
 * <h2>The wrap is the whole point</h2>
 * Two conventions meet in this codebase and disagree:
 *
 * <ul>
 *   <li>{@code Math.atan2} returns {@code (-pi, pi]}</li>
 *   <li>Every {@code Pose} Pedro hands back is in {@code [0, 2pi)}</li>
 * </ul>
 *
 * Mix them and a naive comparison breaks. Worse, a controller fed a raw angle difference across the
 * seam sees 359 degrees and 1 degree as 358 degrees apart and drives the long way round at full
 * power. {@link #angleError} always takes the short way; {@link #normalizeAngle} puts an angle in
 * Pedro's convention.
 */
public final class Angles {
    private Angles() {}

    /** Wraps an angle into {@code [0, 2pi)}, matching Pedro's convention. */
    public static double normalizeAngle(double radians) {
        double a = radians % (2 * Math.PI);
        if (a < 0) a += 2 * Math.PI;
        return a;
    }

    /**
     * Shortest signed turn from {@code from} to {@code to}, in {@code (-pi, pi]}.
     *
     * <p>Use this for "how far do I still need to rotate?". Plain subtraction answers that 350
     * degrees to 10 degrees is a 340-degree turn; this answers 20.
     */
    public static double angleError(double fromRad, double toRad) {
        double diff = normalizeAngle(toRad) - normalizeAngle(fromRad);
        if (diff > Math.PI) diff -= 2 * Math.PI;
        if (diff <= -Math.PI) diff += 2 * Math.PI;
        return diff;
    }
}
