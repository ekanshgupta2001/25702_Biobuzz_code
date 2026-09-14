package org.firstinspires.ftc.teamcode.util.math;

/**
 * Angle arithmetic that respects the wrap at 0/2pi.
 *
 * <h2>Why this is its own class</h2>
 * Field geometry, heading hold, and macro targeting all need this arithmetic, and none of it has
 * anything to do with a camera. Keeping it out of {@link VisionMath} keeps that class's name
 * honest and shows where general math belongs.
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
 * Pedro's convention. Pure arithmetic, no hardware, fully unit tested.
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

    /**
     * Heading that points from the robot toward a robot-frame offset, normalised to {@code [0, 2pi)}.
     *
     * @param robotHeadingRad the robot's current field heading
     * @param forward         offset ahead of the robot, in any consistent unit
     * @param left            offset to the robot's left, same unit
     */
    public static double headingToward(double robotHeadingRad, double forward, double left) {
        return normalizeAngle(robotHeadingRad + Math.atan2(left, forward));
    }
}
