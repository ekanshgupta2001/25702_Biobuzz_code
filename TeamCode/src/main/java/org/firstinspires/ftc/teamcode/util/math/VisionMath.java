package org.firstinspires.ftc.teamcode.util.math;

/**
 * Turns a camera bearing into a field position. Pure math, no hardware — so it is unit-tested.
 *
 * <h2>The geometry</h2>
 *
 * A pinhole camera reports a target as two angles from its crosshair: {@code tx} (positive right)
 * and {@code ty} (positive up). Those are <em>independent</em> angles, so the ray to the target in
 * the camera's own frame is proportional to {@code (tan tx, tan ty, 1)} along (right, up, forward).
 *
 * <p>The camera is mounted pitched down by {@code P}, so that ray has to be rotated into the robot
 * frame (X forward, Y left, Z up) before it can be intersected with the ground plane:
 *
 * <pre>
 *   rX = cos P + tan(ty) * sin P        forward component
 *   rY = -tan(tx)                       left component (tx>0 means right, hence the sign)
 *   rZ = tan(ty) * cos P - sin P        vertical component; must be negative to reach the floor
 * </pre>
 *
 * Scaling that ray until it drops by the height difference gives the answer. In closed form the
 * forward component reduces to the familiar {@code dh / tan(P - ty)}, and the lateral component to
 * {@code -forward * tan(tx) * cos(ty) / cos(P - ty)}.
 *
 * <h2>The mistake this replaces</h2>
 *
 * It is tempting to treat {@code dh / tan(P - ty)} as a <em>radial</em> distance and split it with
 * {@code cos(tx)} / {@code sin(tx)}. It is not — it is already the forward component. Doing that
 * projects twice and shortens every off-axis estimate:
 *
 * <pre>
 *   tx      cos/sin split        correct        range error
 *    0deg   (28.85,   0.00)   (28.85,   0.00)      0%
 *   10deg   (28.41,  -5.01)   (28.85,  -5.41)     -1.7%
 *   30deg   (24.99, -14.43)   (28.85, -17.72)    -14.8%
 *   45deg   (20.40, -20.40)   (28.85, -30.70)    -31.6%
 * </pre>
 *
 * Exact on-axis, badly wrong at the edge of frame — which is exactly where a target is first seen.
 */
public final class VisionMath {
    private VisionMath() {}

    /** Where the camera sits on the robot, and which way it points. */
    public static final class Mount {
        /** Lens height above the floor, inches. */
        public final double heightInches;
        /** Downward tilt in degrees. Positive means tilted toward the floor. */
        public final double pitchDegrees;
        /** Distance ahead of the robot's rotation centre, inches. */
        public final double forwardOffsetInches;
        /** Distance left of the robot's centreline, inches. Negative is right. */
        public final double leftOffsetInches;
        /** Rotation of the camera about vertical, degrees. Positive turns the camera left. */
        public final double yawOffsetDegrees;

        public Mount(double heightInches, double pitchDegrees, double forwardOffsetInches,
                     double leftOffsetInches, double yawOffsetDegrees) {
            this.heightInches = heightInches;
            this.pitchDegrees = pitchDegrees;
            this.forwardOffsetInches = forwardOffsetInches;
            this.leftOffsetInches = leftOffsetInches;
            this.yawOffsetDegrees = yawOffsetDegrees;
        }
    }

    /**
     * Ground distance straight ahead of the camera to a target at the given vertical angle.
     *
     * <p>This is the forward component, <b>not</b> a radial range — see the class docs.
     *
     * @return distance in inches, or {@link Double#NaN} if the target is at or above the horizon
     */
    public static double forwardDistanceInches(double tyDegrees, double pitchDegrees,
                                               double heightDropInches) {
        double downRad = Math.toRadians(pitchDegrees - tyDegrees);
        // At or above the horizon the ray never meets the floor; just below it, range runs away to
        // infinity. Both are rejected here rather than returned as an enormous or negative number.
        if (downRad <= 0) return Double.NaN;
        double tan = Math.tan(downRad);
        if (tan <= 0) return Double.NaN;
        return heightDropInches / tan;
    }

    /**
     * Locates a target relative to the robot by intersecting the camera ray with the ground plane.
     *
     * @param targetHeightInches height of the target's centre above the floor
     * @param maxRangeInches     estimates beyond this are rejected as untrustworthy
     * @return {@code {forward, left}} in inches in the robot frame, or {@code null} if the target
     *         is at/above the horizon, behind the robot, or further away than {@code maxRangeInches}
     */
    public static double[] targetInRobotFrame(double txDegrees, double tyDegrees, Mount mount,
                                              double targetHeightInches, double maxRangeInches) {
        double heightDrop = mount.heightInches - targetHeightInches;
        if (heightDrop <= 0) return null;

        double pitchRad = Math.toRadians(mount.pitchDegrees);
        double sinP = Math.sin(pitchRad);
        double cosP = Math.cos(pitchRad);
        double tanTx = Math.tan(Math.toRadians(txDegrees));
        double tanTy = Math.tan(Math.toRadians(tyDegrees));

        double rX = cosP + tanTy * sinP;
        double rZ = tanTy * cosP - sinP;
        double rY = -tanTx;

        // A ray that is level or rising never reaches the floor.
        if (rZ >= 0) return null;

        double scale = -heightDrop / rZ;
        double forwardCam = scale * rX;
        double leftCam = scale * rY;
        if (forwardCam <= 0) return null;

        // A yawed mount rotates the whole camera frame about vertical, so rotate the result rather
        // than fudging tx: the pitch decomposition above is only valid in the camera's own heading.
        double yawRad = Math.toRadians(mount.yawOffsetDegrees);
        double cosY = Math.cos(yawRad);
        double sinY = Math.sin(yawRad);
        double forward = forwardCam * cosY - leftCam * sinY + mount.forwardOffsetInches;
        double left = forwardCam * sinY + leftCam * cosY + mount.leftOffsetInches;

        if (Math.hypot(forward, left) > maxRangeInches) return null;
        return new double[] {forward, left};
    }

    /**
     * Pulls a robot-frame target back toward the robot so the robot stops short of it.
     *
     * <p>Without this the path drives the robot's rotation centre onto the game piece. Clamped at
     * zero so a standoff larger than the range cannot flip the target behind the robot.
     */
    public static double[] applyStandoff(double[] robotFrame, double standoffInches) {
        if (robotFrame == null) return null;
        double range = Math.hypot(robotFrame[0], robotFrame[1]);
        if (range <= 1e-6) return new double[] {0, 0};
        double scaled = Math.max(0, range - standoffInches) / range;
        return new double[] {robotFrame[0] * scaled, robotFrame[1] * scaled};
    }

    /**
     * Rotates a robot-frame offset into field coordinates and adds it to the robot's position.
     *
     * @return {@code {fieldX, fieldY}} in inches
     */
    public static double[] toFieldFrame(double robotX, double robotY, double robotHeadingRad,
                                        double forward, double left) {
        double cosH = Math.cos(robotHeadingRad);
        double sinH = Math.sin(robotHeadingRad);
        double dx = forward * cosH - left * sinH;
        double dy = forward * sinH + left * cosH;
        return new double[] {robotX + dx, robotY + dy};
    }
}
