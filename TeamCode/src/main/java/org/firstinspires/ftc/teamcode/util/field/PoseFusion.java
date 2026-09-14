package org.firstinspires.ftc.teamcode.util.field;

import com.pedropathing.controllers.filters.KalmanFilter;
import com.pedropathing.math.Pose;

/**
 * Blends wheel/Pinpoint odometry with AprilTag position fixes.
 *
 * <h2>Why not just call setPose()</h2>
 * Writing a vision pose straight into the follower teleports the robot's belief. One bad frame and
 * the next path drives somewhere arbitrary. Odometry is smooth but drifts; vision is absolute but
 * noisy and late. Fusing keeps the good half of each.
 *
 * <h2>BIOBUZZ</h2>
 * Every AprilTag this season is on a moving HIVE CELL, and the SDK v12.0 release notes state that
 * BIOBUZZ tags are not suitable for absolute field localisation. This class accepts any absolute
 * (x, y) fix, so it stays for a future static reference (wall or FLOWER geometry from a distance
 * sensor, for example), but during BIOBUZZ expect it to run odometry-only. Pass {@code null} for the
 * vision pose and it costs one filter step per axis.
 *
 * <h2>Position is fused; heading is not</h2>
 * Only X and Y go through the filter. Heading is taken from odometry unchanged, for two reasons:
 * a Pinpoint's fused IMU heading is far better than a tag-derived one, and angles wrap at 0/2pi,
 * which a scalar Kalman filter handles badly — a filter straddling the seam would average 359 and 1
 * into 180 and point the robot backwards. Avoiding the problem beats patching it.
 *
 * <h2>Latency</h2>
 * A camera fix describes where the robot <em>was</em> when the shutter opened, typically 60-120 ms
 * ago. At 40 in/s that is several inches. This class keeps a short history of odometry samples,
 * looks up where the robot was at capture time, and applies the vision-minus-history offset to the
 * <em>current</em> odometry — so the correction lands in the present.
 *
 * <p>Pure math apart from Pedro's {@link KalmanFilter} and {@link Pose}, both of which are plain
 * Java, so all of this is unit-testable off-robot.
 */
public class PoseFusion {
    /** Reject a fix further than this from the current estimate. Almost certainly a bad frame. */
    public static double MAX_JUMP_INCHES = 24.0;
    /** How much the odometry increment is trusted. Larger = quicker to accept vision. */
    public static double MODEL_COVARIANCE = 0.1;
    /** How noisy vision is assumed to be. Larger = vision moves the estimate less. */
    public static double DATA_COVARIANCE = 0.6;
    /** Odometry samples retained for latency lookup. 64 at ~50 Hz is a bit over a second. */
    public static final int HISTORY_SIZE = 64;

    public enum Result { SEEDED, ODOMETRY_ONLY, ACCEPTED, REJECTED_JUMP, REJECTED_OFF_FIELD }

    private final KalmanFilter filterX;
    private final KalmanFilter filterY;

    private final long[] histTime = new long[HISTORY_SIZE];
    private final double[] histX = new double[HISTORY_SIZE];
    private final double[] histY = new double[HISTORY_SIZE];
    private int histCount = 0;
    private int histNext = 0;

    private Pose lastOdometry = null;
    private boolean seeded = false;

    private int accepted = 0;
    private int rejected = 0;
    private Result lastResult = Result.ODOMETRY_ONLY;

    public PoseFusion() {
        filterX = new KalmanFilter(MODEL_COVARIANCE, DATA_COVARIANCE);
        filterY = new KalmanFilter(MODEL_COVARIANCE, DATA_COVARIANCE);
    }

    /** Establishes the starting estimate. Call once when the pose is first known. */
    public void seed(Pose pose) {
        if (pose == null) return;
        filterX.reset(pose.x(), 1, 1);
        filterY.reset(pose.y(), 1, 1);
        lastOdometry = pose;
        seeded = true;
        histCount = 0;
        histNext = 0;
        lastResult = Result.SEEDED;
    }

    public boolean isSeeded() {
        return seeded;
    }

    /**
     * Advances the estimate by one loop.
     *
     * <p><b>Contract: the caller must write the returned pose back to the follower.</b> The
     * per-loop delta is measured against the pose this class last handed out, which is only the
     * true odometry increment if that pose became the follower's new baseline. Ignore the return
     * value and the next correction gets counted a second time as if the robot had moved.
     *
     * @param nowMs           current time in milliseconds
     * @param odometry        the follower's own pose estimate this loop, never null
     * @param vision          an AprilTag pose, or {@code null} if none is available or trustworthy
     * @param visionLatencyMs how old the vision fix is (capture + targeting latency)
     * @return the fused pose, or the odometry pose unchanged if this is the first call
     */
    public Pose update(long nowMs, Pose odometry, Pose vision, long visionLatencyMs) {
        if (odometry == null) return null;
        if (!seeded) {
            seed(odometry);
            return odometry;
        }

        double dx = odometry.x() - lastOdometry.x();
        double dy = odometry.y() - lastOdometry.y();

        recordHistory(nowMs, odometry.x(), odometry.y());

        // Predicted position if we only trusted odometry this loop.
        double predictedX = filterX.state() + dx;
        double predictedY = filterY.state() + dy;

        if (vision == null) {
            // No measurement: feed the prediction back as the "measurement" so the correction term
            // is zero and the filter simply integrates the odometry increment.
            filterX.update(dx, predictedX);
            filterY.update(dy, predictedY);
            lastResult = Result.ODOMETRY_ONLY;
            return fused(odometry);
        }

        double[] measurement = latencyCompensate(nowMs, visionLatencyMs, odometry, vision);
        double measX = measurement[0];
        double measY = measurement[1];

        if (!FieldConstants.isInsideField(measX, measY)) {
            filterX.update(dx, predictedX);
            filterY.update(dy, predictedY);
            rejected++;
            lastResult = Result.REJECTED_OFF_FIELD;
            return fused(odometry);
        }

        if (Math.hypot(measX - predictedX, measY - predictedY) > MAX_JUMP_INCHES) {
            filterX.update(dx, predictedX);
            filterY.update(dy, predictedY);
            rejected++;
            lastResult = Result.REJECTED_JUMP;
            return fused(odometry);
        }

        filterX.update(dx, measX);
        filterY.update(dy, measY);
        accepted++;
        lastResult = Result.ACCEPTED;
        return fused(odometry);
    }

    /**
     * Moves a vision fix forward in time to account for camera latency.
     *
     * <p>The fix describes where the robot was at capture. Subtracting the odometry position from
     * that same instant yields a pure correction offset, which is then applied to where odometry
     * says we are now. If there is no history that far back, the fix is used as-is.
     */
    private double[] latencyCompensate(long nowMs, long latencyMs, Pose odometry, Pose vision) {
        double[] then = historicalPosition(nowMs - Math.max(0, latencyMs));
        if (then == null) {
            return new double[] {vision.x(), vision.y()};
        }
        double offsetX = vision.x() - then[0];
        double offsetY = vision.y() - then[1];
        return new double[] {odometry.x() + offsetX, odometry.y() + offsetY};
    }

    /**
     * Builds the fused pose and records it as the baseline for next loop's delta.
     *
     * <p>Recording the <em>fused</em> pose rather than the raw odometry is what keeps a correction
     * from being re-counted as motion, given the caller writes this pose back to the follower.
     */
    private Pose fused(Pose odometry) {
        // Heading passes through untouched - see the class docs.
        Pose result = new Pose(filterX.state(), filterY.state(), odometry.heading());
        lastOdometry = result;
        return result;
    }

    private void recordHistory(long timeMs, double x, double y) {
        histTime[histNext] = timeMs;
        histX[histNext] = x;
        histY[histNext] = y;
        histNext = (histNext + 1) % HISTORY_SIZE;
        if (histCount < HISTORY_SIZE) histCount++;
    }

    /** Odometry position at the recorded sample closest to {@code timeMs}, or null if no history. */
    public double[] historicalPosition(long timeMs) {
        if (histCount == 0) return null;
        int best = -1;
        long bestDelta = Long.MAX_VALUE;
        for (int i = 0; i < histCount; i++) {
            long delta = Math.abs(histTime[i] - timeMs);
            if (delta < bestDelta) {
                bestDelta = delta;
                best = i;
            }
        }
        return new double[] {histX[best], histY[best]};
    }

    public Result getLastResult() {
        return lastResult;
    }

    public int getAcceptedCount() {
        return accepted;
    }

    public int getRejectedCount() {
        return rejected;
    }

    /** Short status line for telemetry. */
    public String getStatus() {
        return lastResult + " (ok=" + accepted + " rej=" + rejected + ")";
    }
}
