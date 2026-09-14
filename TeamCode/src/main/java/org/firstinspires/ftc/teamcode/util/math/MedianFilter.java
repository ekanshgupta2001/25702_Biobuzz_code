package org.firstinspires.ftc.teamcode.util.math;

import java.util.Arrays;

/**
 * Rolling median over the last N samples. Pure math, no hardware — unit-tested.
 *
 * <p><b>Why a median and not a low-pass.</b> Pedro ships {@code LowPassFilter} and
 * {@code KalmanFilter}, and either is the right tool for smoothing a continuously noisy signal.
 * Vision detections do not fail that way: they are usually good and occasionally <em>completely</em>
 * wrong — a reflection, a partially occluded blob, a frame caught mid-exposure. An averaging filter
 * pulls those outliers into the output; a median throws them away entirely, which is what we want
 * before committing the robot to a path.
 *
 * <p>Use {@link #isReady()} to require a full window before acting. Acting on one frame is how a
 * single bad detection sends the robot at a wall.
 */
public class MedianFilter {
    private final double[] samples;
    private final double[] scratch;
    private int count = 0;
    private int next = 0;

    public MedianFilter(int windowSize) {
        if (windowSize < 1) throw new IllegalArgumentException("windowSize must be >= 1");
        this.samples = new double[windowSize];
        this.scratch = new double[windowSize];
    }

    public void add(double value) {
        samples[next] = value;
        next = (next + 1) % samples.length;
        if (count < samples.length) count++;
    }

    /** True once a full window has been collected. */
    public boolean isReady() {
        return count == samples.length;
    }

    public int getCount() {
        return count;
    }

    public int getWindowSize() {
        return samples.length;
    }

    /** Median of the samples collected so far, or {@link Double#NaN} if there are none. */
    public double median() {
        if (count == 0) return Double.NaN;
        System.arraycopy(samples, 0, scratch, 0, count);
        Arrays.sort(scratch, 0, count);
        int mid = count / 2;
        return count % 2 == 1 ? scratch[mid] : (scratch[mid - 1] + scratch[mid]) / 2.0;
    }

    /** Largest absolute deviation from the median — a cheap "is this window consistent?" check. */
    public double spread() {
        if (count == 0) return Double.NaN;
        double m = median();
        double worst = 0;
        for (int i = 0; i < count; i++) {
            worst = Math.max(worst, Math.abs(samples[i] - m));
        }
        return worst;
    }

    public void reset() {
        count = 0;
        next = 0;
    }
}
