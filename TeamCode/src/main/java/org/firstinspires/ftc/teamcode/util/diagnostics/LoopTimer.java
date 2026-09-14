package org.firstinspires.ftc.teamcode.util.diagnostics;

import java.util.Locale;

/**
 * Loop-time statistics, including percentiles, in fixed memory.
 *
 * <h2>Why an instantaneous reading is not enough</h2>
 * Showing only the current loop time tells you nothing: a spike lasts one cycle and the number is
 * gone before anyone can read it. What you actually want to know after a match is "did the loop
 * ever stall, how badly, and how often" — and that needs history.
 *
 * <h2>Why a histogram and not a buffer of samples</h2>
 * Keeping every sample would mean an unbounded array over a two-minute match, and keeping the last
 * N would only describe the last few seconds. Counting samples into fixed millisecond buckets gives
 * any percentile over the <em>whole</em> run in constant memory and constant time per sample, with
 * no allocation in the loop.
 *
 * <p>The cost is resolution: a percentile is accurate to one bucket. That is the right trade here —
 * whether p95 is 21 ms or 21.4 ms changes nothing, whereas whether it is 21 ms or 210 ms changes
 * everything.
 *
 * <p>Anything at or above {@link #MAX_TRACKED_MS} lands in the overflow bucket and is reported as
 * that value. {@link #getMax()} is exact regardless, so a genuine stall is never understated.
 */
public class LoopTimer {
    /** Bucket count. Loop times above this are counted in the overflow bucket. */
    public static final int MAX_TRACKED_MS = 200;
    /** A loop slower than this counts as a spike. ~50 Hz is a 20 ms budget. */
    public static double SPIKE_THRESHOLD_MS = 30;

    private final int[] buckets = new int[MAX_TRACKED_MS + 1];
    private long count = 0;
    private double sum = 0;
    private double last = 0;
    private double max = 0;
    private long spikes = 0;

    /** Records one loop duration. Allocation-free. */
    public void record(double loopMs) {
        if (Double.isNaN(loopMs) || loopMs < 0) return;

        last = loopMs;
        count++;
        sum += loopMs;
        if (loopMs > max) max = loopMs;
        if (loopMs >= SPIKE_THRESHOLD_MS) spikes++;

        int bucket = (int) loopMs;
        if (bucket > MAX_TRACKED_MS) bucket = MAX_TRACKED_MS;
        buckets[bucket]++;
    }

    public double getLast() {
        return last;
    }

    public double getMax() {
        return max;
    }

    public long getCount() {
        return count;
    }

    /** Loops at or above {@link #SPIKE_THRESHOLD_MS}. */
    public long getSpikeCount() {
        return spikes;
    }

    public double getMean() {
        return count == 0 ? 0 : sum / count;
    }

    /**
     * Approximate percentile of recorded loop times, in milliseconds.
     *
     * @param percentile 0-100; 95 gives the value 95% of loops came in under
     * @return the lower edge of the bucket containing that percentile, or 0 with no samples
     */
    public double getPercentile(double percentile) {
        if (count == 0) return 0;
        double clamped = Math.max(0, Math.min(100, percentile));

        // Ceiling, so p100 asks for the last sample rather than one short of it.
        long target = (long) Math.ceil(clamped / 100.0 * count);
        if (target < 1) target = 1;

        long seen = 0;
        for (int i = 0; i <= MAX_TRACKED_MS; i++) {
            seen += buckets[i];
            if (seen >= target) return i;
        }
        return MAX_TRACKED_MS;
    }

    public void reset() {
        java.util.Arrays.fill(buckets, 0);
        count = 0;
        sum = 0;
        last = 0;
        max = 0;
        spikes = 0;
    }

    /** One-line summary for telemetry, e.g. {@code "18.2 now / 19 p95 / 47.3 max / 3 spikes"}. */
    public String getStatus() {
        return String.format(Locale.US, "%.1f now / %.0f p95 / %.1f max / %d spikes",
                last, getPercentile(95), max, spikes);
    }
}
