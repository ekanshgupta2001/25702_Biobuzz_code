package org.firstinspires.ftc.teamcode.util.diagnostics;

import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertTrue;

import org.junit.Before;
import org.junit.Test;

public class LoopTimerTest {
    private static final double EPS = 1e-9;

    private LoopTimer timer;

    @Before
    public void setUp() {
        timer = new LoopTimer();
        LoopTimer.SPIKE_THRESHOLD_MS = 30;
    }

    @Test
    public void reportsZeroesWithNoSamples() {
        assertEquals(0, timer.getCount());
        assertEquals(0, timer.getMean(), EPS);
        assertEquals(0, timer.getMax(), EPS);
        assertEquals(0, timer.getPercentile(95), EPS);
    }

    @Test
    public void tracksLastMeanAndMax() {
        timer.record(10);
        timer.record(20);
        timer.record(30);

        assertEquals(30, timer.getLast(), EPS);
        assertEquals(20, timer.getMean(), EPS);
        assertEquals(30, timer.getMax(), EPS);
        assertEquals(3, timer.getCount());
    }

    @Test
    public void maxIsExactEvenAboveTheBucketRange() {
        // The histogram saturates, but a genuine stall must never be understated - that number is
        // the whole reason anyone looks at this.
        timer.record(15);
        timer.record(LoopTimer.MAX_TRACKED_MS + 850);
        assertEquals(LoopTimer.MAX_TRACKED_MS + 850, timer.getMax(), EPS);
    }

    @Test
    public void percentileFindsTheRightBucket() {
        // 99 loops at 10 ms, one at 100 ms.
        for (int i = 0; i < 99; i++) timer.record(10);
        timer.record(100);

        assertEquals(10, timer.getPercentile(50), EPS);
        assertEquals(10, timer.getPercentile(95), EPS);
        assertEquals(100, timer.getPercentile(100), EPS);
    }

    @Test
    public void percentileIsAccurateToOneBucket() {
        // Bucketing truncates to whole milliseconds, which is the documented trade-off.
        timer.record(12.9);
        assertEquals(12, timer.getPercentile(100), EPS);
    }

    @Test
    public void percentileClampsOutOfRangeArguments() {
        for (int i = 0; i < 10; i++) timer.record(5);
        assertEquals(5, timer.getPercentile(-20), EPS);
        assertEquals(5, timer.getPercentile(500), EPS);
    }

    @Test
    public void countsSpikesAboveTheThreshold() {
        timer.record(10);
        timer.record(29.9);
        timer.record(30);      // at the threshold counts
        timer.record(45);
        assertEquals(2, timer.getSpikeCount());
    }

    @Test
    public void ignoresNegativeAndNaNSamples() {
        // A timer read across a reset can produce nonsense; it must not poison the statistics.
        timer.record(10);
        timer.record(-5);
        timer.record(Double.NaN);

        assertEquals(1, timer.getCount());
        assertEquals(10, timer.getMean(), EPS);
        assertEquals(10, timer.getMax(), EPS);
    }

    @Test
    public void overflowSamplesLandInTheTopBucket() {
        timer.record(LoopTimer.MAX_TRACKED_MS + 1000);
        assertEquals(LoopTimer.MAX_TRACKED_MS, timer.getPercentile(100), EPS);
    }

    @Test
    public void resetClearsEverything() {
        for (int i = 0; i < 50; i++) timer.record(40);
        timer.reset();

        assertEquals(0, timer.getCount());
        assertEquals(0, timer.getMax(), EPS);
        assertEquals(0, timer.getSpikeCount());
        assertEquals(0, timer.getPercentile(95), EPS);
    }

    @Test
    public void statusMentionsTheNumbersThatMatter() {
        timer.record(18.2);
        timer.record(47.3);
        String status = timer.getStatus();
        assertTrue(status, status.contains("47.3"));   // the max
        assertTrue(status, status.contains("spike"));
    }

    @Test
    public void memoryDoesNotGrowWithSampleCount() {
        // A two-minute match at 50 Hz is ~6000 samples; a fixed histogram handles it in the same
        // space as a handful.
        for (int i = 0; i < 100_000; i++) timer.record(i % 40);
        assertEquals(100_000, timer.getCount());
        assertTrue(timer.getPercentile(95) <= 40);
    }
}
