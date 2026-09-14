package org.firstinspires.ftc.teamcode.util.math;

import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertFalse;
import static org.junit.Assert.assertTrue;

import org.junit.Test;

public class MedianFilterTest {
    private static final double EPS = 1e-9;

    @Test
    public void notReadyUntilWindowIsFull() {
        MedianFilter f = new MedianFilter(3);
        assertFalse(f.isReady());
        f.add(1);
        assertFalse(f.isReady());
        f.add(2);
        assertFalse(f.isReady());
        f.add(3);
        assertTrue(f.isReady());
    }

    @Test
    public void medianOfOddWindow() {
        MedianFilter f = new MedianFilter(5);
        for (double v : new double[] {5, 1, 4, 2, 3}) f.add(v);
        assertEquals(3, f.median(), EPS);
    }

    @Test
    public void medianOfEvenWindowAveragesTheMiddleTwo() {
        MedianFilter f = new MedianFilter(4);
        for (double v : new double[] {1, 2, 3, 4}) f.add(v);
        assertEquals(2.5, f.median(), EPS);
    }

    @Test
    public void medianIgnoresASingleWildOutlier() {
        // This is the whole reason for choosing a median over an average: one garbage frame must
        // not move the answer at all.
        MedianFilter f = new MedianFilter(5);
        for (double v : new double[] {10, 10.2, 9.9, 10.1, 500}) f.add(v);
        assertEquals(10.1, f.median(), EPS);
    }

    @Test
    public void averageWouldHaveBeenDraggedByThatOutlier() {
        double[] samples = {10, 10.2, 9.9, 10.1, 500};
        double sum = 0;
        for (double s : samples) sum += s;
        double mean = sum / samples.length;
        assertTrue("mean is pulled far off by the outlier", mean > 100);
    }

    @Test
    public void windowSlidesAndForgetsOldSamples() {
        MedianFilter f = new MedianFilter(3);
        f.add(100);
        f.add(100);
        f.add(100);
        assertEquals(100, f.median(), EPS);
        f.add(1);
        f.add(1);
        f.add(1);
        assertEquals(1, f.median(), EPS);
    }

    @Test
    public void spreadReportsWorstDeviation() {
        MedianFilter f = new MedianFilter(3);
        f.add(10);
        f.add(12);
        f.add(11);
        assertEquals(11, f.median(), EPS);
        assertEquals(1, f.spread(), EPS);
    }

    @Test
    public void spreadIsLargeWhenSamplesDisagree() {
        MedianFilter f = new MedianFilter(3);
        f.add(0);
        f.add(30);
        f.add(60);
        assertEquals(30, f.spread(), EPS);
    }

    @Test
    public void resetClearsTheWindow() {
        MedianFilter f = new MedianFilter(2);
        f.add(1);
        f.add(2);
        assertTrue(f.isReady());
        f.reset();
        assertFalse(f.isReady());
        assertEquals(0, f.getCount());
        assertTrue(Double.isNaN(f.median()));
    }

    @Test
    public void emptyFilterReturnsNaN() {
        assertTrue(Double.isNaN(new MedianFilter(3).median()));
        assertTrue(Double.isNaN(new MedianFilter(3).spread()));
    }

    @Test(expected = IllegalArgumentException.class)
    public void rejectsZeroWindow() {
        new MedianFilter(0);
    }
}
