package org.firstinspires.ftc.teamcode.util.diagnostics;

import static org.junit.Assert.assertFalse;
import static org.junit.Assert.assertTrue;

import org.junit.Test;

public class RateLimiterTest {

    @Test
    public void firstCallAlwaysRuns() {
        // Nothing should be delayed at startup waiting for an interval that has not elapsed yet.
        assertTrue(new RateLimiter(100).ready(0));
        assertTrue(new RateLimiter(100).ready(5_000_000));
    }

    @Test
    public void suppressesUntilTheIntervalElapses() {
        RateLimiter limiter = new RateLimiter(100);
        assertTrue(limiter.ready(1000));

        assertFalse(limiter.ready(1050));
        assertFalse(limiter.ready(1099));
        assertTrue(limiter.ready(1100));
    }

    @Test
    public void measuresFromTheLastRunNotTheLastCall() {
        RateLimiter limiter = new RateLimiter(100);
        limiter.ready(0);

        // Being asked repeatedly in between must not push the next slot back.
        for (long t = 10; t < 100; t += 10) assertFalse(limiter.ready(t));
        assertTrue(limiter.ready(100));
    }

    @Test
    public void claimsTheSlotSoTwoCallsInOneLoopDoNotBothRun() {
        RateLimiter limiter = new RateLimiter(100);
        assertTrue(limiter.ready(500));
        assertFalse("calling ready() twice at the same instant must not run the work twice",
                limiter.ready(500));
    }

    @Test
    public void zeroIntervalRunsEveryTime() {
        RateLimiter limiter = new RateLimiter(0);
        for (long t = 0; t < 10; t++) assertTrue(limiter.ready(t));
    }

    @Test
    public void negativeIntervalIsTreatedAsZero() {
        RateLimiter limiter = new RateLimiter(-50);
        assertTrue(limiter.ready(0));
        assertTrue(limiter.ready(0));
    }

    @Test
    public void intervalCanBeChangedLive() {
        RateLimiter limiter = new RateLimiter(1000);
        assertTrue(limiter.ready(0));
        assertFalse(limiter.ready(200));

        limiter.setIntervalMs(100);
        assertTrue(limiter.ready(200));
    }

    @Test
    public void resetMakesTheNextCallRun() {
        RateLimiter limiter = new RateLimiter(1000);
        assertTrue(limiter.ready(0));
        assertFalse(limiter.ready(10));

        limiter.reset();
        assertTrue(limiter.ready(10));
    }

    @Test
    public void producesRoughlyTheExpectedRate() {
        // 100 ms interval sampled at 50 Hz should fire about ten times per second.
        RateLimiter limiter = new RateLimiter(100);
        int runs = 0;
        for (long t = 0; t < 1000; t += 20) {
            if (limiter.ready(t)) runs++;
        }
        assertTrue("expected ~10 runs, got " + runs, runs >= 9 && runs <= 11);
    }
}
