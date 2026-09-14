package org.firstinspires.ftc.teamcode.util.diagnostics;

/**
 * Lets a piece of work run at most once every N milliseconds.
 *
 * <p>Not everything in the loop needs to happen at 50 Hz. Pushing a field-rendering packet to the
 * dashboard, for instance, costs far more than the information it adds at full rate — nobody can see
 * fifty frames a second, and the time comes straight out of the control loop.
 *
 * <pre>
 *   private final RateLimiter drawLimiter = new RateLimiter(100);   // 10 Hz
 *   ...
 *   if (drawLimiter.ready(now)) draw();
 * </pre>
 *
 * <p>The first call always runs, so nothing is delayed on startup waiting for an interval that has
 * not elapsed yet. Setting the interval to 0 makes it run every time, which is the honest way to
 * turn the limiter off rather than adding a second flag to check.
 */
public class RateLimiter {
    private long intervalMs;
    private long lastRunMs = 0;
    private boolean everRun = false;

    public RateLimiter(long intervalMs) {
        this.intervalMs = Math.max(0, intervalMs);
    }

    /** Adjusts the interval. Safe to call every loop for a {@code public static} tunable. */
    public void setIntervalMs(long intervalMs) {
        this.intervalMs = Math.max(0, intervalMs);
    }

    public long getIntervalMs() {
        return intervalMs;
    }

    /**
     * Whether the work should run now. Calling this <em>claims</em> the slot, so call it once per
     * loop and act on the result — two calls in one loop will not both return true.
     */
    public boolean ready(long nowMs) {
        if (!everRun) {
            everRun = true;
            lastRunMs = nowMs;
            return true;
        }
        if (nowMs - lastRunMs < intervalMs) return false;
        lastRunMs = nowMs;
        return true;
    }

    /** Forgets the last run, so the next {@link #ready} call returns true. */
    public void reset() {
        everRun = false;
        lastRunMs = 0;
    }
}
