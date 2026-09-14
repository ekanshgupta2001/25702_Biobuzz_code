package org.firstinspires.ftc.teamcode.util.time;

/**
 * {@link Clock} backed by {@link System#nanoTime()}, which is monotonic, rather than
 * {@link System#currentTimeMillis()}, which is not.
 */
public final class SystemClock implements Clock {
    static final SystemClock INSTANCE = new SystemClock();

    private final long originNanos = System.nanoTime();

    private SystemClock() {}

    @Override
    public long nowMs() {
        return (System.nanoTime() - originNanos) / 1_000_000L;
    }
}
