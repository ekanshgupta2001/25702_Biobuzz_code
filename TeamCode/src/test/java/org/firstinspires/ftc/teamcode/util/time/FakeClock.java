package org.firstinspires.ftc.teamcode.util.time;

/** A {@link Clock} that only moves when a test says so. */
public final class FakeClock implements Clock {
    private long nowMs;

    public FakeClock() {
        this(0);
    }

    public FakeClock(long startMs) {
        this.nowMs = startMs;
    }

    @Override
    public long nowMs() {
        return nowMs;
    }

    public void advance(long ms) {
        if (ms < 0) throw new IllegalArgumentException("clock cannot go backwards");
        nowMs += ms;
    }

    public void set(long ms) {
        if (ms < nowMs) throw new IllegalArgumentException("clock cannot go backwards");
        nowMs = ms;
    }
}
