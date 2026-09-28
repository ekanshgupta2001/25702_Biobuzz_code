package org.firstinspires.ftc.teamcode.util.time;

/**
 * Tracks how much of the match period is left.
 *
 * <h2>BIOBUZZ</h2>
 * The numbers below are the BIOBUZZ match structure. There is <b>no endgame period</b>: a clock
 * reads {@link Phase#RUNNING} until the buzzer, and the only thing the robot does with time is
 * warn the drivers in the final seconds ({@link #isFinalSeconds()}) and stop at the buzzer
 * ({@link #isExpired()}).
 *
 * <h2>The timestamp is passed in</h2>
 * This class never reads a clock of its own — the caller hands the current millisecond count to
 * {@link #start(long)} and {@link #update(long)}. The rule that makes that worth doing: the loop
 * samples {@code System.currentTimeMillis()} <em>once</em> and passes the same value to everything
 * it drives that loop. Every time-based decision in one iteration then agrees with every other, and
 * a slow loop cannot produce a clock that expired halfway through its own telemetry.
 *
 * <p>Durations are plain {@code public static} fields because off-season scrimmages and practice
 * sessions routinely run non-standard periods.
 */
public class MatchClock {
    // BIOBUZZ match structure (Competition Manual V1, sections 10.1 and 10.4, Table 9-1):
    // AUTO 0:30, an 8 s transition with no powered movement (G403), TELEOP 2:00. There is no
    // separate endgame period; the field timer sounds a final warning at 0:20. The visual field
    // timer is authoritative over audio cues (section 9.11). See docs/04-biobuzz-season-analysis.md.

    /** BIOBUZZ autonomous period. */
    public static long AUTONOMOUS_MS = 30_000;
    /** BIOBUZZ driver-controlled period. */
    public static long TELEOP_MS = 120_000;
    /** The field's final warning (train whistle) sounds with this much time left. */
    public static long FINAL_WARNING_MS = 20_000;

    /** Which match period this clock is counting. Chosen by the OpMode; only the length differs. */
    public enum Period { AUTONOMOUS, TELEOP }

    /** Where we are in the period. */
    public enum Phase { NOT_STARTED, RUNNING, EXPIRED }

    private final long durationMs;

    private long startMs = 0;
    private long nowMs = 0;
    private boolean started = false;

    private MatchClock(long durationMs) {
        this.durationMs = Math.max(0, durationMs);
    }

    /** The clock for a period: {@link #AUTONOMOUS_MS} or {@link #TELEOP_MS} long. */
    public static MatchClock forPeriod(Period period) {
        return new MatchClock(period == Period.AUTONOMOUS ? AUTONOMOUS_MS : TELEOP_MS);
    }

    /** Marks the start of the period. Call once, from the OpMode's {@code start()}. */
    public void start(long nowMs) {
        this.startMs = nowMs;
        this.nowMs = nowMs;
        this.started = true;
    }

    /** Advances the clock. Call once per loop, before anything that reads it. */
    public void update(long nowMs) {
        this.nowMs = nowMs;
    }

    /** Milliseconds since {@link #start}, or 0 before it. Never negative. */
    public long getElapsedMs() {
        if (!started) return 0;
        return Math.max(0, nowMs - startMs);
    }

    /** Milliseconds left in the period. Clamped at 0: never counts past the buzzer. */
    public long getRemainingMs() {
        if (!started) return durationMs;
        return Math.max(0, durationMs - getElapsedMs());
    }

    public double getRemainingSeconds() {
        return getRemainingMs() / 1000.0;
    }

    public Phase getPhase() {
        if (!started) return Phase.NOT_STARTED;
        return getRemainingMs() <= 0 ? Phase.EXPIRED : Phase.RUNNING;
    }

    /** True in the final {@link #FINAL_WARNING_MS} of a started period. */
    public boolean isFinalSeconds() {
        return started && getRemainingMs() <= FINAL_WARNING_MS && getRemainingMs() > 0;
    }

    public boolean isExpired() {
        return getPhase() == Phase.EXPIRED;
    }

    /** Compact status for telemetry, e.g. {@code "RUNNING 24.6s"}. */
    public String getStatus() {
        if (!started) return "not started";
        return getPhase() + String.format(java.util.Locale.US, " %.1fs", getRemainingSeconds());
    }
}
