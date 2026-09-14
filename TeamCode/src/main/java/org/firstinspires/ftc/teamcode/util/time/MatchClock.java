package org.firstinspires.ftc.teamcode.util.time;

/**
 * Tracks how much of the match period is left.
 *
 * <h2>BIOBUZZ</h2>
 * The numbers below are the BIOBUZZ match structure. There is <b>no endgame period</b>: a teleop
 * clock reads {@link Phase#RUNNING} until the buzzer. The only time-gated rule in the game is the
 * 1:00 FLOWER unlock, exposed as {@link #isFlowerUnlocked()} (V1 has no FLOWER mechanism, so nothing
 * reads it yet). {@link Phase#ENDGAME} remains reachable only through {@link #of} for off-season
 * games that have one.
 *
 * <h2>Why this exists</h2>
 * Without a clock the robot cannot make any decision that depends on time — it cannot warn the
 * drivers that endgame has started, cannot decide that there is no longer room for a full scoring
 * cycle, and cannot record in the log which phase a problem happened in. Every one of those needs
 * the same three numbers, so they live here once.
 *
 * <h2>Time is passed in, not read</h2>
 * This class never calls {@code System.currentTimeMillis()} or touches an {@code ElapsedTime}. The
 * caller supplies the timestamp to {@link #start(long)} and {@link #update(long)}. That is what
 * makes it unit-testable off-robot — the same reason {@code VisionMath} and {@code PoseFusion} are
 * separate from the subsystems that use them. Tests can advance time by an hour instantly.
 *
 * <pre>
 *   // in an OpMode
 *   clock = MatchClock.forTeleop();
 *   clock.start(System.currentTimeMillis());   // in start()
 *   clock.update(System.currentTimeMillis());  // top of every loop
 * </pre>
 *
 * <p>Durations are plain {@code public static} fields because off-season scrimmages and practice sessions
 * routinely run non-standard periods, and a wrong endgame warning is worse than none.
 */
public class MatchClock {
    // BIOBUZZ match structure (Competition Manual V1, sections 10.1 and 10.4, Table 9-1):
    // AUTO 0:30, an 8 s transition with no powered movement (G403), TELEOP 2:00. There is no
    // separate endgame period. The one timed unlock is at 1:00 remaining, when FLOWER ownership
    // scoring opens and NECTAR may enter a FLOWER (G410) and all remaining NECTAR may be loaded
    // (G426); the field timer sounds a final warning at 0:20. The visual field timer is
    // authoritative over audio cues (section 9.11). See docs/04-biobuzz-season-analysis.md.

    /** BIOBUZZ autonomous period. */
    public static long AUTONOMOUS_MS = 30_000;
    /** Transition between AUTO and TELEOP during which no powered movement is allowed (G403). */
    public static long TRANSITION_MS = 8_000;
    /** BIOBUZZ driver-controlled period, inclusive of the FLOWER-unlock window. */
    public static long TELEOP_MS = 120_000;
    /** FLOWER ownership scoring opens, and NECTAR may enter a FLOWER, with this much time left (G410). */
    public static long FLOWER_UNLOCK_MS = 60_000;
    /** The field's final warning (train whistle) sounds with this much time left. */
    public static long FINAL_WARNING_MS = 20_000;

    /** Which match period this clock is counting. */
    public enum Period { AUTONOMOUS, TELEOP }

    /**
     * Where we are in the period.
     *
     * <p>{@link #ENDGAME} never occurs on a BIOBUZZ clock ({@link #forTeleop()} has no endgame);
     * only a clock built with {@link #of} and a non-zero {@code endgameMs} reports it.
     */
    public enum Phase { NOT_STARTED, RUNNING, ENDGAME, EXPIRED }

    private final Period period;
    private final long durationMs;
    private final long endgameMs;

    private long startMs = 0;
    private long nowMs = 0;
    private boolean started = false;

    private MatchClock(Period period, long durationMs, long endgameMs) {
        this.period = period;
        this.durationMs = Math.max(0, durationMs);
        this.endgameMs = Math.max(0, Math.min(endgameMs, this.durationMs));
    }

    /** A clock for the autonomous period. Has no endgame. */
    public static MatchClock forAutonomous() {
        return new MatchClock(Period.AUTONOMOUS, AUTONOMOUS_MS, 0);
    }

    /** A clock for the BIOBUZZ driver-controlled period: no endgame, RUNNING until the buzzer. */
    public static MatchClock forTeleop() {
        return new MatchClock(Period.TELEOP, TELEOP_MS, 0);
    }

    /** A clock with explicit durations, for practice periods that are not match length. */
    public static MatchClock of(Period period, long durationMs, long endgameMs) {
        return new MatchClock(period, durationMs, endgameMs);
    }

    /** The standard clock for a period: {@link #forAutonomous()} or {@link #forTeleop()}. */
    public static MatchClock forPeriod(Period period) {
        return period == Period.AUTONOMOUS ? forAutonomous() : forTeleop();
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

    public boolean isStarted() {
        return started;
    }

    public Period getPeriod() {
        return period;
    }

    /** Milliseconds since {@link #start}, or 0 before it. Never negative. */
    public long getElapsedMs() {
        if (!started) return 0;
        return Math.max(0, nowMs - startMs);
    }

    /** Milliseconds left in the period. Clamped at 0 — never counts past the buzzer. */
    public long getRemainingMs() {
        if (!started) return durationMs;
        return Math.max(0, durationMs - getElapsedMs());
    }

    public double getRemainingSeconds() {
        return getRemainingMs() / 1000.0;
    }

    public Phase getPhase() {
        if (!started) return Phase.NOT_STARTED;
        long remaining = getRemainingMs();
        if (remaining <= 0) return Phase.EXPIRED;
        if (endgameMs > 0 && remaining <= endgameMs) return Phase.ENDGAME;
        return Phase.RUNNING;
    }

    public boolean isEndgame() {
        return getPhase() == Phase.ENDGAME;
    }

    /**
     * True once NECTAR may legally enter a FLOWER and FLOWER ownership scoring is open: teleop
     * with at most {@link #FLOWER_UNLOCK_MS} remaining (G410, G426). False before the clock starts,
     * so a routine without a clock never scores NECTAR into a FLOWER early.
     */
    public boolean isFlowerUnlocked() {
        return started && period == Period.TELEOP && getRemainingMs() <= FLOWER_UNLOCK_MS
                && getRemainingMs() > 0;
    }

    /** True in the final {@link #FINAL_WARNING_MS} of a started period. */
    public boolean isFinalSeconds() {
        return started && getRemainingMs() <= FINAL_WARNING_MS && getRemainingMs() > 0;
    }

    public boolean isExpired() {
        return getPhase() == Phase.EXPIRED;
    }

    /**
     * Whether {@code budgetMs} of work still fits before the buzzer.
     *
     * <p>This is the hook for time-aware fallbacks: ask before committing to a long cycle, and take
     * the short one when the answer is no. Returns {@code true} before the clock has started, so a
     * routine that is never given a clock behaves exactly as it did before this class existed.
     */
    public boolean hasTimeFor(long budgetMs) {
        if (!started) return true;
        return getRemainingMs() >= budgetMs;
    }

    /** Compact status for telemetry, e.g. {@code "RUNNING 24.6s"}. */
    public String getStatus() {
        if (!started) return "not started";
        return getPhase() + String.format(java.util.Locale.US, " %.1fs", getRemainingSeconds());
    }
}
