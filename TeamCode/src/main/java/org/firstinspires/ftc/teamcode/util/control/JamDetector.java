package org.firstinspires.ftc.teamcode.util.control;

/**
 * Current-based stall detection with bounded automatic un-jamming.
 *
 * <h2>Why this is not inside Intake</h2>
 * It is the most intricate logic on the robot — three pieces of timing state, an attempt counter,
 * and an eligibility rule that is wrong in two different directions if you get it backwards — and
 * inside a class holding a {@code DcMotorEx} none of it could be tested. Here it is pure state and
 * arithmetic, so every branch below is covered by {@code JamDetectorTest} without a robot.
 *
 * <h2>How it decides</h2>
 * High current alone is not a jam — a motor accelerating from rest draws stall current for a moment.
 * So over-current must <em>persist</em> for {@code stallTimeoutMs} before it counts. Once it does,
 * the caller is told to reverse for {@code unjamDurationMs}, and that is one attempt. Current
 * staying below the threshold for {@code healthyResetMs} clears the attempt count, so the limit
 * bounds <em>consecutive</em> failed attempts rather than attempts over the whole match. That
 * window matters: after every reversal the current dips for a few loops while the motor
 * re-accelerates, and a reset on a single quiet sample would hand a hard jam an unlimited budget.
 *
 * <h2>The eligibility rule is load-bearing</h2>
 * The caller passes {@code eligible}, and must only pass true while actively intaking. Holding a
 * captured game piece against a hard stop is, by definition, a stalled motor: a detector that ran
 * during HOLDING would see hold current, declare a jam, and spit the piece straight back out.
 *
 * <pre>
 *   detector.configure(STALL_AMPS, STALL_MS, UNJAM_MS, MAX_ATTEMPTS);
 *   if (detector.update(now, mode == Mode.INTAKING, amps)) {
 *       motor.setVelocity(UNJAM_TICKS_PER_SEC);
 *   } else {
 *       motor.setVelocity(targetVelocity);
 *   }
 * </pre>
 */
public class JamDetector {
    private double stallCurrentAmps = 5.0;
    private long stallTimeoutMs = 200;
    private long unjamDurationMs = 150;
    private int maxAttempts = 3;
    /** How long current must stay healthy before the attempt count is forgiven. 0 = one sample. */
    private long healthyResetMs = 0;

    /**
     * When over-current was first seen. Paired with {@link #stalling} rather than using 0 as a
     * "not stalling" sentinel: 0 is a perfectly valid timestamp, and a sentinel that overlaps the
     * valid range means a stall beginning at t=0 restarts its own dwell every loop and never fires.
     */
    private long stallStartMs = 0;
    private boolean stalling = false;
    /** Time the active un-jam reversal ends. Only meaningful while {@link #unjamming} is true. */
    private long unjamUntilMs = 0;
    private boolean unjamming = false;
    private int attempts = 0;
    /** Start of the current run of sub-threshold samples; paired with {@link #healthy}. */
    private long healthySinceMs = 0;
    private boolean healthy = false;

    /**
     * Updates the thresholds. Safe to call every loop — that is what keeps dashboard edits to the
     * caller's {@code public static} constants taking effect live.
     */
    public void configure(double stallCurrentAmps, long stallTimeoutMs, long unjamDurationMs,
                          int maxAttempts) {
        configure(stallCurrentAmps, stallTimeoutMs, unjamDurationMs, maxAttempts, 0);
    }

    /** As above, with the healthy-current window that must pass before attempts are forgiven. */
    public void configure(double stallCurrentAmps, long stallTimeoutMs, long unjamDurationMs,
                          int maxAttempts, long healthyResetMs) {
        this.stallCurrentAmps = stallCurrentAmps;
        this.stallTimeoutMs = stallTimeoutMs;
        this.unjamDurationMs = unjamDurationMs;
        this.maxAttempts = maxAttempts;
        this.healthyResetMs = healthyResetMs;
    }

    /**
     * Advances the detector by one loop.
     *
     * @param nowMs    current time in milliseconds
     * @param eligible whether anti-jam applies right now — true only while actively intaking
     * @param amps     motor current this loop
     * @return {@code true} if the caller should drive the un-jam (reverse) velocity this loop
     */
    public boolean update(long nowMs, boolean eligible, double amps) {
        if (!eligible || attempts >= maxAttempts) {
            // Not eligible: forget any partial stall so a brief spike before the mode changed does
            // not carry over and trip on the first eligible loop. The attempt count deliberately
            // survives, so a mechanism that has given up stays given up until something resets it.
            stalling = false;
            return false;
        }

        if (isUnjamming(nowMs)) return true;

        if (amps > stallCurrentAmps) {
            healthy = false;
            if (!stalling) {
                stalling = true;
                stallStartMs = nowMs;
                return false;
            }
            if (nowMs - stallStartMs >= stallTimeoutMs) {
                unjamming = true;
                unjamUntilMs = nowMs + unjamDurationMs;
                attempts++;
                stalling = false;
                return true;
            }
            return false;
        }

        // Current is normal. Only once it has stayed normal for healthyResetMs is the mechanism
        // running freely and any earlier trouble over; a brief dip between reversals is not that.
        stalling = false;
        if (!healthy) {
            healthy = true;
            healthySinceMs = nowMs;
        }
        if (nowMs - healthySinceMs >= healthyResetMs) attempts = 0;
        return false;
    }

    /** True while an un-jam reversal is in progress. */
    public boolean isUnjamming(long nowMs) {
        if (unjamming && nowMs >= unjamUntilMs) unjamming = false;
        return unjamming;
    }

    /**
     * True when over-current has been seen but has not yet lasted long enough to count as a jam.
     *
     * <p>A <em>suspicion</em>, not a verdict — during the reversal itself this reads false and
     * {@link #isUnjamming} reads true.
     */
    public boolean isStallSuspected() {
        return stalling;
    }

    public int getAttempts() {
        return attempts;
    }

    /** True once the detector has stopped trying. The caller should tell the drivers. */
    public boolean hasGivenUp() {
        return attempts >= maxAttempts;
    }

    /**
     * Clears all state, including the attempt count.
     *
     * <p>Call on a deliberate mode change. Without it, re-pressing intake during a reversal would
     * silently keep running the motor backwards instead of doing what was just asked.
     */
    public void reset() {
        resetTiming();
        attempts = 0;
    }

    /** Clears the timing state but keeps the attempt count. */
    public void resetTiming() {
        stalling = false;
        stallStartMs = 0;
        unjamming = false;
        unjamUntilMs = 0;
        healthy = false;
    }
}
