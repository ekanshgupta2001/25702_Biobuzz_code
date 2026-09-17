package org.firstinspires.ftc.teamcode.util.time;

import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertFalse;
import static org.junit.Assert.assertTrue;

import org.junit.Test;

/**
 * These pass because {@link MatchClock} takes its timestamps as arguments rather than reading a
 * system clock: the whole two-minute period is exercised here in microseconds.
 */
public class MatchClockTest {
    /** An arbitrary epoch, chosen to prove nothing depends on the clock starting near zero. */
    private static final long T0 = 1_700_000_000_000L;

    private static MatchClock teleop() {
        return MatchClock.forPeriod(MatchClock.Period.TELEOP);
    }

    @Test
    public void reportsNotStartedBeforeStart() {
        MatchClock clock = teleop();
        assertFalse(clock.isStarted());
        assertEquals(MatchClock.Phase.NOT_STARTED, clock.getPhase());
        assertEquals(0, clock.getElapsedMs());
        assertEquals("not started", clock.getStatus());
    }

    @Test
    public void reportsFullPeriodRemainingBeforeStart() {
        // Not zero: a routine that asks "how long is left?" before the match begins should be told
        // the whole period, not that time has run out.
        assertEquals(MatchClock.TELEOP_MS, teleop().getRemainingMs());
        assertEquals(MatchClock.AUTONOMOUS_MS, MatchClock.forPeriod(MatchClock.Period.AUTONOMOUS).getRemainingMs());
    }

    @Test
    public void elapsedAndRemainingTrackTime() {
        MatchClock clock = teleop();
        clock.start(T0);
        clock.update(T0 + 30_000);

        assertEquals(30_000, clock.getElapsedMs());
        assertEquals(MatchClock.TELEOP_MS - 30_000, clock.getRemainingMs());
        assertEquals(MatchClock.Phase.RUNNING, clock.getPhase());
        assertEquals(MatchClock.Period.TELEOP, clock.getPeriod());
    }

    @Test
    public void teleopClockHasNoEndgameInBiobuzz() {
        MatchClock clock = teleop();
        clock.start(T0);
        clock.update(T0 + MatchClock.TELEOP_MS - 30_000);
        assertEquals("BIOBUZZ has no endgame period", MatchClock.Phase.RUNNING, clock.getPhase());
        assertEquals("RUNNING 30.0s", clock.getStatus());
        clock.update(T0 + MatchClock.TELEOP_MS - 1);
        assertEquals(MatchClock.Phase.RUNNING, clock.getPhase());
    }

    @Test
    public void expiresAtTheBuzzerAndStaysExpired() {
        MatchClock clock = teleop();
        clock.start(T0);

        clock.update(T0 + MatchClock.TELEOP_MS);
        assertEquals(MatchClock.Phase.EXPIRED, clock.getPhase());
        assertTrue(clock.isExpired());

        // Long past the end it must still read EXPIRED, not wrap or go negative.
        clock.update(T0 + MatchClock.TELEOP_MS + 600_000);
        assertEquals(MatchClock.Phase.EXPIRED, clock.getPhase());
    }

    @Test
    public void remainingNeverGoesNegative() {
        MatchClock clock = teleop();
        clock.start(T0);
        clock.update(T0 + MatchClock.TELEOP_MS + 10_000);
        assertEquals(0, clock.getRemainingMs());
    }

    @Test
    public void elapsedNeverGoesNegativeIfTimeMovesBackwards() {
        // Defensive: a caller mixing two time sources must not produce a negative elapsed time.
        MatchClock clock = teleop();
        clock.start(T0);
        clock.update(T0 - 5_000);
        assertEquals(0, clock.getElapsedMs());
    }

    @Test
    public void autonomousClockExpiresAtThirtySeconds() {
        MatchClock clock = MatchClock.forPeriod(MatchClock.Period.AUTONOMOUS);
        clock.start(T0);
        clock.update(T0 + MatchClock.AUTONOMOUS_MS - 1);
        assertEquals(MatchClock.Phase.RUNNING, clock.getPhase());
        clock.update(T0 + MatchClock.AUTONOMOUS_MS);
        assertEquals(MatchClock.Phase.EXPIRED, clock.getPhase());
    }

    @Test
    public void finalSecondsFlagMatchesTheFieldWarning() {
        MatchClock clock = teleop();
        assertFalse("never before start", clock.isFinalSeconds());
        clock.start(T0);
        clock.update(T0 + MatchClock.TELEOP_MS - MatchClock.FINAL_WARNING_MS - 1);
        assertFalse(clock.isFinalSeconds());
        clock.update(T0 + MatchClock.TELEOP_MS - MatchClock.FINAL_WARNING_MS);
        assertTrue(clock.isFinalSeconds());
        clock.update(T0 + MatchClock.TELEOP_MS);
        assertFalse("not once expired", clock.isFinalSeconds());
    }
}
