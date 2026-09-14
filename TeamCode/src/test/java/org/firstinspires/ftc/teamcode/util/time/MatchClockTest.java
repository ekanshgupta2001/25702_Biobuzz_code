package org.firstinspires.ftc.teamcode.util.time;

import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertFalse;
import static org.junit.Assert.assertTrue;

import org.junit.Test;

/**
 * These pass because {@link MatchClock} takes its timestamps as arguments rather than reading a
 * system clock — the whole two-minute period is exercised here in microseconds.
 */
public class MatchClockTest {
    /** An arbitrary epoch, chosen to prove nothing depends on the clock starting near zero. */
    private static final long T0 = 1_700_000_000_000L;

    @Test
    public void reportsNotStartedBeforeStart() {
        MatchClock clock = MatchClock.forTeleop();
        assertFalse(clock.isStarted());
        assertEquals(MatchClock.Phase.NOT_STARTED, clock.getPhase());
        assertEquals(0, clock.getElapsedMs());
    }

    @Test
    public void reportsFullPeriodRemainingBeforeStart() {
        // Not zero: a routine that asks "how long is left?" before the match begins should be told
        // the whole period, not that time has run out.
        assertEquals(MatchClock.TELEOP_MS, MatchClock.forTeleop().getRemainingMs());
    }

    @Test
    public void elapsedAndRemainingTrackTime() {
        MatchClock clock = MatchClock.forTeleop();
        clock.start(T0);
        clock.update(T0 + 30_000);

        assertEquals(30_000, clock.getElapsedMs());
        assertEquals(MatchClock.TELEOP_MS - 30_000, clock.getRemainingMs());
        assertEquals(MatchClock.Phase.RUNNING, clock.getPhase());
    }

    @Test
    public void entersEndgameAtTheBoundary() {
        MatchClock clock = MatchClock.forTeleop();
        clock.start(T0);

        // One millisecond before the boundary is still RUNNING.
        clock.update(T0 + MatchClock.TELEOP_MS - MatchClock.ENDGAME_MS - 1);
        assertEquals(MatchClock.Phase.RUNNING, clock.getPhase());
        assertFalse(clock.isEndgame());

        // Exactly at the boundary, endgame has begun.
        clock.update(T0 + MatchClock.TELEOP_MS - MatchClock.ENDGAME_MS);
        assertEquals(MatchClock.Phase.ENDGAME, clock.getPhase());
        assertTrue(clock.isEndgame());
    }

    @Test
    public void expiresAtTheBuzzerAndStaysExpired() {
        MatchClock clock = MatchClock.forTeleop();
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
        MatchClock clock = MatchClock.forTeleop();
        clock.start(T0);
        clock.update(T0 + MatchClock.TELEOP_MS + 10_000);
        assertEquals(0, clock.getRemainingMs());
    }

    @Test
    public void elapsedNeverGoesNegativeIfTimeMovesBackwards() {
        // Defensive: a caller mixing two time sources should not produce a negative elapsed time
        // that makes hasTimeFor() answer nonsense.
        MatchClock clock = MatchClock.forTeleop();
        clock.start(T0);
        clock.update(T0 - 5_000);
        assertEquals(0, clock.getElapsedMs());
    }

    @Test
    public void autonomousClockHasNoEndgame() {
        MatchClock clock = MatchClock.forAutonomous();
        clock.start(T0);

        // Deep into the period, where a teleop clock would be in endgame.
        clock.update(T0 + MatchClock.AUTONOMOUS_MS - 1);
        assertEquals(MatchClock.Phase.RUNNING, clock.getPhase());
        assertFalse(clock.isEndgame());

        clock.update(T0 + MatchClock.AUTONOMOUS_MS);
        assertEquals(MatchClock.Phase.EXPIRED, clock.getPhase());
    }

    @Test
    public void hasTimeForComparesAgainstWhatIsLeft() {
        MatchClock clock = MatchClock.forTeleop();
        clock.start(T0);
        clock.update(T0 + MatchClock.TELEOP_MS - 5_000);   // five seconds left

        assertTrue(clock.hasTimeFor(4_999));
        assertTrue(clock.hasTimeFor(5_000));    // exactly enough still counts
        assertFalse(clock.hasTimeFor(5_001));
    }

    @Test
    public void hasTimeForIsPermissiveBeforeTheClockStarts() {
        // A routine given no clock must behave exactly as it did before this class existed, rather
        // than silently skipping every action because it believes there is no time.
        assertTrue(MatchClock.forTeleop().hasTimeFor(999_999));
    }

    @Test
    public void endgameCannotOutlastTheWholePeriod() {
        // A misconfigured endgame longer than the period would otherwise make the clock report
        // ENDGAME from the first millisecond.
        MatchClock clock = MatchClock.of(MatchClock.Period.TELEOP, 10_000, 30_000);
        clock.start(T0);
        clock.update(T0);
        assertEquals(MatchClock.Phase.ENDGAME, clock.getPhase());
        assertEquals(10_000, clock.getRemainingMs());
    }

    @Test
    public void flowerUnlockOpensAtOneMinuteRemainingInTeleopOnly() {
        MatchClock clock = MatchClock.forTeleop();
        assertFalse("never before start", clock.isFlowerUnlocked());
        clock.start(T0);
        clock.update(T0 + MatchClock.TELEOP_MS - MatchClock.FLOWER_UNLOCK_MS - 1);
        assertFalse(clock.isFlowerUnlocked());
        clock.update(T0 + MatchClock.TELEOP_MS - MatchClock.FLOWER_UNLOCK_MS);
        assertTrue(clock.isFlowerUnlocked());
        clock.update(T0 + MatchClock.TELEOP_MS);
        assertFalse("closed once the period expires", clock.isFlowerUnlocked());

        MatchClock auto = MatchClock.forAutonomous();
        auto.start(T0);
        auto.update(T0 + MatchClock.AUTONOMOUS_MS - 1);
        assertFalse("autonomous never unlocks the FLOWER", auto.isFlowerUnlocked());
    }

    @Test
    public void finalSecondsFlagMatchesTheFieldWarning() {
        MatchClock clock = MatchClock.forTeleop();
        clock.start(T0);
        clock.update(T0 + MatchClock.TELEOP_MS - MatchClock.FINAL_WARNING_MS - 1);
        assertFalse(clock.isFinalSeconds());
        clock.update(T0 + MatchClock.TELEOP_MS - MatchClock.FINAL_WARNING_MS);
        assertTrue(clock.isFinalSeconds());
    }
}
