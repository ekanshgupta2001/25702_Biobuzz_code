package org.firstinspires.ftc.teamcode.util.control;

import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertFalse;
import static org.junit.Assert.assertTrue;

import org.junit.Before;
import org.junit.Test;

/**
 * The anti-jam state machine, exercised without a motor.
 *
 * <p>This is the logic that used to live inside {@code Intake}, where none of it could be tested —
 * every case below would have needed a real robot, a real jam, and a stopwatch.
 */
public class JamDetectorTest {
    private static final double STALL_AMPS = 5.0;
    private static final long STALL_MS = 200;
    private static final long UNJAM_MS = 150;
    private static final int MAX_ATTEMPTS = 3;

    private static final double QUIET = 1.0;    // well under the threshold
    private static final double STALLED = 8.0;  // well over it

    private JamDetector detector;

    @Before
    public void setUp() {
        detector = new JamDetector();
        detector.configure(STALL_AMPS, STALL_MS, UNJAM_MS, MAX_ATTEMPTS);
    }

    @Test
    public void normalRunningNeverUnjams() {
        for (long t = 0; t < 5_000; t += 20) {
            assertFalse(detector.update(t, true, QUIET));
        }
        assertEquals(0, detector.getAttempts());
        assertFalse(detector.isStallSuspected());
    }

    @Test
    public void briefCurrentSpikeIsNotAJam() {
        // A motor accelerating from rest draws stall current momentarily. Reacting to that would
        // eject a game piece every single time the intake spun up.
        assertFalse(detector.update(0, true, STALLED));
        assertTrue(detector.isStallSuspected());

        assertFalse(detector.update(100, true, STALLED));   // still inside the dwell
        assertFalse(detector.update(120, true, QUIET));     // current recovered

        assertFalse(detector.isStallSuspected());
        assertEquals(0, detector.getAttempts());
    }

    @Test
    public void sustainedOverCurrentTriggersAnUnjam() {
        assertFalse(detector.update(0, true, STALLED));
        assertFalse(detector.update(STALL_MS - 1, true, STALLED));

        // The dwell has now elapsed, so this is a real jam.
        assertTrue(detector.update(STALL_MS, true, STALLED));
        assertEquals(1, detector.getAttempts());
    }

    @Test
    public void unjamRunsForItsFullDurationThenStops() {
        detector.update(0, true, STALLED);
        assertTrue(detector.update(STALL_MS, true, STALLED));   // reversal begins at t=200

        assertTrue(detector.isUnjamming(STALL_MS + 1));
        assertTrue(detector.update(STALL_MS + UNJAM_MS - 1, true, STALLED));

        // At exactly the end of the window the reversal is over.
        assertFalse(detector.isUnjamming(STALL_MS + UNJAM_MS));
    }

    @Test
    public void attemptsAreBoundedSoAHardJamCannotCookTheMotor() {
        long t = 0;
        for (int i = 0; i < MAX_ATTEMPTS; i++) {
            detector.update(t, true, STALLED);              // start the dwell
            t += STALL_MS;
            assertTrue("attempt " + (i + 1), detector.update(t, true, STALLED));
            t += UNJAM_MS;                                   // let the reversal finish
        }
        assertEquals(MAX_ATTEMPTS, detector.getAttempts());
        assertTrue(detector.hasGivenUp());

        // Past the limit it stops trying, however long the current stays high.
        for (int i = 0; i < 100; i++) {
            t += STALL_MS;
            assertFalse(detector.update(t, true, STALLED));
        }
        assertEquals(MAX_ATTEMPTS, detector.getAttempts());
    }

    @Test
    public void recoveryClearsTheAttemptCount() {
        // The limit bounds CONSECUTIVE failed attempts. A jam that clears and recurs later in the
        // match should get the full budget again, not inherit the earlier one.
        detector.update(0, true, STALLED);
        assertTrue(detector.update(STALL_MS, true, STALLED));
        assertEquals(1, detector.getAttempts());

        long afterUnjam = STALL_MS + UNJAM_MS;
        assertFalse(detector.update(afterUnjam, true, QUIET));
        assertEquals(0, detector.getAttempts());
        assertFalse(detector.hasGivenUp());
    }

    @Test
    public void ineligibleNeverUnjamsHoweverHighTheCurrent() {
        // This is the one that matters most. Holding a captured piece against a hard stop IS a
        // stalled motor; a detector running during HOLDING would eject the piece it just collected.
        for (long t = 0; t < 5_000; t += 20) {
            assertFalse(detector.update(t, false, STALLED));
        }
        assertEquals(0, detector.getAttempts());
    }

    @Test
    public void becomingIneligibleDiscardsAPartialStall() {
        detector.update(0, true, STALLED);
        assertTrue(detector.isStallSuspected());

        detector.update(50, false, STALLED);          // mode changed away from intaking
        assertFalse(detector.isStallSuspected());

        // Back to intaking: the dwell starts over rather than firing immediately on the stale timer.
        assertFalse(detector.update(60, true, STALLED));
        assertFalse(detector.update(60 + STALL_MS - 1, true, STALLED));
        assertTrue(detector.update(60 + STALL_MS, true, STALLED));
    }

    @Test
    public void givingUpSurvivesAnIneligibleLoop() {
        long t = 0;
        for (int i = 0; i < MAX_ATTEMPTS; i++) {
            detector.update(t, true, STALLED);
            t += STALL_MS;
            detector.update(t, true, STALLED);
            t += UNJAM_MS;
        }
        assertTrue(detector.hasGivenUp());

        detector.update(t, false, QUIET);     // one loop where anti-jam does not apply
        assertTrue("an ineligible loop must not silently re-arm the detector",
                detector.hasGivenUp());
    }

    @Test
    public void resetClearsEverything() {
        detector.update(0, true, STALLED);
        detector.update(STALL_MS, true, STALLED);
        assertEquals(1, detector.getAttempts());

        detector.reset();

        assertEquals(0, detector.getAttempts());
        assertFalse(detector.isStallSuspected());
        assertFalse(detector.isUnjamming(STALL_MS + 1));
    }

    @Test
    public void configureTakesEffectImmediately() {
        // Live dashboard tuning has to reach the running detector, not just future ones.
        detector.configure(STALL_AMPS, 1_000, UNJAM_MS, MAX_ATTEMPTS);

        detector.update(0, true, STALLED);
        assertFalse("old 200ms dwell must no longer apply", detector.update(STALL_MS, true, STALLED));
        assertTrue(detector.update(1_000, true, STALLED));
    }

    @Test
    public void currentExactlyAtThresholdIsNotAStall() {
        // Strictly greater-than, so a threshold set to the observed running current does not
        // continuously trip.
        for (long t = 0; t < 2_000; t += 20) {
            assertFalse(detector.update(t, true, STALL_AMPS));
        }
        assertFalse(detector.isStallSuspected());
    }
}
