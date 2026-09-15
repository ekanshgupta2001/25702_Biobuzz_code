package org.firstinspires.ftc.teamcode.util.diagnostics;

import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertTrue;

import org.junit.After;
import org.junit.Before;
import org.junit.Test;

import java.util.List;

/** The tunable snapshot against a small class of knobs, so the match cards can name a bench edit. */
public class TunablesTest {
    /** Stands in for a subsystem: two tunables, one constant, one private. */
    public static final class Knobs {
        public static double GAIN = 1.5;
        public static boolean ENABLED = false;
        public static final int FIXED = 3;
        private static int hidden = 7;
    }

    @Before
    public void setUp() {
        Tunables.resetForTests();
        Knobs.GAIN = 1.5;
        Knobs.ENABLED = false;
    }

    @After
    public void tearDown() {
        Tunables.resetForTests();
        Knobs.GAIN = 1.5;
        Knobs.ENABLED = false;
    }

    @Test
    public void reportsWhatDiffersFromTheSnapshotAndRestoresIt() {
        // fixthese R2-A5: a static bumped on a bench is what the next Teleop runs with, and nothing
        // showed it.
        Tunables.snapshot(Knobs.class);
        assertEquals("public static non-final only", 2, Tunables.size());
        assertTrue(Tunables.changed().isEmpty());

        Knobs.GAIN = 2.0;
        Knobs.ENABLED = true;
        List<String> changed = Tunables.changed();
        assertEquals(2, changed.size());
        assertEquals("Knobs.GAIN = 2.0 (default 1.5)", changed.get(0));
        assertEquals("Knobs.ENABLED = true (default false)", changed.get(1));

        assertEquals(2, Tunables.restoreDefaults());
        assertEquals(1.5, Knobs.GAIN, 1e-9);
        assertTrue(!Knobs.ENABLED);
        assertTrue(Tunables.changed().isEmpty());
        assertEquals("nothing left to restore", 0, Tunables.restoreDefaults());
    }

    @Test
    public void theFirstSnapshotOfAClassWins() {
        Tunables.snapshot(Knobs.class);
        Knobs.GAIN = 9;
        Tunables.snapshot(Knobs.class);             // a second OpMode's init, after a bench edit
        assertEquals("the compiled value is still the default", 1, Tunables.changed().size());
        assertTrue(Tunables.changed().get(0).contains("default 1.5"));
    }
}
