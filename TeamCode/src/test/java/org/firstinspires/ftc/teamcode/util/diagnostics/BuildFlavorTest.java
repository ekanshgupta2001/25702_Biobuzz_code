package org.firstinspires.ftc.teamcode.util.diagnostics;

import static org.junit.Assert.assertFalse;
import static org.junit.Assert.assertTrue;

import org.junit.Test;

/** The runtime check for the AutoTune library (fixthese R2-A7). The real marker is not asserted, so this passes under either build. */
public class BuildFlavorTest {
    @Test
    public void detectsWhetherAClassIsOnTheClasspath() {
        assertTrue(BuildFlavor.classPresent("java.lang.String"));
        assertFalse(BuildFlavor.classPresent("com.pedropathing.tuning.NoSuchClass"));
        boolean first = BuildFlavor.isTuningBuild();
        assertTrue("cached, consistent", first == BuildFlavor.isTuningBuild());
        assertTrue(BuildFlavor.TUNING_WARNING.contains("R704"));
    }
}
