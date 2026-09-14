package org.firstinspires.ftc.teamcode.util.math;

import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertTrue;

import org.junit.Test;

/**
 * The 0/2pi seam, in isolation.
 *
 * <p>These cases used to live in {@code VisionMathTest}, which was the clue that the functions
 * themselves were in the wrong class: none of them involve a camera.
 */
public class AnglesTest {
    private static final double LOOSE = 1e-6;
    private static final double EPS = 1e-9;

    @Test
    public void headingTowardIsNormalizedAndCorrect() {
        // Straight ahead while facing 0 -> heading 0.
        assertEquals(0, Angles.headingToward(0, 10, 0), LOOSE);
        // Directly left while facing 0 -> 90 degrees.
        assertEquals(Math.PI / 2, Angles.headingToward(0, 0, 10), LOOSE);
        // Directly right while facing 0 -> 270 degrees, not -90.
        assertEquals(3 * Math.PI / 2, Angles.headingToward(0, 0, -10), LOOSE);
    }

    @Test
    public void normalizeAngleAlwaysLandsInZeroToTwoPi() {
        double[] inputs = {-7, -Math.PI, 0, Math.PI, 7, 100};
        for (double in : inputs) {
            double out = Angles.normalizeAngle(in);
            assertTrue("out of range for " + in, out >= 0 && out < 2 * Math.PI + EPS);
        }
        assertEquals(Math.PI, Angles.normalizeAngle(-Math.PI), LOOSE);
    }

    @Test
    public void angleErrorTakesTheShortWayAround() {
        assertEquals(0.2, Angles.angleError(0.1, 0.3), LOOSE);
        // From 350 to 10 degrees is +20, not -340.
        assertEquals(Math.toRadians(20),
                Angles.angleError(Math.toRadians(350), Math.toRadians(10)), LOOSE);
        assertEquals(Math.toRadians(-20),
                Angles.angleError(Math.toRadians(10), Math.toRadians(350)), LOOSE);
    }

    @Test
    public void angleErrorIsAntisymmetric() {
        // error(a, b) and error(b, a) must be equal and opposite, or a controller fed one of them
        // would settle in a different place than one fed the other.
        for (double a = 0; a < 2 * Math.PI; a += 0.37) {
            for (double b = 0; b < 2 * Math.PI; b += 0.41) {
                double forward = Angles.angleError(a, b);
                double back = Angles.angleError(b, a);
                if (Math.abs(Math.abs(forward) - Math.PI) < LOOSE) continue;  // pi is its own opposite
                assertEquals(forward, -back, LOOSE);
            }
        }
    }

    @Test
    public void angleErrorIsNeverMoreThanHalfATurn() {
        // The property that stops a heading controller driving the long way round at full power.
        for (double a = -20; a < 20; a += 0.13) {
            for (double b = -20; b < 20; b += 0.29) {
                assertTrue(Math.abs(Angles.angleError(a, b)) <= Math.PI + LOOSE);
            }
        }
    }

    @Test
    public void normalizeIsIdempotent() {
        for (double a = -20; a < 20; a += 0.17) {
            double once = Angles.normalizeAngle(a);
            assertEquals(once, Angles.normalizeAngle(once), LOOSE);
        }
    }
}
