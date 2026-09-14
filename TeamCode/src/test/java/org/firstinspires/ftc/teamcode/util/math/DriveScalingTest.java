package org.firstinspires.ftc.teamcode.util.math;

import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertTrue;

import org.junit.Test;

/** Unit tests for stick shaping. Runs on a laptop with no robot: ./gradlew :TeamCode:test */
public class DriveScalingTest {
    private static final double EPS = 1e-9;

    @Test
    public void deadbandZeroesSmallInput() {
        assertEquals(0.0, DriveScaling.applyDeadband(0.05, 0.1), EPS);
        assertEquals(0.0, DriveScaling.applyDeadband(-0.05, 0.1), EPS);
    }

    @Test
    public void deadbandStillReachesFullMagnitude() {
        // The rescale is the whole point: without /(1-deadband) full deflection would fall short.
        assertEquals(1.0, DriveScaling.applyDeadband(1.0, 0.1), EPS);
        assertEquals(-1.0, DriveScaling.applyDeadband(-1.0, 0.1), EPS);
    }

    @Test
    public void deadbandResumesFromZeroNotFromTheEdge() {
        // Just past the deadband the output should be near 0, not jump to 0.1.
        double justPast = DriveScaling.applyDeadband(0.1001, 0.1);
        assertTrue("expected near-zero, got " + justPast, justPast < 0.01);
    }

    @Test
    public void deadbandOfOneDoesNotDivideByZero() {
        assertEquals(0.0, DriveScaling.applyDeadband(0.5, 1.0), EPS);
        assertEquals(0.0, DriveScaling.applyDeadband(1.0, 1.5), EPS);
    }

    @Test
    public void expoPreservesSignAndEndpoints() {
        assertEquals(1.0, DriveScaling.applyExpo(1.0, 2.0), EPS);
        assertEquals(-1.0, DriveScaling.applyExpo(-1.0, 2.0), EPS);
        assertEquals(0.0, DriveScaling.applyExpo(0.0, 2.0), EPS);
        assertEquals(-0.25, DriveScaling.applyExpo(-0.5, 2.0), EPS);
    }

    @Test
    public void expoIsSymmetric() {
        for (double v = 0.1; v <= 1.0; v += 0.1) {
            assertEquals(-DriveScaling.applyExpo(v, 3.0), DriveScaling.applyExpo(-v, 3.0), EPS);
        }
    }

    @Test
    public void shapeIsMonotonicAndBounded() {
        double prev = -1.1;
        for (double v = -1.0; v <= 1.0; v += 0.05) {
            double out = DriveScaling.shape(v);
            assertTrue("not monotonic at " + v, out >= prev - EPS);
            assertTrue("out of range at " + v, out >= -1.0 - EPS && out <= 1.0 + EPS);
            prev = out;
        }
    }

    @Test
    public void slowScaleIsFullSpeedBelowThreshold() {
        assertEquals(1.0, DriveScaling.slowScale(0.0, 0.3, 0.1), EPS);
        assertEquals(1.0, DriveScaling.slowScale(0.09, 0.3, 0.1), EPS);
    }

    @Test
    public void slowScaleReachesMinAtFullPull() {
        assertEquals(0.3, DriveScaling.slowScale(1.0, 0.3, 0.1), EPS);
    }

    @Test
    public void slowScaleDecreasesAsTriggerIsPulled() {
        double prev = 1.1;
        for (double t = 0.1; t <= 1.0; t += 0.05) {
            double s = DriveScaling.slowScale(t, 0.3, 0.1);
            assertTrue("should decrease at " + t, s <= prev + EPS);
            prev = s;
        }
    }

    @Test
    public void slowScaleThresholdOfOneDoesNotDivideByZero() {
        assertEquals(1.0, DriveScaling.slowScale(1.0, 0.3, 1.0), EPS);
    }
}
