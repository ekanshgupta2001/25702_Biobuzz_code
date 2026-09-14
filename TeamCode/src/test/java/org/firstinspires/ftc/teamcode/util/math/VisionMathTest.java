package org.firstinspires.ftc.teamcode.util.math;

import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertNotNull;
import static org.junit.Assert.assertNull;
import static org.junit.Assert.assertTrue;

import org.junit.Test;

/**
 * Tests for the camera-to-field geometry.
 *
 * <p>The reference numbers come from the closed form
 * {@code forward = dh/tan(P-ty)}, {@code lateral = -forward*tan(tx)*cos(ty)/cos(P-ty)},
 * derived independently of the ray-intersection code under test — so the two have to agree by
 * geometry, not by construction.
 */
public class VisionMathTest {
    private static final double EPS = 1e-6;
    private static final double LOOSE = 1e-3;

    // Mount used throughout: 12" lens, 20 degrees down, no offsets. Target 1.5" tall -> dh = 10.5".
    private static final double CAM_H = 12.0;
    private static final double PITCH = 20.0;
    private static final double TARGET_H = 1.5;
    private static final double DH = CAM_H - TARGET_H;
    private static final double MAX_RANGE = 500;

    private static VisionMath.Mount plainMount() {
        return new VisionMath.Mount(CAM_H, PITCH, 0, 0, 0);
    }

    private static double expectedForward(double ty) {
        return DH / Math.tan(Math.toRadians(PITCH - ty));
    }

    private static double expectedLateral(double tx, double ty) {
        double fwd = expectedForward(ty);
        return -fwd * Math.tan(Math.toRadians(tx))
                * Math.cos(Math.toRadians(ty)) / Math.cos(Math.toRadians(PITCH - ty));
    }

    @Test
    public void forwardDistanceMatchesClosedForm() {
        // 10.5 / tan(20 deg) = 28.8485
        assertEquals(28.8485, VisionMath.forwardDistanceInches(0, PITCH, DH), 1e-3);
        for (double ty = -10; ty <= 15; ty += 2.5) {
            assertEquals(expectedForward(ty), VisionMath.forwardDistanceInches(ty, PITCH, DH), LOOSE);
        }
    }

    @Test
    public void forwardDistanceRejectsHorizonAndAbove() {
        // ty == pitch puts the ray exactly level: it never reaches the floor.
        assertTrue(Double.isNaN(VisionMath.forwardDistanceInches(PITCH, PITCH, DH)));
        assertTrue(Double.isNaN(VisionMath.forwardDistanceInches(PITCH + 5, PITCH, DH)));
    }

    @Test
    public void forwardDistanceGrowsAsTargetApproachesHorizon() {
        double near = VisionMath.forwardDistanceInches(0, PITCH, DH);
        double far = VisionMath.forwardDistanceInches(15, PITCH, DH);
        assertTrue("closer to the horizon must read further away", far > near);
    }

    @Test
    public void rayIntersectionAgreesWithClosedForm() {
        for (double tx = -40; tx <= 40; tx += 5) {
            for (double ty = -10; ty <= 12; ty += 2) {
                double[] rf = VisionMath.targetInRobotFrame(
                        tx, ty, plainMount(), TARGET_H, MAX_RANGE);
                assertNotNull("expected a solution at tx=" + tx + " ty=" + ty, rf);
                assertEquals("forward at tx=" + tx + " ty=" + ty,
                        expectedForward(ty), rf[0], LOOSE);
                assertEquals("lateral at tx=" + tx + " ty=" + ty,
                        expectedLateral(tx, ty), rf[1], LOOSE);
            }
        }
    }

    /**
     * Locks in the numbers that motivated this rewrite. The old code split the forward distance
     * with cos(tx)/sin(tx) as though it were a radial range, which is exact on-axis and increasingly
     * short off it.
     */
    @Test
    public void correctsTheOldOffAxisUnderestimate() {
        final double D = 28.8485;   // 10.5 / tan(20 deg) — the forward distance at ty = 0
        double[][] cases = {
                //  tx     old forward                          old lateral
                {  0,   D * 1.0,                              -D * 0.0                              },
                { 10,   D * Math.cos(Math.toRadians(10)),     -D * Math.sin(Math.toRadians(10))     },
                { 30,   D * Math.cos(Math.toRadians(30)),     -D * Math.sin(Math.toRadians(30))     },
                { 45,   D * Math.cos(Math.toRadians(45)),     -D * Math.sin(Math.toRadians(45))     },
        };
        double[] expectedRangeErrorPct = {0.0, 1.7, 14.8, 31.6};

        for (int i = 0; i < cases.length; i++) {
            double tx = cases[i][0];
            double[] correct = VisionMath.targetInRobotFrame(tx, 0, plainMount(), TARGET_H, MAX_RANGE);
            assertNotNull(correct);

            // The corrected forward component does not depend on tx at all.
            assertEquals(D, correct[0], 1e-2);

            double oldRange = Math.hypot(cases[i][1], cases[i][2]);
            double newRange = Math.hypot(correct[0], correct[1]);
            double errorPct = 100 * (oldRange - newRange) / newRange;
            assertEquals("range error at tx=" + tx, expectedRangeErrorPct[i], Math.abs(errorPct), 0.2);
        }
    }

    @Test
    public void positiveTxPutsTargetToTheRight() {
        // Limelight tx > 0 means right of the crosshair, and +Y is left, so lateral must be negative.
        double[] right = VisionMath.targetInRobotFrame(20, 0, plainMount(), TARGET_H, MAX_RANGE);
        double[] left = VisionMath.targetInRobotFrame(-20, 0, plainMount(), TARGET_H, MAX_RANGE);
        assertTrue(right[1] < 0);
        assertTrue(left[1] > 0);
        assertEquals(-right[1], left[1], LOOSE);
    }

    @Test
    public void mountOffsetsShiftTheResult() {
        VisionMath.Mount offset = new VisionMath.Mount(CAM_H, PITCH, 6, 2, 0);
        double[] plain = VisionMath.targetInRobotFrame(0, 0, plainMount(), TARGET_H, MAX_RANGE);
        double[] shifted = VisionMath.targetInRobotFrame(0, 0, offset, TARGET_H, MAX_RANGE);
        assertEquals(plain[0] + 6, shifted[0], LOOSE);
        assertEquals(plain[1] + 2, shifted[1], LOOSE);
    }

    @Test
    public void yawedMountRotatesTheBearing() {
        // A camera yawed 15 degrees left sees an on-axis target 15 degrees left of robot forward.
        VisionMath.Mount yawed = new VisionMath.Mount(CAM_H, PITCH, 0, 0, 15);
        double[] rf = VisionMath.targetInRobotFrame(0, 0, yawed, TARGET_H, MAX_RANGE);
        assertNotNull(rf);
        double bearing = Math.toDegrees(Math.atan2(rf[1], rf[0]));
        assertEquals(15, bearing, 1e-3);
        // Rotation preserves range.
        assertEquals(expectedForward(0), Math.hypot(rf[0], rf[1]), LOOSE);
    }

    @Test
    public void rejectsTargetsAboveTheHorizon() {
        assertNull(VisionMath.targetInRobotFrame(0, PITCH, plainMount(), TARGET_H, MAX_RANGE));
        assertNull(VisionMath.targetInRobotFrame(0, PITCH + 10, plainMount(), TARGET_H, MAX_RANGE));
    }

    @Test
    public void rejectsTargetsBeyondMaxRange() {
        assertNull(VisionMath.targetInRobotFrame(0, 0, plainMount(), TARGET_H, 10));
        assertNotNull(VisionMath.targetInRobotFrame(0, 0, plainMount(), TARGET_H, 100));
    }

    @Test
    public void rejectsCameraBelowTarget() {
        VisionMath.Mount low = new VisionMath.Mount(1.0, PITCH, 0, 0, 0);
        assertNull(VisionMath.targetInRobotFrame(0, 0, low, TARGET_H, MAX_RANGE));
    }

    @Test
    public void standoffPullsTargetTowardTheRobot() {
        double[] target = {30, 40};   // range 50
        double[] approach = VisionMath.applyStandoff(target, 10);
        assertEquals(40, Math.hypot(approach[0], approach[1]), LOOSE);
        // Direction is unchanged.
        assertEquals(target[1] / target[0], approach[1] / approach[0], LOOSE);
    }

    @Test
    public void standoffLargerThanRangeClampsToZeroRatherThanFlipping() {
        double[] approach = VisionMath.applyStandoff(new double[] {3, 4}, 100);
        assertEquals(0, Math.hypot(approach[0], approach[1]), LOOSE);
    }

    @Test
    public void fieldFrameTransformRotatesAndTranslates() {
        // Robot at (10,20) facing +Y (90 degrees): "forward" maps to +Y, "left" maps to -X.
        double[] field = VisionMath.toFieldFrame(10, 20, Math.PI / 2, 5, 3);
        assertEquals(10 - 3, field[0], LOOSE);
        assertEquals(20 + 5, field[1], LOOSE);
    }

    @Test
    public void fieldFrameIsIdentityAtZeroHeading() {
        double[] field = VisionMath.toFieldFrame(10, 20, 0, 5, 3);
        assertEquals(15, field[0], LOOSE);
        assertEquals(23, field[1], LOOSE);
    }

}
