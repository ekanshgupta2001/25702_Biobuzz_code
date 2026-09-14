package org.firstinspires.ftc.teamcode.util.field;

import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertFalse;
import static org.junit.Assert.assertSame;
import static org.junit.Assert.assertTrue;

import com.pedropathing.math.Pose;

import org.junit.Test;

/**
 * Tests for alliance mirroring.
 *
 * <p>The coordinates in {@link FieldConstants} are placeholders, but the mirroring math is real and
 * is the part that silently breaks one alliance while the other looks fine — so it gets tested.
 */
public class FieldConstantsTest {
    private static final double EPS = 1e-9;

    @Test
    public void blueIsTheIdentity() {
        Pose p = new Pose(12, 60, Math.toRadians(30));
        assertSame(p, FieldConstants.forAlliance(p, Alliance.BLUE));
    }

    @Test
    public void mirrorAcrossXReflectsPositionAndHeading() {
        Pose p = new Pose(12, 60, 0);
        Pose m = FieldConstants.mirrorAcrossX(p);
        assertEquals(132, m.x(), EPS);          // 144 - 12
        assertEquals(60, m.y(), EPS);           // unchanged
        assertEquals(Math.PI, m.heading(), 1e-9); // facing +X becomes facing -X
    }

    @Test
    public void mirrorAcrossXIsItsOwnInverse() {
        Pose p = new Pose(30, 100, Math.toRadians(37));
        Pose back = FieldConstants.mirrorAcrossX(FieldConstants.mirrorAcrossX(p));
        assertEquals(p.x(), back.x(), 1e-9);
        assertEquals(p.y(), back.y(), 1e-9);
        assertEquals(p.heading(), back.heading(), 1e-9);
    }

    @Test
    public void mirrorAcrossYReflectsPositionAndHeading() {
        Pose p = new Pose(12, 60, Math.toRadians(90));
        Pose m = FieldConstants.mirrorAcrossY(p);
        assertEquals(12, m.x(), EPS);
        assertEquals(84, m.y(), EPS);           // 144 - 60
        assertEquals(Math.toRadians(270), m.heading(), 1e-9);  // +Y becomes -Y
    }

    @Test
    public void mirrorAcrossYIsItsOwnInverse() {
        Pose p = new Pose(30, 100, Math.toRadians(200));
        Pose back = FieldConstants.mirrorAcrossY(FieldConstants.mirrorAcrossY(p));
        assertEquals(p.x(), back.x(), 1e-9);
        assertEquals(p.y(), back.y(), 1e-9);
        assertEquals(p.heading(), back.heading(), 1e-9);
    }

    @Test
    public void mirroredHeadingIsAlwaysNormalized() {
        for (double deg = 0; deg < 360; deg += 15) {
            Pose p = new Pose(20, 20, Math.toRadians(deg));
            double hx = FieldConstants.mirrorAcrossX(p).heading();
            double hy = FieldConstants.mirrorAcrossY(p).heading();
            assertTrue("mirrorX out of range at " + deg, hx >= 0 && hx < 2 * Math.PI);
            assertTrue("mirrorY out of range at " + deg, hy >= 0 && hy < 2 * Math.PI);
        }
    }

    @Test
    public void mirroringPreservesDistanceFromTheCentreLine() {
        Pose p = new Pose(20, 60, 0);
        Pose m = FieldConstants.mirrorAcrossX(p);
        double before = Math.abs(p.x() - FieldConstants.FIELD_CENTER_INCHES);
        double after = Math.abs(m.x() - FieldConstants.FIELD_CENTER_INCHES);
        assertEquals(before, after, EPS);
    }

    @Test
    public void aPoseOnTheCentreLineIsUnmoved() {
        Pose p = new Pose(FieldConstants.FIELD_CENTER_INCHES, 40, Math.toRadians(90));
        Pose m = FieldConstants.mirrorAcrossX(p);
        assertEquals(p.x(), m.x(), EPS);
        // Facing straight up is unchanged by a left/right mirror.
        assertEquals(Math.toRadians(90), m.heading(), 1e-9);
    }

    @Test
    public void fieldBoundsCheck() {
        assertTrue(FieldConstants.isInsideField(0, 0));
        assertTrue(FieldConstants.isInsideField(144, 144));
        assertTrue(FieldConstants.isInsideField(72, 72));
        assertFalse(FieldConstants.isInsideField(-1, 72));
        assertFalse(FieldConstants.isInsideField(72, 145));
    }

    @Test
    public void nullIsPassedThroughRatherThanThrowing() {
        assertEquals(null, FieldConstants.forAlliance(null, Alliance.RED));
        assertEquals(null, FieldConstants.mirrorAcrossX(null));
        assertEquals(null, FieldConstants.mirrorAcrossY(null));
        assertFalse(FieldConstants.isInsideField(null));
    }

    @Test
    public void rotate180ReflectsBothCoordinatesAndTurnsTheHeadingAround() {
        Pose p = new Pose(12, 60, Math.toRadians(30));
        Pose r = FieldConstants.rotate180(p);
        assertEquals(132, r.x(), EPS);
        assertEquals(84, r.y(), EPS);
        assertEquals(Math.toRadians(210), r.heading(), 1e-9);
    }

    @Test
    public void rotate180IsItsOwnInverse() {
        Pose p = new Pose(30, 100, Math.toRadians(300));
        Pose back = FieldConstants.rotate180(FieldConstants.rotate180(p));
        assertEquals(p.x(), back.x(), 1e-9);
        assertEquals(p.y(), back.y(), 1e-9);
        assertEquals(p.heading(), back.heading(), 1e-9);
    }

    @Test
    public void forAllianceFollowsTheConfiguredSymmetry() {
        FieldConstants.Symmetry saved = FieldConstants.SYMMETRY;
        try {
            Pose p = new Pose(12, 60, 0);
            FieldConstants.SYMMETRY = FieldConstants.Symmetry.MIRROR_X;
            assertEquals(132, FieldConstants.forAlliance(p, Alliance.RED).x(), EPS);
            assertEquals(60, FieldConstants.forAlliance(p, Alliance.RED).y(), EPS);
            FieldConstants.SYMMETRY = FieldConstants.Symmetry.MIRROR_Y;
            assertEquals(12, FieldConstants.forAlliance(p, Alliance.RED).x(), EPS);
            assertEquals(84, FieldConstants.forAlliance(p, Alliance.RED).y(), EPS);
            FieldConstants.SYMMETRY = FieldConstants.Symmetry.ROTATE_180;
            assertEquals(132, FieldConstants.forAlliance(p, Alliance.RED).x(), EPS);
            assertEquals(84, FieldConstants.forAlliance(p, Alliance.RED).y(), EPS);
            assertEquals(Math.PI, FieldConstants.forAlliance(p, Alliance.RED).heading(), 1e-9);
        } finally {
            FieldConstants.SYMMETRY = saved;
        }
    }

    @Test
    public void rotatedHeadingIsAlwaysNormalized() {
        for (double deg = 0; deg < 360; deg += 15) {
            double h = FieldConstants.rotate180(new Pose(20, 20, Math.toRadians(deg))).heading();
            assertTrue("rotate180 out of range at " + deg, h >= 0 && h < 2 * Math.PI);
        }
    }
}
