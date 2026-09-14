package org.firstinspires.ftc.teamcode.util.field;

import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertTrue;

import com.pedropathing.math.Pose;

import org.junit.Test;

/**
 * Tests for odometry/vision fusion.
 *
 * <p>Time is passed in rather than read from a clock, so these run deterministically off-robot.
 * Each test follows the real contract: feed the follower's pose in, write the returned pose back.
 */
public class PoseFusionTest {

    /** Simulates the caller's write-back: the returned pose becomes the follower's new baseline. */
    private static class Sim {
        final PoseFusion fusion = new PoseFusion();
        Pose current;
        long t = 0;

        Sim(Pose start) {
            current = start;
            fusion.seed(start);
        }

        /** Advances one loop with a known odometry motion and an optional vision fix. */
        Pose step(double dx, double dy, Pose vision, long latencyMs) {
            t += 20;
            Pose moved = new Pose(current.x() + dx, current.y() + dy, current.heading());
            Pose fused = fusion.update(t, moved, vision, latencyMs);
            current = fused;
            return fused;
        }
    }

    @Test
    public void withoutVisionItIntegratesOdometryExactly() {
        Sim sim = new Sim(new Pose(20, 30, 0));
        for (int i = 0; i < 25; i++) {
            sim.step(1, 0.5, null, 0);
        }
        // 25 loops of (+1, +0.5) from (20, 30).
        assertEquals(45, sim.current.x(), 1e-6);
        assertEquals(42.5, sim.current.y(), 1e-6);
        assertEquals(PoseFusion.Result.ODOMETRY_ONLY, sim.fusion.getLastResult());
        assertEquals(0, sim.fusion.getAcceptedCount());
    }

    @Test
    public void headingAlwaysComesFromOdometry() {
        Sim sim = new Sim(new Pose(20, 30, 1.234));
        Pose out = sim.step(1, 0, new Pose(25, 35, 2.5), 0);
        assertEquals("heading must not be taken from vision", 1.234, out.heading(), 1e-9);
    }

    @Test
    public void aConsistentVisionFixPullsTheEstimateToward() {
        Sim sim = new Sim(new Pose(20, 30, 0));
        // Odometry says we are not moving, vision insists we are 4 inches further along.
        for (int i = 0; i < 40; i++) {
            sim.step(0, 0, new Pose(24, 30, 0), 0);
        }
        assertEquals(PoseFusion.Result.ACCEPTED, sim.fusion.getLastResult());
        assertTrue("should converge toward the vision fix, got " + sim.current.x(),
                sim.current.x() > 23.5);
        assertEquals(30, sim.current.y(), 0.5);
    }

    @Test
    public void correctionIsGradualNotATeleport() {
        Sim sim = new Sim(new Pose(20, 30, 0));
        Pose after = sim.step(0, 0, new Pose(30, 30, 0), 0);
        assertEquals(PoseFusion.Result.ACCEPTED, sim.fusion.getLastResult());
        assertTrue("must not jump straight to the measurement", after.x() < 29);
        assertTrue("must move toward it though", after.x() > 20);
    }

    @Test
    public void anImplausibleJumpIsRejected() {
        Sim sim = new Sim(new Pose(20, 30, 0));
        Pose after = sim.step(0, 0, new Pose(120, 130, 0), 0);
        assertEquals(PoseFusion.Result.REJECTED_JUMP, sim.fusion.getLastResult());
        assertEquals(1, sim.fusion.getRejectedCount());
        assertEquals("estimate must be untouched", 20, after.x(), 1e-6);
        assertEquals(30, after.y(), 1e-6);
    }

    @Test
    public void anOffFieldFixIsRejected() {
        Sim sim = new Sim(new Pose(5, 5, 0));
        Pose after = sim.step(0, 0, new Pose(-20, 5, 0), 0);
        assertEquals(PoseFusion.Result.REJECTED_OFF_FIELD, sim.fusion.getLastResult());
        assertEquals(5, after.x(), 1e-6);
    }

    @Test
    public void oneBadFrameDoesNotDerailAGoodStream() {
        Sim sim = new Sim(new Pose(20, 30, 0));
        for (int i = 0; i < 20; i++) sim.step(0, 0, new Pose(22, 30, 0), 0);
        double before = sim.current.x();

        sim.step(0, 0, new Pose(130, 130, 0), 0);   // garbage frame
        assertEquals(PoseFusion.Result.REJECTED_JUMP, sim.fusion.getLastResult());
        assertEquals("a rejected frame must not move the estimate at all",
                before, sim.current.x(), 1e-6);
    }

    /**
     * The robot moves +2 in per 20 ms loop. A fix that <em>correctly</em> describes where it was
     * 100 ms ago should, once compensated, agree with the present and barely move the estimate.
     */
    @Test
    public void latencyCompensationShiftsTheFixForward() {
        Sim sim = new Sim(new Pose(20, 30, 0));
        for (int i = 0; i < 10; i++) sim.step(2, 0, null, 0);

        // The next step lands at t = 220, so a 100 ms latency means capture happened at t = 120.
        long captureTime = sim.t + 20 - 100;
        double trueXAtCapture = sim.fusion.historicalPosition(captureTime)[0];
        double expectedNow = sim.current.x() + 2;

        sim.step(2, 0, new Pose(trueXAtCapture, 30, 0), 100);

        assertEquals(PoseFusion.Result.ACCEPTED, sim.fusion.getLastResult());
        assertEquals("a correctly back-dated fix should leave the present estimate alone",
                expectedNow, sim.current.x(), 0.05);
    }

    @Test
    public void withoutCompensationTheSameFixWouldPullBackwards() {
        Sim sim = new Sim(new Pose(20, 30, 0));
        for (int i = 0; i < 10; i++) sim.step(2, 0, null, 0);

        long captureTime = sim.t + 20 - 100;
        double trueXAtCapture = sim.fusion.historicalPosition(captureTime)[0];
        double expectedNow = sim.current.x() + 2;

        // Identical fix, but declared as current. It now looks like the robot is 4 inches behind
        // where odometry says, and drags the estimate back - which is the error compensation removes.
        sim.step(2, 0, new Pose(trueXAtCapture, 30, 0), 0);

        assertTrue("an uncompensated stale fix pulls the estimate backwards, got "
                        + sim.current.x(),
                sim.current.x() < expectedNow - 0.3);
    }

    @Test
    public void historyLookupFindsTheNearestSample() {
        Sim sim = new Sim(new Pose(0, 0, 0));
        for (int i = 0; i < 10; i++) sim.step(1, 0, null, 0);
        // Samples were recorded at t = 20,40,...,200 with x = 1..10.
        double[] at100 = sim.fusion.historicalPosition(100);
        assertEquals(5, at100[0], 1e-6);
        double[] at60 = sim.fusion.historicalPosition(60);
        assertEquals(3, at60[0], 1e-6);
    }

    @Test
    public void firstUpdateSeedsFromOdometry() {
        PoseFusion fusion = new PoseFusion();
        Pose out = fusion.update(0, new Pose(50, 60, 1.0), null, 0);
        assertEquals(50, out.x(), 1e-9);
        assertEquals(60, out.y(), 1e-9);
        assertTrue(fusion.isSeeded());
    }

    @Test
    public void nullOdometryIsHandled() {
        assertEquals(null, new PoseFusion().update(0, null, null, 0));
    }
}
