package org.firstinspires.ftc.teamcode.subsystems;

import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertFalse;
import static org.junit.Assert.assertNull;
import static org.junit.Assert.assertTrue;

import com.pedropathing.api.Paths;
import com.pedropathing.follower.Follower;
import com.pedropathing.ivy.Command;
import com.pedropathing.ivy.Scheduler;
import com.pedropathing.math.Pose;
import com.pedropathing.paths.Path;

import org.firstinspires.ftc.teamcode.util.hardware.Hardware;
import org.firstinspires.ftc.teamcode.util.time.FakeClock;
import org.junit.After;
import org.junit.Before;
import org.junit.Test;

/**
 * Drivetrain commands, heading hold, and the hold-at-end semantics, against a fake follower.
 *
 * <p>The fake mirrors Pedro 3.0.0's state machine ({@code atParametricEnd} true whenever not
 * following, mode implied by the last command, {@code follow} restarts), because every one of
 * these behaviours is where a Pedro upgrade could silently change what the robot does.
 */
public class DrivetrainCommandTest {
    private static final double EPS = 1e-9;

    private FakePathFollower fake;
    private FakeClock clock;
    private Drivetrain drivetrain;
    private double turnStick = 0;

    @Before
    public void setUp() {
        Scheduler.reset();
        Hardware.reset();
        fake = new FakePathFollower();
        clock = new FakeClock();
        drivetrain = new Drivetrain(fake, clock);
        Drivetrain.HEADING_HOLD_ENABLED = true;
        Drivetrain.MIN_PATH_MS = 60;
        Drivetrain.TURN_TIMEOUT_MS = 2500;
        turnStick = 0;
    }

    @After
    public void tearDown() {
        Scheduler.reset();
    }

    private static Path somePath() {
        return Paths.line(Pose.zero(), new Pose(12, 24)).constant(0);
    }

    private Command driverControl() {
        return drivetrain.driverControlCommand(() -> 0, () -> 0, () -> turnStick);
    }

    /** One robot loop: decide, then act. */
    private void tick() {
        Scheduler.execute();
        drivetrain.update();
        clock.advance(20);
    }

    // ---- Availability ----

    @Test
    public void unavailableDrivetrainCommandsFinishAtOnce() {
        Drivetrain none = new Drivetrain((PathFollower) null, clock);
        assertFalse(none.isAvailable());
        assertNull(none.getPose());
        Command follow = none.followLazyCommand(DrivetrainCommandTest::somePath, true);
        follow.schedule();
        Scheduler.execute();
        assertFalse(Scheduler.isScheduled(follow));
        Command turn = none.turnToCommand(1.0);
        turn.schedule();
        Scheduler.execute();
        assertFalse(Scheduler.isScheduled(turn));
    }

    @Test
    public void rawFollowerIsNullUnlessBuiltOnPedro() {
        assertNull(drivetrain.getFollower());
        assertTrue(drivetrain.isAvailable());
    }

    // ---- Manual drive ----

    @Test
    public void driverOffsetMakesBlueForwardFieldMinusX() {
        // The blue wall is +X, so a blue driver faces -X: stick up must drive -X.
        drivetrain.setFieldCentric(true);
        drivetrain.setDriverHeadingOffset(Math.PI);
        fake.pose = new Pose(0, 0, Math.PI);          // robot facing away from the blue driver
        drivetrain.drive(1, 0, 0);
        assertEquals("facing away from the driver, stick up is robot forward", 1, fake.lastForward, 1e-9);
        assertEquals(0, fake.lastStrafe, 1e-9);

        fake.pose = new Pose(0, 0, 0);                // robot facing the blue driver
        drivetrain.drive(1, 0, 0);
        assertEquals("facing the driver, stick up is robot backward", -1, fake.lastForward, 1e-9);

        drivetrain.setDriverHeadingOffset(0);         // red: the -X wall
        drivetrain.drive(1, 0, 0);
        assertEquals(1, fake.lastForward, 1e-9);
    }

    @Test
    public void resetHeadingUsesTheDriverForwardHeading() {
        drivetrain.setDriverHeadingOffset(Math.PI);
        fake.pose = new Pose(12, 34, 1.0);
        drivetrain.resetHeading();
        assertEquals(12, fake.pose.x(), EPS);
        assertEquals(34, fake.pose.y(), EPS);
        assertEquals("facing away from the blue wall is heading pi", Math.PI, fake.pose.heading(), 1e-9);
        assertFalse(drivetrain.isHeadingHoldActive());
    }

    @Test
    public void poseWrittenDuringCalibrationIsRewrittenOnceSettled() {
        Drivetrain.LOCALIZER_SETTLE_MS = 1000;
        Pose start = new Pose(48, 8.75, Math.toRadians(270));
        clock.advance(100);                           // init(): the Pinpoint is still calibrating
        drivetrain.setPose(start);
        assertEquals(1, fake.setPoseCalls);
        assertTrue(drivetrain.isPoseReapplyPending());
        drivetrain.onStart();
        assertEquals(Follower.Mode.MANUAL, fake.mode);
        tick();                                       // 120 ms: too early
        assertEquals(1, fake.setPoseCalls);
        clock.advance(1000);
        fake.pose = new Pose(50, 9, 4.0);             // whatever calibration left behind
        tick();
        assertEquals("written once more after the settle time", 2, fake.setPoseCalls);
        assertEquals(start.heading(), fake.pose.heading(), 1e-9);
        assertFalse(drivetrain.isPoseReapplyPending());
        tick();
        tick();
        assertEquals("and never again", 2, fake.setPoseCalls);

        drivetrain.setPose(new Pose(1, 2, 3));        // a write after settling is not repeated
        assertFalse(drivetrain.isPoseReapplyPending());
        tick();
        assertEquals(3, fake.setPoseCalls);
    }

    @Test
    public void driveForMsIssuesManualPowersThenHandsBack() {
        Command cmd = drivetrain.driveForMsCommand(-0.3, 0.1, 0, 100);
        cmd.schedule();
        tick();
        assertEquals(Follower.Mode.MANUAL, fake.mode);
        assertEquals(-0.3, fake.lastForward, EPS);
        assertEquals(0.1, fake.lastStrafe, EPS);
        tick();
        assertTrue("re-issued every loop", fake.manualCalls >= 2);
        clock.advance(100);
        tick();
        assertFalse(Scheduler.isScheduled(cmd));
        assertEquals("handed back with zero power", 0, fake.lastForward, EPS);
        assertEquals(0, fake.lastStrafe, EPS);
        assertEquals(Follower.Mode.MANUAL, fake.mode);

        Command none = new Drivetrain((PathFollower) null, clock).driveForMsCommand(1, 0, 0, 100);
        none.schedule();
        Scheduler.execute();
        assertFalse("no follower: finishes at once", Scheduler.isScheduled(none));
    }

    @Test
    public void fieldCentricRotatesStickInputIntoTheRobotFrame() {
        fake.pose = new Pose(0, 0, Math.PI / 2);      // facing field +Y
        drivetrain.setFieldCentric(true);
        drivetrain.drive(1, 0, 0);                    // driver pushes field +X
        assertEquals(Follower.Mode.MANUAL, fake.mode);
        assertEquals(0, fake.lastForward, 1e-9);
        assertEquals("field +X is robot-right, i.e. negative strafe", -1, fake.lastStrafe, 1e-9);

        drivetrain.setFieldCentric(false);
        drivetrain.drive(1, 0, 0);
        assertEquals(1, fake.lastForward, EPS);
        assertEquals(0, fake.lastStrafe, EPS);
    }

    @Test
    public void driverControlDoesNotWriteOverAnActivePath() {
        driverControl().schedule();
        tick();
        assertEquals(1, fake.manualCalls);

        fake.follow(somePath());                      // something else started a path
        tick();
        assertEquals("a manual write would abandon the path", 1, fake.manualCalls);
        assertEquals(Follower.Mode.FOLLOW, fake.mode);
    }

    // ---- Lazy path command ----

    @Test
    public void nullSupplierFinishesImmediatelyWithoutTouchingTheFollower() {
        Command cmd = drivetrain.followLazyCommand(() -> null, true);
        cmd.schedule();
        Scheduler.execute();
        assertFalse(Scheduler.isScheduled(cmd));
        assertEquals(0, fake.followCalls);
        assertEquals("no path started, so nothing to cancel", 0, fake.manualCalls);
    }

    @Test
    public void aPathWithoutAHeadingIsRejectedRatherThanThrown() {
        Command cmd = drivetrain.followLazyCommand(
                () -> Paths.line(Pose.zero(), new Pose(12, 24)), false);   // no .constant(...)
        cmd.schedule();
        Scheduler.execute();
        assertFalse(Scheduler.isScheduled(cmd));
        assertEquals(0, fake.followCalls);
        assertFalse("the failure is recorded for SelfTest", Hardware.getMissing().isEmpty());
    }

    @Test
    public void pathCannotFinishBeforeMinPathMs() {
        Command cmd = drivetrain.followLazyCommand(DrivetrainCommandTest::somePath, false);
        cmd.schedule();
        assertEquals(1, fake.followCalls);
        fake.finishPath();                          // follower says done on the very first tick

        Scheduler.execute();
        assertTrue("must not believe a zero-time path", Scheduler.isScheduled(cmd));

        clock.advance(Drivetrain.MIN_PATH_MS);
        Scheduler.execute();
        assertFalse(Scheduler.isScheduled(cmd));
    }

    @Test
    public void naturalEndWithHoldEndLeavesTheFollowerHoldingAtThePathEnd() {
        Command cmd = drivetrain.followLazyCommand(DrivetrainCommandTest::somePath, true);
        cmd.schedule();
        fake.finishPath();
        clock.advance(Drivetrain.MIN_PATH_MS);
        Scheduler.execute();

        assertFalse(Scheduler.isScheduled(cmd));
        assertEquals("station-keeping", Follower.Mode.HOLD, fake.mode);
        assertEquals(12, fake.lastHoldPose.x(), EPS);
        assertEquals(24, fake.lastHoldPose.y(), EPS);
        assertEquals("cancelPath must not have run", 0, fake.manualCalls);
    }

    @Test
    public void naturalEndWithoutHoldEndReturnsControlToTheDriver() {
        Command cmd = drivetrain.followLazyCommand(DrivetrainCommandTest::somePath, false);
        cmd.schedule();
        fake.finishPath();
        clock.advance(Drivetrain.MIN_PATH_MS);
        Scheduler.execute();

        assertFalse(Scheduler.isScheduled(cmd));
        assertEquals(Follower.Mode.MANUAL, fake.mode);
        assertEquals(1, fake.manualCalls);
        assertEquals(0, fake.holdCalls);
    }

    @Test
    public void interruptionAlwaysReturnsControlEvenWhenAskedToHold() {
        Command cmd = drivetrain.followLazyCommand(DrivetrainCommandTest::somePath, true);
        cmd.schedule();
        assertEquals(Follower.Mode.FOLLOW, fake.mode);
        Scheduler.cancel(cmd);
        assertEquals("the follower would otherwise keep driving itself",
                Follower.Mode.MANUAL, fake.mode);
        assertEquals(0, fake.holdCalls);
    }

    @Test
    public void driverControlResumingReleasesAHeldPose() {
        driverControl().schedule();
        tick();

        Command cmd = drivetrain.followLazyCommand(DrivetrainCommandTest::somePath, true);
        cmd.schedule();                             // suspends driver control
        fake.finishPath();
        clock.advance(Drivetrain.MIN_PATH_MS);
        tick();                                     // path ends holding; driver control re-added
        assertEquals(Follower.Mode.HOLD, fake.mode);

        tick();                                     // driver control executes again
        assertEquals("the driver taking over releases the hold", Follower.Mode.MANUAL, fake.mode);
    }

    @Test
    public void followPathCommandHasTheSameGuardsAsTheLazyOne() {
        Command cmd = drivetrain.followPathCommand(somePath(), false);
        cmd.schedule();
        assertEquals(1, fake.followCalls);
        fake.finishPath();
        Scheduler.execute();
        assertTrue(Scheduler.isScheduled(cmd));
        clock.advance(Drivetrain.MIN_PATH_MS);
        Scheduler.execute();
        assertFalse(Scheduler.isScheduled(cmd));
        assertEquals(Follower.Mode.MANUAL, fake.mode);
    }

    @Test
    public void aRebuiltCommandRunsAgainFromScratch() {
        Command cmd = drivetrain.followLazyCommand(DrivetrainCommandTest::somePath, false);
        cmd.schedule();
        fake.finishPath();
        clock.advance(Drivetrain.MIN_PATH_MS);
        Scheduler.execute();
        assertFalse(Scheduler.isScheduled(cmd));

        cmd.schedule();                             // same object, second run
        assertEquals(2, fake.followCalls);
        Scheduler.execute();
        assertTrue("MIN_PATH_MS restarts per run", Scheduler.isScheduled(cmd));
    }

    // ---- Turn and hold ----

    @Test
    public void turnToCommandChangesNothingUntilItStarts() {
        Command turn = drivetrain.turnToCommand(1.0);
        assertEquals("building a command must not touch the follower", 0, fake.holdCalls);

        turn.schedule();
        assertEquals(1, fake.holdCalls);
        assertEquals(1.0, fake.lastHoldPose.heading(), EPS);
        assertEquals(Follower.Mode.HOLD, fake.mode);

        fake.finishTurn();
        Scheduler.execute();
        assertTrue("MIN_PATH_MS applies to turns too", Scheduler.isScheduled(turn));
        clock.advance(Drivetrain.MIN_PATH_MS);
        Scheduler.execute();
        assertFalse(Scheduler.isScheduled(turn));
        assertEquals("a snap hands the sticks back", Follower.Mode.MANUAL, fake.mode);
    }

    @Test
    public void turnThatNeverArrivesGivesUpAfterTheTimeout() {
        Command turn = drivetrain.turnToCommand(2.0);
        turn.schedule();
        clock.advance(Drivetrain.TURN_TIMEOUT_MS - 1);
        Scheduler.execute();
        assertTrue(Scheduler.isScheduled(turn));
        clock.advance(1);
        Scheduler.execute();
        assertFalse(Scheduler.isScheduled(turn));
        assertEquals(Follower.Mode.MANUAL, fake.mode);
    }

    @Test
    public void holdCommandHoldsUntilInterrupted() {
        Command hold = drivetrain.holdCommand();
        hold.schedule();
        assertEquals(Follower.Mode.HOLD, fake.mode);
        for (int i = 0; i < 100; i++) tick();
        assertTrue("hold must not finish on its own", Scheduler.isScheduled(hold));

        Scheduler.cancel(hold);
        assertEquals(Follower.Mode.MANUAL, fake.mode);
    }

    // ---- Heading hold ----

    @Test
    public void headingHoldCorrectsDriftAndYieldsToTheDriver() {
        fake.pose = new Pose(0, 0, 1.0);
        driverControl().schedule();
        tick();                                     // captures 1.0
        assertTrue(drivetrain.isHeadingHoldActive());
        assertEquals(1.0, drivetrain.getHeldHeading(), EPS);

        fake.pose = new Pose(0, 0, 1.2);            // knocked counter-clockwise by 0.2 rad
        tick();
        assertTrue("must correct back toward the held heading", fake.lastTurn < 0);
        assertTrue(Math.abs(fake.lastTurn) <= Drivetrain.HEADING_HOLD_MAX_TURN + EPS);

        turnStick = 0.5;                            // driver steers
        tick();
        assertEquals(0.5, fake.lastTurn, EPS);
        assertFalse("deliberate turn releases the hold", drivetrain.isHeadingHoldActive());
    }

    @Test
    public void resetHeadingReleasesTheHoldInsteadOfSpinningTheRobot() {
        fake.pose = new Pose(0, 0, 1.5);
        driverControl().schedule();
        tick();
        tick();
        assertTrue(drivetrain.isHeadingHoldActive());

        drivetrain.resetHeading();
        assertFalse(drivetrain.isHeadingHoldActive());
        assertEquals(0, fake.pose.heading(), EPS);

        tick();                                     // re-captures at 0
        tick();
        assertEquals("no correction toward the discarded heading", 0, fake.lastTurn, EPS);
    }

    @Test
    public void setPoseReleasesTheHold() {
        fake.pose = new Pose(0, 0, 1.5);
        driverControl().schedule();
        tick();
        assertTrue(drivetrain.isHeadingHoldActive());
        drivetrain.setPose(new Pose(5, 5, 0.2));
        assertFalse(drivetrain.isHeadingHoldActive());
        assertEquals(1, fake.setPoseCalls);
    }

    @Test
    public void suspendingDriverControlReleasesTheHoldAndZeroesTheSticks() {
        fake.pose = new Pose(0, 0, 1.5);
        driverControl().schedule();
        tick();
        assertTrue(drivetrain.isHeadingHoldActive());

        drivetrain.holdCommand().schedule();       // preempts driver control
        assertFalse(drivetrain.isHeadingHoldActive());
        assertEquals(0, fake.lastForward, EPS);
        assertEquals(0, fake.lastTurn, EPS);
        assertEquals(Follower.Mode.HOLD, fake.mode);
    }

    // ---- Aim lock (fixed shooter: the hold's setpoint comes from outside) ----

    @Test
    public void aimLockSteersTowardTheSuppliedHeadingWhenTheStickIsCentred() {
        fake.pose = Pose.zero();
        driverControl().schedule();
        drivetrain.setAimLock(() -> Math.PI / 2);
        tick();
        tick();
        assertTrue(drivetrain.isAimLocked());
        assertTrue(drivetrain.isHeadingHoldActive());
        assertEquals(Math.PI / 2, drivetrain.getHeldHeading(), EPS);
        assertTrue("counter-clockwise toward 90 deg", fake.lastTurn > 0);
        assertTrue(fake.lastTurn <= Drivetrain.HEADING_HOLD_MAX_TURN + EPS);
    }

    @Test
    public void aStickTurnPassesThroughUnderAimLockAndTheLockResumesOnRelease() {
        fake.pose = Pose.zero();
        driverControl().schedule();
        drivetrain.setAimLock(() -> Math.PI / 2);
        tick();
        turnStick = -0.5;
        tick();
        assertEquals("never fight the driver", -0.5, fake.lastTurn, EPS);
        turnStick = 0;
        tick();
        tick();
        assertTrue("the lock resumes", fake.lastTurn > 0);
        assertTrue(drivetrain.isAimLocked());
    }

    @Test
    public void clearingTheLockReturnsToCaptureOnRelease() {
        fake.pose = Pose.zero();
        driverControl().schedule();
        drivetrain.setAimLock(() -> Math.PI / 2);
        tick();
        drivetrain.clearAimLock();
        assertFalse(drivetrain.isAimLocked());
        tick();                                     // re-captures the current heading, 0
        assertEquals(0, drivetrain.getHeldHeading(), EPS);
        tick();
        assertEquals("holding where it is, not the old lock", 0, fake.lastTurn, EPS);
    }

    @Test
    public void aNaNSupplierFallsBackToTheNormalHold() {
        fake.pose = new Pose(0, 0, 1.0);
        driverControl().schedule();
        drivetrain.setAimLock(() -> Double.NaN);
        tick();                                     // captures 1.0 like an ordinary hold
        assertEquals(1.0, drivetrain.getHeldHeading(), EPS);
        fake.pose = new Pose(0, 0, 1.2);
        tick();
        assertTrue(fake.lastTurn < 0);
    }

    @Test
    public void holdHeadingIsAPlainPedroHold() {
        fake.pose = new Pose(3, 4, 0);
        drivetrain.holdHeading(Math.PI);
        assertEquals(1, fake.holdCalls);
        assertEquals(3, fake.lastHoldPose.x(), EPS);
        assertEquals(Math.PI, fake.lastHoldPose.heading(), EPS);
        assertEquals(Follower.Mode.HOLD, fake.mode);
    }
}
