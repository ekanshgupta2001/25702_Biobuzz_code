package org.firstinspires.ftc.teamcode.commands;

import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertFalse;
import static org.junit.Assert.assertNotNull;
import static org.junit.Assert.assertTrue;
import static org.junit.Assert.fail;

import com.pedropathing.follower.Follower;
import com.pedropathing.ivy.Command;
import com.pedropathing.ivy.Scheduler;
import com.pedropathing.math.Pose;

import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.subsystems.ColorSensor;
import org.firstinspires.ftc.teamcode.subsystems.Drivetrain;
import org.firstinspires.ftc.teamcode.subsystems.FakeDcMotorEx;
import org.firstinspires.ftc.teamcode.subsystems.FakePathFollower;
import org.firstinspires.ftc.teamcode.subsystems.Intake;
import org.firstinspires.ftc.teamcode.subsystems.Limelight;
import org.firstinspires.ftc.teamcode.subsystems.OpenLoopDrive;
import org.firstinspires.ftc.teamcode.subsystems.Shooter;
import org.firstinspires.ftc.teamcode.subsystems.Storage;
import org.firstinspires.ftc.teamcode.subsystems.Transfer;
import org.firstinspires.ftc.teamcode.subsystems.PathFollower;
import org.firstinspires.ftc.teamcode.util.hardware.Hardware;
import org.firstinspires.ftc.teamcode.util.time.FakeClock;
import org.junit.After;
import org.junit.Before;
import org.junit.Test;

/**
 * Macros end to end through Ivy's real Scheduler against a fake follower, fake motors, no camera
 * and no sensors unless a test wires one. Timeouts run on the injected clock, so every test is
 * deterministic: one tick is one robot loop plus 20 fake milliseconds. Failure cases first.
 */
public class MacrosTest {
    private static final double EPS = 1e-9;
    /** 60 s of fake time: no macro here may run longer. */
    private static final int MAX_TICKS = 3000;

    private FakeClock clock;
    private FakePathFollower follower;
    private FakeDcMotorEx intakeMotor;
    private FakeDcMotorEx storageMotor;
    private FakeDcMotorEx transferMotor;
    private FakeDcMotorEx shooterMotor;
    private Robot robot;

    @Before
    public void setUp() {
        Scheduler.reset();
        Hardware.reset();
        clock = new FakeClock();
        follower = new FakePathFollower();
        intakeMotor = new FakeDcMotorEx();
        storageMotor = new FakeDcMotorEx();
        transferMotor = new FakeDcMotorEx();
        shooterMotor = new FakeDcMotorEx();
        robot = buildRobot(new Drivetrain(follower, clock));
        resetTunables();
    }

    @After
    public void tearDown() {
        Scheduler.reset();
        Hardware.reset();
        resetTunables();
    }

    private Robot buildRobot(Drivetrain drivetrain) {
        return new Robot(
                drivetrain,
                new OpenLoopDrive((com.pedropathing.drivetrain.Drivetrain) null, clock),
                new Intake(intakeMotor, clock),
                new Storage(storageMotor, null, clock),
                new Transfer(transferMotor, clock),
                new Shooter(shooterMotor, null, clock),
                new Limelight(null),
                new ColorSensor(null), new ColorSensor(null),
                new ColorSensor(null), new ColorSensor(null),
                clock);
    }

    private static void resetTunables() {
        Macros.INTAKE_TIMEOUT_MS = 8000;
        Macros.INTAKE_RUNS_STORAGE = false;
        Macros.SHOOT_ONE_TIMEOUT_MS = 6000;
        Macros.SHOOT_ALL_TIMEOUT_MS = 20000;
        Macros.SENSORLESS_FEED_PULSE_MS = 600;
        Shooter.SHOT_RECOVERY_MIN_MS = 150;
        Shooter.SHOT_RECOVERY_TIMEOUT_MS = 1500;
        Macros.AIM_TIMEOUT_MS = 2500;
        Macros.AIM_TOLERANCE_DEGREES = 2.0;
        Macros.AIM_REISSUE_DEGREES = 1.0;
        Macros.ALIGN_TOLERANCE_DEGREES = 1.5;
        Shooter.HEADING_OFFSET_RAD = Math.PI;
        Macros.PIPELINE_WARMUP_MS = 250;
        Macros.SEARCH_TIMEOUT_MS = 2000;
        Macros.APPROACH_TIMEOUT_MS = 4000;
        Macros.ALIGN_TIMEOUT_MS = 1500;
        Macros.RELOCALIZE_TIMEOUT_MS = 1500;
        Macros.SNAP_TIMEOUT_MS = 1500;
        Macros.SNAP_TOLERANCE_DEGREES = 3.0;
        Macros.DRIVE_TO_TIMEOUT_MS = 6000;
        Macros.DRIVE_TO_TOLERANCE_INCHES = 3.0;
        Storage.CAPACITY = 4;
        Storage.ADVANCE_TIMEOUT_MS = 2500;
        Transfer.LIFT_TIMEOUT_MS = 2000;
        Transfer.LIFT_PULSE_MS = 600;
        Transfer.FEED_TIMEOUT_MS = 1000;
        Transfer.FEED_PULSE_MS = 300;
        Transfer.FEED_CLEAR_DWELL_MS = 100;
        Shooter.SPINUP_TIMEOUT_MS = 3000;
        Intake.ANTI_JAM_ENABLED = true;
        Drivetrain.MIN_PATH_MS = 60;
        Drivetrain.TURN_TIMEOUT_MS = 2500;
    }

    /** One robot loop in production order: observe, decide, act. */
    private void tick() {
        robot.readSensors();
        Scheduler.execute();
        robot.writeActuators();
        clock.advance(20);
    }

    private int runToCompletion(Command macro) {
        return runToCompletion(macro, null);
    }

    /** Schedules the macro and ticks until it finishes; {@code eachLoop} runs before every tick. */
    private int runToCompletion(Command macro, Runnable eachLoop) {
        int ticks = 0;
        macro.schedule();
        while (Scheduler.isScheduled(macro)) {
            if (eachLoop != null) eachLoop.run();
            tick();
            if (++ticks > MAX_TICKS) fail("macro did not finish: " + robot.macros.getStatus());
        }
        return ticks;
    }

    private void shooterAtSpeed() {
        shooterMotor.measuredVelocity = Shooter.rpmToTicksPerSec(Shooter.SHOOT_RPM);
    }

    // ---- Shooting ----

    @Test
    public void shootOneReportsNoTargetWhenTheEntranceSensorSaysEmpty() {
        robot.storage.setEntranceSupplier(() -> false);     // a sensor is fitted and sees nothing
        runToCompletion(robot.macros.shootOne());
        assertEquals(Macros.Outcome.NO_TARGET, robot.macros.getOutcome());
        assertEquals("idle", robot.macros.getActiveName());
        assertEquals("flywheel never spun", 0, shooterMotor.maxCommandedVelocity, EPS);
        assertEquals("transfer never ran", 0, transferMotor.maxCommandedVelocity, EPS);
        assertEquals(0, robot.macros.getShotsFired());
    }

    @Test
    public void shootOneCyclesATimedPulseRobotAndDeadReckonsTheQueue() {
        robot.storage.setCount(2);
        shooterAtSpeed();
        runToCompletion(robot.macros.shootOne());

        assertEquals(Macros.Outcome.SUCCESS, robot.macros.getOutcome());
        assertEquals(1, robot.macros.getShotsFired());
        assertEquals("no exit sensor: the count is dead-reckoned down", 1, robot.storage.count());
        assertEquals(Storage.ADVANCE_TICKS_PER_SEC, storageMotor.maxCommandedVelocity, EPS);
        assertEquals("the timed feed ran at feed speed", Transfer.FEED_TICKS_PER_SEC, transferMotor.maxCommandedVelocity, EPS);
        assertEquals("shooter held at speed during the feed",
                Shooter.rpmToTicksPerSec(Shooter.SHOOT_RPM), shooterMotor.maxCommandedVelocity, EPS);
        assertEquals("everything stopped afterwards", 0, shooterMotor.commandedVelocity, EPS);
        assertEquals(0, transferMotor.commandedVelocity, EPS);
        assertEquals(0, storageMotor.commandedVelocity, EPS);
    }

    @Test
    public void shootOneHoldsTheShooterStorageAndTransferButNeverTheDrivetrain() {
        Command macro = robot.macros.shootOne();
        assertTrue(macro.requirements().contains(robot.shooter));
        assertTrue(macro.requirements().contains(robot.storage));
        assertTrue(macro.requirements().contains(robot.transfer));
        assertFalse("a shot must not take the sticks away", macro.requirements().contains(robot.drivetrain));
    }

    @Test
    public void shootOneUsesTheSensorsWhenFitted() {
        final boolean[] inTransfer = {false};
        final boolean[] atFeed = {false};
        final int[] feedTicks = {0};
        robot.storage.setExitSupplier(() -> inTransfer[0]);
        robot.transfer.setInLiftSupplier(() -> inTransfer[0]);
        robot.transfer.setAtFeedSupplier(() -> atFeed[0]);
        robot.storage.setCount(2);
        shooterAtSpeed();

        runToCompletion(robot.macros.shootOne(), () -> {
            if (robot.storage.getMode() == Storage.Mode.ADVANCING) inTransfer[0] = true;   // piece reaches the lift
            if (robot.transfer.getMode() == Transfer.Mode.LIFTING) {                        // lift stages it
                atFeed[0] = true;
                inTransfer[0] = false;
            }
            if (robot.transfer.getMode() == Transfer.Mode.FEEDING && ++feedTicks[0] > 2) atFeed[0] = false;
        });

        assertEquals(Macros.Outcome.SUCCESS, robot.macros.getOutcome());
        assertEquals(1, robot.macros.getShotsFired());
        assertEquals("the exit sensor decremented the count, exactly once", 1, robot.storage.count());
        assertEquals(1, robot.storage.getExitEvents());
    }

    @Test
    public void shootOneTimesOutWhenTheLiftNeverStages() {
        final boolean[] inTransfer = {false};
        robot.storage.setExitSupplier(() -> inTransfer[0]);
        robot.transfer.setAtFeedSupplier(() -> false);      // a feed sensor that never sees a piece
        robot.storage.setCount(2);
        shooterAtSpeed();

        runToCompletion(robot.macros.shootOne(), () -> {
            if (robot.storage.getMode() == Storage.Mode.ADVANCING) inTransfer[0] = true;
        });

        assertEquals(Macros.Outcome.TIMED_OUT, robot.macros.getOutcome());
        assertEquals("nothing was fired", 0, robot.macros.getShotsFired());
        assertEquals("the sensor's own decrement stands; no dead-reckoning on top", 1, robot.storage.count());
    }

    @Test
    public void shootAllEmptiesTheQueueWithOneSpinUp() {
        robot.storage.setCount(3);
        shooterAtSpeed();
        final boolean[] spinning = {false};
        final boolean[] droppedMidRun = {false};
        runToCompletion(robot.macros.shootAll(), () -> {
            // The wheel idles once the last piece has gone; a drop before that is the bug.
            if (robot.shooter.getTargetRpm() > 0) spinning[0] = true;
            else if (spinning[0] && robot.macros.getShotsFired() < 3) droppedMidRun[0] = true;
        });

        assertEquals(Macros.Outcome.SUCCESS, robot.macros.getOutcome());
        assertEquals(3, robot.macros.getShotsFired());
        assertEquals(0, robot.storage.count());
        assertTrue("the flywheel spun up", spinning[0]);
        assertFalse("the flywheel never wound down between pieces", droppedMidRun[0]);
        assertEquals(0, shooterMotor.commandedVelocity, EPS);
    }

    @Test
    public void shootAllReportsNoTargetWhenTheEntranceSensorSaysEmpty() {
        robot.storage.setEntranceSupplier(() -> false);
        runToCompletion(robot.macros.shootAll());
        assertEquals(Macros.Outcome.NO_TARGET, robot.macros.getOutcome());
    }

    @Test
    public void shootOneFiresOnePulseWhenTheCountIsUnknowable() {
        // The first-event robot: no entrance sensor, so the count never rises in teleop. Refusing
        // to shoot would make the operator's button useless; it fires one metered pulse instead.
        assertFalse(robot.storage.hasEntranceSensor());
        assertFalse(robot.macros.isCountKnown());
        assertEquals(Storage.CAPACITY, robot.macros.piecesOnBoard());
        shooterAtSpeed();
        runToCompletion(robot.macros.shootOne());
        assertEquals(Macros.Outcome.SUCCESS, robot.macros.getOutcome());
        assertEquals(1, robot.macros.getShotsFired());
        assertEquals("the transfer fed at feed speed", Transfer.FEED_TICKS_PER_SEC, transferMotor.maxCommandedVelocity, EPS);
        assertEquals("the transport pulsed with it", Storage.ADVANCE_TICKS_PER_SEC, storageMotor.maxCommandedVelocity, EPS);
        assertEquals("count clamps at zero, still unknown", 0, robot.storage.count());
        assertEquals(0, transferMotor.commandedVelocity, EPS);
    }

    @Test
    public void shootAllEmptiesTheRobotWithFourPulsesWhenTheCountIsUnknowable() {
        shooterAtSpeed();
        runToCompletion(robot.macros.shootAll());
        assertEquals(Macros.Outcome.SUCCESS, robot.macros.getOutcome());
        assertEquals("one pulse per slot the robot could be holding", Storage.CAPACITY, robot.macros.getShotsFired());
    }

    @Test
    public void aKnownCountIsShotExactlyEvenWithoutAnEntranceSensor() {
        robot.storage.setCount(2);                            // the auto-to-teleop hand-off
        assertTrue(robot.macros.isCountKnown());
        shooterAtSpeed();
        runToCompletion(robot.macros.shootAll());
        assertEquals(2, robot.macros.getShotsFired());
        assertEquals(0, robot.storage.count());
    }

    @Test
    public void shootAllShootsAPieceWaitingInTheLiftToo() {
        // 4 in the queue + 1 already lifted = 5: one more feed step than the storage holds.
        robot.transfer.setInLiftSupplier(() -> true);
        robot.storage.setCount(4);
        shooterAtSpeed();
        runToCompletion(robot.macros.shootAll());
        assertEquals(5, robot.macros.getShotsFired());
        assertEquals("five on board, five fired", Macros.Outcome.SUCCESS, robot.macros.getOutcome());
    }

    @Test
    public void shootOneDoesNotWaitForRecoveryAfterItsOnlyShot() {
        // The wheel never reads at speed, so every wait runs to its timeout: spin-up 3 s, the pulse,
        // then only the 150 ms dwell. A 1.5 s recovery wait after the last piece would be dead time.
        robot.storage.setCount(2);
        int ticks = runToCompletion(robot.macros.shootOne());
        long ms = ticks * 20L;
        long withRecovery = Shooter.SPINUP_TIMEOUT_MS + Macros.SENSORLESS_FEED_PULSE_MS
                + Shooter.SHOT_RECOVERY_MIN_MS + Shooter.SHOT_RECOVERY_TIMEOUT_MS;
        assertTrue("finished in " + ms + " ms, must be well under " + withRecovery, ms < withRecovery - 1000);
        assertTrue("but not before the dwell", ms >= Shooter.SPINUP_TIMEOUT_MS
                + Macros.SENSORLESS_FEED_PULSE_MS + Shooter.SHOT_RECOVERY_MIN_MS);
        assertEquals(Macros.Outcome.SUCCESS, robot.macros.getOutcome());
    }

    @Test
    public void shootAllWaitsForRecoveryOnlyBetweenPieces() {
        robot.storage.setCount(2);                            // never at speed: every wait times out
        int ticks = runToCompletion(robot.macros.shootAll());
        long ms = ticks * 20L;
        long perPiece = Macros.SENSORLESS_FEED_PULSE_MS + Shooter.SHOT_RECOVERY_MIN_MS;
        long expected = Shooter.SPINUP_TIMEOUT_MS + 2 * perPiece + Shooter.SHOT_RECOVERY_TIMEOUT_MS;  // one recovery, between
        // Ivy hands off one child per tick, so a few dozen 20 ms ticks of overhead ride on top; a
        // second full recovery would add another SHOT_RECOVERY_TIMEOUT_MS, which is what is ruled out.
        assertTrue("took " + ms + " ms, expected about " + expected, ms >= expected && ms < expected + 1000);
        assertEquals(2, robot.macros.getShotsFired());
    }

    @Test
    public void markCancelledLeavesAFinishedMacroAlone() {
        robot.storage.setCount(1);
        shooterAtSpeed();
        runToCompletion(robot.macros.shootOne());
        assertEquals(Macros.Outcome.SUCCESS, robot.macros.getOutcome());
        robot.macros.markCancelled();                          // a late callback from an old group
        assertEquals("a terminal outcome is never changed", Macros.Outcome.SUCCESS, robot.macros.getOutcome());
        assertEquals("idle", robot.macros.getActiveName());
    }

    @Test
    public void sensorlessShootAllOfFourFinishesInsideThirteenSeconds() {
        // fixthese B2 / C9: four pre-loads on timed pulses used to cost ~16.6 s, most of it the storage
        // running to its 2.5 s timeout against a stopped transfer.
        robot.storage.setCount(4);
        shooterAtSpeed();
        final boolean[] pushedIntoStoppedTransfer = {false};
        int ticks = runToCompletion(robot.macros.shootAll(), () -> {
            if (robot.storage.getMode() == Storage.Mode.ADVANCING
                    && robot.transfer.getMode() != Transfer.Mode.FEEDING) {
                pushedIntoStoppedTransfer[0] = true;
            }
        });
        assertEquals(Macros.Outcome.SUCCESS, robot.macros.getOutcome());
        assertEquals(4, robot.macros.getShotsFired());
        assertEquals(0, robot.storage.count());
        assertTrue("four pieces in " + ticks * 20 + " ms", ticks * 20 < 13000);
        assertFalse("the storage never ran into a stopped transfer", pushedIntoStoppedTransfer[0]);
    }

    @Test
    public void feedSensorWithoutExitSensorStillAdvancesOnATimer() {
        // fixthese B2, the mixed case: a feed sensor but no transfer sensor. The advance has no edge
        // to end on, so it is the timed pulse, run together with the sensed lift.
        final boolean[] atFeed = {false};
        final int[] feedTicks = {0};
        final int[] advancingTicks = {0};
        robot.transfer.setAtFeedSupplier(() -> atFeed[0]);
        robot.storage.setCount(2);
        shooterAtSpeed();
        runToCompletion(robot.macros.shootOne(), () -> {
            if (robot.storage.getMode() == Storage.Mode.ADVANCING) advancingTicks[0]++;
            if (robot.transfer.getMode() == Transfer.Mode.LIFTING) atFeed[0] = true;       // the lift stages it
            if (robot.transfer.getMode() == Transfer.Mode.FEEDING && ++feedTicks[0] > 2) atFeed[0] = false;
        });
        assertEquals(Macros.Outcome.SUCCESS, robot.macros.getOutcome());
        assertEquals(1, robot.macros.getShotsFired());
        assertEquals("dead-reckoned down without an exit sensor", 1, robot.storage.count());
        assertTrue("the advance was the timed pulse, not a run to the 2.5 s timeout: " + advancingTicks[0] * 20 + " ms",
                advancingTicks[0] * 20 <= Macros.SENSORLESS_FEED_PULSE_MS + 60);
    }

    @Test
    public void boundedTimeoutLeavesATerminalOutcome() {
        // fixthese C6: a caller that bounds a macro used to leave its outcome RUNNING for good.
        robot.storage.setCount(2);                      // and the wheel never comes up to speed
        Command cut = Waits.bounded(clock, robot.macros.shootAll(), 200);
        runToCompletion(cut);
        assertEquals(Macros.Outcome.CANCELLED, robot.macros.getOutcome());
        assertEquals("idle", robot.macros.getActiveName());
        assertFalse(robot.macros.isRunning());
        assertEquals("the flywheel was let go", 0, shooterMotor.commandedVelocity, EPS);
    }

    // ---- Collecting ----

    @Test
    public void intakeUntilFullTimesOutAndStopsEverything() {
        int ticks = runToCompletion(robot.macros.intakeUntilFull());
        assertEquals(Macros.Outcome.TIMED_OUT, robot.macros.getOutcome());
        assertTrue("ran for the whole timeout", ticks >= Macros.INTAKE_TIMEOUT_MS / 20);
        assertEquals(Intake.INTAKE_TICKS_PER_SEC, intakeMotor.maxCommandedVelocity, EPS);
        assertEquals("the transport stays still by default (fixthese C8)", 0, storageMotor.maxCommandedVelocity, EPS);
        assertEquals(0, intakeMotor.commandedVelocity, EPS);
        assertEquals(0, storageMotor.commandedVelocity, EPS);
    }

    @Test
    public void intakeUntilFullRunsTheTransportWhenEnabled() {
        Macros.INTAKE_RUNS_STORAGE = true;
        runToCompletion(robot.macros.intakeUntilFull());
        assertEquals(Storage.ADVANCE_TICKS_PER_SEC, storageMotor.maxCommandedVelocity, EPS);
        assertEquals(0, storageMotor.commandedVelocity, EPS);
    }

    @Test
    public void intakeUntilFullSucceedsOnFourEntranceEdges() {
        final boolean[] entrance = {false};
        final int[] loops = {0};
        robot.storage.setEntranceSupplier(() -> entrance[0]);
        runToCompletion(robot.macros.intakeUntilFull(), () -> {
            if (robot.storage.count() < 4) entrance[0] = (++loops[0] % 4) < 2;   // a pulse every four loops
        });
        assertEquals(Macros.Outcome.SUCCESS, robot.macros.getOutcome());
        assertEquals(4, robot.storage.count());
        assertTrue(robot.storage.isFull());
        assertEquals(0, intakeMotor.commandedVelocity, EPS);
    }

    @Test
    public void intakeUntilFullReportsNoTargetWhenAlreadyFull() {
        robot.storage.setCount(4);
        runToCompletion(robot.macros.intakeUntilFull());
        assertEquals(Macros.Outcome.NO_TARGET, robot.macros.getOutcome());
        assertEquals(0, intakeMotor.maxCommandedVelocity, EPS);
    }

    @Test
    public void collectPieceTimesOutWhenNothingIsVisible() {
        Command macro = robot.macros.collectPiece();
        assertTrue(macro.requirements().contains(robot.drivetrain));
        assertTrue(macro.requirements().contains(robot.intake));

        runToCompletion(macro);
        assertEquals(Macros.Outcome.TIMED_OUT, robot.macros.getOutcome());
        assertEquals("no target, so no path", 0, follower.followCalls);
        assertEquals(0, intakeMotor.commandedVelocity, EPS);
        assertFalse(robot.intake.hasPiece());
    }

    @Test
    public void alignToPieceReportsNoTargetWithoutACamera() {
        runToCompletion(robot.macros.alignToPiece());
        assertEquals(Macros.Outcome.NO_TARGET, robot.macros.getOutcome());
        assertEquals("no blob, so no turn was started", 0, follower.holdCalls);
    }

    // ---- Aiming (fixed shooter: the drivetrain turns) ----

    @Test
    public void aimHeadingPointsTheRearAtTheTarget() {
        follower.pose = Pose.zero();
        assertEquals("bearing 45 deg, shooter fires out the rear: face 225",
                Math.toRadians(225), robot.macros.aimHeading(new Pose(72, 72), 30, 45), 1e-9);
        Shooter.HEADING_OFFSET_RAD = 0;
        assertEquals("a forward-firing shooter would face the target",
                Math.toRadians(45), robot.macros.aimHeading(new Pose(72, 72), 30, 45), 1e-9);
    }

    @Test
    public void aimHeadingIsNaNWithoutAPoseAndAimAtThenTimesOut() {
        robot = buildRobot(new Drivetrain((PathFollower) null, clock));
        assertTrue(Double.isNaN(robot.macros.aimHeading(new Pose(72, 72), 30, 45)));
        runToCompletion(robot.macros.aimAt(new Pose(72, 72), 30, 45));
        assertEquals(Macros.Outcome.TIMED_OUT, robot.macros.getOutcome());
    }

    @Test
    public void aimAtHoldsTheShooterSideTowardTheTargetAndHandsBack() {
        follower.pose = Pose.zero();
        runToCompletion(robot.macros.aimAt(new Pose(72, 72), 30, 45), follower::finishTurn);
        assertEquals(Macros.Outcome.SUCCESS, robot.macros.getOutcome());
        assertEquals("one Pedro hold: the odometry target never moved", 1, follower.holdCalls);
        assertEquals(Math.toRadians(225), follower.lastHoldPose.heading(), 1e-9);
        assertEquals(Math.toRadians(225), follower.pose.heading(), 1e-9);
        assertEquals("sticks handed back", Follower.Mode.MANUAL, follower.mode);
    }

    @Test
    public void aimAtTimesOutWhenTheRobotNeverTurns() {
        follower.pose = Pose.zero();
        int ticks = runToCompletion(robot.macros.aimAt(new Pose(72, 72), 30, 45));
        assertEquals(Macros.Outcome.TIMED_OUT, robot.macros.getOutcome());
        assertTrue(ticks >= Macros.AIM_TIMEOUT_MS / 20);
        assertEquals(Follower.Mode.MANUAL, follower.mode);
    }

    @Test
    public void aimAndShootAllGivesUpAimingAfterTheTimeoutAndStillShoots() {
        follower.pose = Pose.zero();                          // and the robot never turns
        robot.storage.setCount(2);
        shooterAtSpeed();
        int ticks = runToCompletion(robot.macros.aimAndShootAll(new Pose(72, 72), 30, 45));
        long ms = ticks * 20L;
        assertTrue("the aim is bounded: " + ms + " ms", ms < Macros.AIM_TIMEOUT_MS + 2 * Macros.SENSORLESS_FEED_PULSE_MS + 3000);
        assertEquals("a held piece scores nothing: it shot anyway", 2, robot.macros.getShotsFired());
        assertEquals(Macros.Outcome.SUCCESS, robot.macros.getOutcome());
        assertEquals("the hold was released", Follower.Mode.MANUAL, follower.mode);
    }

    @Test
    public void aimAndShootAllTurnsThenEmptiesTheQueue() {
        follower.pose = Pose.zero();
        robot.storage.setCount(2);
        shooterAtSpeed();
        Command macro = robot.macros.aimAndShootAll(new Pose(72, 72), 30, 45);
        assertTrue("autonomous's move owns the drivetrain", macro.requirements().contains(robot.drivetrain));
        assertTrue(macro.requirements().contains(robot.shooter));
        runToCompletion(macro, follower::finishTurn);
        assertEquals(Macros.Outcome.SUCCESS, robot.macros.getOutcome());
        assertEquals(Math.toRadians(225), follower.pose.heading(), 1e-9);
        assertEquals(2, robot.macros.getShotsFired());
        assertEquals(0, robot.storage.count());
        assertEquals(Follower.Mode.MANUAL, follower.mode);
    }

    // ---- Localization and heading ----

    @Test
    public void relocalizeReportsNoTargetAndLeavesThePose() {
        Command macro = robot.macros.relocalize();
        assertTrue(macro.requirements().contains(robot.drivetrain));
        runToCompletion(macro);
        assertEquals(Macros.Outcome.NO_TARGET, robot.macros.getOutcome());
        assertEquals("pose must be untouched", 0, follower.setPoseCalls);
    }

    @Test
    public void snapToHeadingSucceedsWhenTheTurnArrives() {
        follower.pose = Pose.zero();
        runToCompletion(robot.macros.snapToHeading(Math.PI / 2), follower::finishTurn);
        assertEquals(Macros.Outcome.SUCCESS, robot.macros.getOutcome());
        assertEquals(Math.PI / 2, follower.lastHoldPose.heading(), 1e-9);
        assertEquals("sticks handed back after the snap", Follower.Mode.MANUAL, follower.mode);
    }

    @Test
    public void snapToHeadingTimesOutWhenTheTurnNeverArrives() {
        follower.pose = Pose.zero();
        runToCompletion(robot.macros.snapToHeading(Math.PI / 2));
        assertEquals(Macros.Outcome.TIMED_OUT, robot.macros.getOutcome());
        assertEquals("the interrupted turn must release the follower", Follower.Mode.MANUAL, follower.mode);
    }

    // ---- Pedro paths ----

    @Test
    public void driveToFollowsAPedroPathAndHandsTheFollowerBack() {
        follower.pose = Pose.zero();
        Pose target = new Pose(24, 12, Math.toRadians(90));
        runToCompletion(robot.macros.driveTo(target), () -> {
            if (follower.mode == Follower.Mode.FOLLOW) {   // the robot arrives
                follower.pose = target;
                follower.finishPath();
            }
        });
        assertEquals(Macros.Outcome.SUCCESS, robot.macros.getOutcome());
        assertEquals("one path, built at start", 1, follower.followCalls);
        assertNotNull(follower.lastPath);
        assertEquals(24, follower.lastPath.endPose().x(), 1e-6);
        assertEquals(12, follower.lastPath.endPose().y(), 1e-6);
        assertEquals("handed back to the sticks", Follower.Mode.MANUAL, follower.mode);
    }

    @Test
    public void driveToTimesOutWhenThePathNeverFinishes() {
        follower.pose = Pose.zero();
        int ticks = runToCompletion(robot.macros.driveTo(new Pose(24, 12, 0)));
        assertEquals(Macros.Outcome.TIMED_OUT, robot.macros.getOutcome());
        assertTrue(ticks >= Macros.DRIVE_TO_TIMEOUT_MS / 20);
        assertEquals(Follower.Mode.MANUAL, follower.mode);
    }

    // ---- Abort ----

    @Test
    public void abortMarksCancelledAndStopsTheMechanisms() {
        Command macro = robot.macros.intakeUntilFull();
        macro.schedule();
        tick();
        tick();
        tick();
        assertEquals(Macros.Outcome.RUNNING, robot.macros.getOutcome());
        assertEquals("intake", robot.macros.getActiveName());
        assertTrue(intakeMotor.commandedVelocity > 0);

        Scheduler.cancel(macro);
        robot.abortMacro();
        tick();
        assertEquals(Macros.Outcome.CANCELLED, robot.macros.getOutcome());
        assertEquals("idle", robot.macros.getActiveName());
        assertEquals(0, intakeMotor.commandedVelocity, EPS);
        assertEquals(0, storageMotor.commandedVelocity, EPS);
        assertEquals(Follower.Mode.MANUAL, follower.mode);
    }
}
