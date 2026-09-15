package org.firstinspires.ftc.teamcode;

import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertFalse;
import static org.junit.Assert.assertNotNull;
import static org.junit.Assert.assertNull;
import static org.junit.Assert.assertTrue;

import com.pedropathing.follower.Follower;
import com.pedropathing.ivy.Scheduler;
import com.pedropathing.math.Pose;

import org.firstinspires.ftc.teamcode.commands.Macros;
import org.firstinspires.ftc.teamcode.game.PieceType;
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
import org.firstinspires.ftc.teamcode.util.diagnostics.MatchLogger;
import org.firstinspires.ftc.teamcode.util.field.PoseFusion;
import org.firstinspires.ftc.teamcode.util.hardware.Hardware;
import org.firstinspires.ftc.teamcode.util.time.FakeClock;
import org.firstinspires.ftc.teamcode.util.time.MatchClock;
import org.junit.After;
import org.junit.Before;
import org.junit.Test;

/**
 * Robot composition against fakes: supplier wiring, the read -> decide -> write loop order,
 * localization write-back, the shared cancel path and the match-log row. Each test steps the loop
 * the way MatchOpMode does: readSensors, scheduler, writeActuators.
 */
public class RobotTest {
    private static final double EPS = 1e-9;

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
        robot = new Robot(
                new Drivetrain(follower, clock),
                new OpenLoopDrive((com.pedropathing.drivetrain.Drivetrain) null, clock),
                new Intake(intakeMotor, clock),
                new Storage(storageMotor, null, clock),
                new Transfer(transferMotor, clock),
                new Shooter(shooterMotor, null, clock),
                new Limelight(null),                 // no camera in the "config"
                new ColorSensor(null), new ColorSensor(null),
                new ColorSensor(null), new ColorSensor(null),
                clock);
        Storage.CAPACITY = 4;
        Intake.ANTI_JAM_ENABLED = true;
        Robot.VOLTAGE_SAMPLE_MS = 250;
    }

    @After
    public void tearDown() {
        Scheduler.reset();
        Hardware.reset();
    }

    /** One robot loop in production order: observe, decide, act. */
    private void tick() {
        robot.readSensors();
        Scheduler.execute();
        robot.writeActuators();
        clock.advance(20);
    }

    /** The full MatchOpMode loop, localization step included. */
    private void fullLoop() {
        robot.readSensors();
        robot.updateLocalization();
        Scheduler.execute();
        robot.writeActuators();
        clock.advance(20);
    }

    private Object cell(Object[] cells, String column) {
        String[] header = MatchLogger.BIOBUZZ_COLUMNS;
        for (int i = 0; i < header.length; i++) {
            if (header[i].equals(column)) return cells[i];
        }
        throw new AssertionError("no column " + column);
    }

    @Test
    public void composesEverySubsystemAndReportsWhatIsFitted() {
        assertNotNull(robot.drivetrain);
        assertNotNull(robot.intake);
        assertNotNull(robot.storage);
        assertNotNull(robot.transfer);
        assertNotNull(robot.shooter);
        assertNotNull(robot.limelight);
        assertNotNull(robot.openLoopDrive);
        assertFalse("no drive motors under test", robot.openLoopDrive.isAvailable());
        assertNotNull(robot.macros);
        assertNotNull(robot.poseFusion);
        assertTrue(robot.drivetrain.isAvailable());
        assertTrue(robot.intake.isAvailable());
        assertTrue(robot.shooter.isAvailable());
        assertFalse("no camera was fitted", robot.limelight.isAvailable());
        assertFalse(robot.storageEntranceSensor.isAvailable());
        assertEquals(Macros.Outcome.IDLE, robot.macros.getOutcome());
        assertEquals("idle", robot.macros.getActiveName());
        assertEquals(0, robot.getBatteryVolts(), EPS);
        assertNull("null before startMatch()", robot.getMatchClock());
        assertEquals(PieceType.POLLEN, robot.getBlobTarget());
        assertEquals(clock, robot.getClock());
    }

    @Test
    public void fullStorageHoldsTheIntakeRollerStill() {
        robot.storage.setCount(4);
        robot.intake.intakeCommand().schedule();
        tick();
        assertTrue("G407: intake blocked at four pieces", robot.intake.isBlockedByFullStorage());
        assertEquals(0, intakeMotor.commandedVelocity, EPS);

        robot.storage.setCount(3);
        tick();
        assertFalse(robot.intake.isBlockedByFullStorage());
        assertEquals(Intake.INTAKE_TICKS_PER_SEC, intakeMotor.commandedVelocity, EPS);
        assertFalse("no entrance sensor, so nothing is ever captured", robot.intake.hasPiece());
    }

    @Test
    public void transferFallsBackToTimedPulsesWithoutAFeedSensor() {
        assertFalse(robot.transfer.hasFeedSensor());
        assertFalse(robot.transfer.pieceAtFeed());
        assertFalse("no transfer sensor: the storage count cannot decrement itself", robot.storage.hasExitSensor());
        assertFalse("no entrance sensor: the count cannot rise, so zero is unknown", robot.storage.hasEntranceSensor());
        assertFalse(robot.macros.isCountKnown());
    }

    @Test
    public void matchClockStartsForThePeriodAndTicksInReadSensors() {
        robot.startMatch(MatchClock.Period.TELEOP);
        MatchClock mc = robot.getMatchClock();
        assertNotNull(mc);
        assertEquals(MatchClock.Period.TELEOP, mc.getPeriod());
        assertEquals(MatchClock.Phase.RUNNING, mc.getPhase());

        clock.advance(1000);
        robot.readSensors();
        assertEquals(1000, mc.getElapsedMs());

        robot.startMatch(MatchClock.Period.AUTONOMOUS);
        assertEquals(MatchClock.Period.AUTONOMOUS, robot.getMatchClock().getPeriod());
    }

    @Test
    public void writeActuatorsAppliesWhatTheSchedulerDecidedThisLoop() {
        assertEquals(0, shooterMotor.commandedVelocity, EPS);
        robot.shooter.spinUpCommand().schedule();
        tick();
        assertEquals(Shooter.rpmToTicksPerSec(Shooter.SHOOT_RPM), shooterMotor.commandedVelocity, EPS);
        assertEquals("drivetrain ticked once per loop", 1, follower.updateCalls);
    }

    @Test
    public void updateLocalizationLeavesTheFollowerAloneWithoutAFix() {
        follower.pose = new Pose(10, 20, 0.5);
        for (int i = 0; i < 20; i++) {
            robot.updateLocalization();
            follower.pose = new Pose(10 + i, 20, 0.5);      // the robot drives on
            clock.advance(20);
        }
        assertEquals("odometry only: nothing is written back", 0, follower.setPoseCalls);
        assertEquals(PoseFusion.Result.ODOMETRY_ONLY, robot.poseFusion.getLastResult());
        assertTrue(robot.poseFusion.isSeeded());
    }

    @Test
    public void headingHoldCorrectsThroughTheFullLoop() {
        // fixthese B1: the per-loop pose write-back used to release the hold every loop, so it never
        // produced a correction on the robot. This runs the real loop order, localization included.
        follower.pose = new Pose(0, 0, 0);
        robot.drivetrain.driverControlCommand(() -> 0, () -> 0, () -> 0).schedule();
        fullLoop();
        fullLoop();
        assertTrue("centred sticks capture the heading", robot.drivetrain.isHeadingHoldActive());
        follower.pose = new Pose(0, 0, 0.2);                  // knocked 0.2 rad counter-clockwise
        fullLoop();
        fullLoop();
        fullLoop();
        assertTrue("still holding", robot.drivetrain.isHeadingHoldActive());
        assertTrue("correcting back clockwise: turn " + follower.lastTurn, follower.lastTurn < -0.05);
    }

    @Test
    public void aprilTagLocalizationReportsNoFixWithoutACamera() {
        assertFalse(robot.tryLocalizeFromAprilTag());
        assertEquals(0, follower.setPoseCalls);
        assertFalse(robot.poseFusion.isSeeded());
    }

    @Test
    public void abortMacroHandsTheFollowerBackAndReportsIdle() {
        follower.mode = Follower.Mode.FOLLOW;
        robot.abortMacro();
        assertEquals(1, follower.manualCalls);
        assertEquals(Follower.Mode.MANUAL, follower.mode);
        assertEquals("idle", robot.macros.getActiveName());
        assertEquals("nothing was running, so nothing was cancelled",
                Macros.Outcome.IDLE, robot.macros.getOutcome());
    }

    @Test
    public void logCellsMatchTheLoggerHeader() {
        Object[] cells = robot.logCells(12.5);
        assertEquals(MatchLogger.BIOBUZZ_COLUMNS.length, cells.length);
        assertEquals(12.5, (Double) cell(cells, "loop_ms"), EPS);
        assertEquals("NONE", cell(cells, "phase"));
        assertTrue(Double.isNaN((Double) cell(cells, "remaining_s")));
        assertEquals("OTHER", cell(cells, "path_mode"));
        assertTrue("no Pedro follower under test", Double.isNaN((Double) cell(cells, "path_completion")));
        assertTrue(Double.isNaN((Double) cell(cells, "trans_error")));
        assertTrue(Double.isNaN((Double) cell(cells, "heading_error")));
        assertTrue("not holding a heading", Double.isNaN((Double) cell(cells, "heading_hold_deg")));
        assertEquals(0, cell(cells, "aim_lock"));
        assertEquals(0, cell(cells, "storage_count"));
        assertEquals("idle", cell(cells, "macro"));
        assertEquals(Macros.Outcome.IDLE, cell(cells, "macro_outcome"));

        robot.startMatch(MatchClock.Period.TELEOP);
        robot.storage.setCount(2);
        follower.mode = Follower.Mode.FOLLOW;
        cells = robot.logCells(8.0);
        assertEquals("RUNNING", cell(cells, "phase"));
        assertEquals(2, cell(cells, "storage_count"));
        assertEquals("FOLLOW", cell(cells, "path_mode"));
    }

    @Test
    public void stopPathsAreSafeWithoutHardwareAndZeroTheMechanisms() {
        robot.shooter.spinUp();
        robot.intake.intake();
        robot.writeActuators();
        assertTrue(shooterMotor.commandedVelocity > 0);
        assertTrue(intakeMotor.commandedVelocity > 0);

        robot.stopMechanisms();
        robot.writeActuators();
        assertEquals(0, shooterMotor.commandedVelocity, EPS);
        assertEquals(0, intakeMotor.commandedVelocity, EPS);
        assertEquals(1, follower.manualCalls);

        robot.stop();   // no camera, no sensors: must not throw
    }

    @Test
    public void blobTargetCanBeChangedAndNeverNulled() {
        robot.setBlobTarget(PieceType.NECTAR);
        assertEquals(PieceType.NECTAR, robot.getBlobTarget());
        robot.setBlobTarget(null);
        assertEquals(PieceType.NECTAR, robot.getBlobTarget());
    }
}
