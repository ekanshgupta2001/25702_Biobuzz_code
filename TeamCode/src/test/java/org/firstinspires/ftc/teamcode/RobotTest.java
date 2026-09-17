package org.firstinspires.ftc.teamcode;

import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertFalse;
import static org.junit.Assert.assertNotNull;
import static org.junit.Assert.assertNull;
import static org.junit.Assert.assertTrue;

import com.pedropathing.follower.Follower;
import com.pedropathing.ivy.Scheduler;
import com.pedropathing.math.Pose;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;

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
import org.firstinspires.ftc.teamcode.subsystems.FakeColorOnlySensor;
import org.firstinspires.ftc.teamcode.subsystems.FakeColorRangeSensor;
import org.firstinspires.ftc.teamcode.util.diagnostics.MatchLogger;
import org.firstinspires.ftc.teamcode.util.field.Alliance;
import org.firstinspires.ftc.teamcode.util.hardware.Hardware;
import org.firstinspires.ftc.teamcode.util.time.FakeClock;
import org.firstinspires.ftc.teamcode.util.time.MatchClock;
import org.junit.After;
import org.junit.Before;
import org.junit.Test;

/**
 * Robot composition against fakes: supplier wiring, the read -> decide -> write loop order,
 * the shared cancel path and the match-log row. Each test steps the loop
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
        Intake.REJECT_ENABLED = false;
        PieceType.HUES_CALIBRATED = false;
    }

    /** The fixture robot with real (fake) sensors at the four points; {@code null} = not fitted. */
    private Robot robotWithSensors(NormalizedColorSensor entrance, NormalizedColorSensor full,
                                   NormalizedColorSensor transfer, NormalizedColorSensor feed) {
        return new Robot(
                new Drivetrain(follower, clock),
                new OpenLoopDrive((com.pedropathing.drivetrain.Drivetrain) null, clock),
                new Intake(intakeMotor, clock),
                new Storage(storageMotor, null, clock),
                new Transfer(transferMotor, clock),
                new Shooter(shooterMotor, null, clock),
                new Limelight(null),
                new ColorSensor(entrance, 2f, true), new ColorSensor(full, 2f, true),
                new ColorSensor(transfer, 2f, true), new ColorSensor(feed, 2f, true),
                clock);
    }

    /** One robot loop in production order: observe, decide, act. */
    private void tick() {
        robot.readSensors();
        Scheduler.execute();
        robot.writeActuators();
        clock.advance(20);
    }

    /** The full MatchOpMode loop. */
    private void fullLoop() {
        robot.readSensors();
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
        assertTrue(robot.drivetrain.isAvailable());
        assertTrue(robot.intake.isAvailable());
        assertTrue(robot.shooter.isAvailable());
        assertFalse("no camera was fitted", robot.limelight.isAvailable());
        assertFalse(robot.storageEntranceSensor.isAvailable());
        assertEquals(Macros.Outcome.IDLE, robot.macros.getOutcome());
        assertEquals("idle", robot.macros.getActiveName());
        assertEquals(0, robot.getBatteryVolts(), EPS);
        assertNull("null before startMatch()", robot.getMatchClock());
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
    }

    @Test
    public void transferFallsBackToTimedPulsesWithoutAFeedSensor() {
        assertFalse(robot.transfer.hasFeedSensor());
        assertFalse(robot.transfer.pieceAtFeed());
        assertFalse("no transfer sensor: the storage count cannot decrement itself", robot.storage.hasExitSensor());
        assertFalse("no entrance sensor: the count cannot rise, so zero is unknown", robot.storage.hasEntranceSensor());
        assertFalse(robot.macros.isCountKnown());
        assertFalse("no full sensor either: nothing can say full", robot.storage.canDetectFull());
        assertEquals("entrance=none (count unknown, shoots blind) | full=none | transfer=none | feed=none (timed pulses)",
                robot.sensingSummary());
    }

    @Test
    public void entranceCountsByDistanceBeforeTheHuesAreMeasured() {
        // fixthese R2-A3: a plugged-in entrance sensor with unmeasured hue windows used to leave the
        // count at 0 for ever, and 0 with a "fitted" sensor meant every shoot macro was NO_TARGET.
        // A REV V3's proximity read needs no calibration, so the count runs on that.
        assertFalse(PieceType.HUES_CALIBRATED);
        FakeColorRangeSensor entrance = new FakeColorRangeSensor();      // black: no hue window matches
        robot = robotWithSensors(entrance, null, null, null);
        assertTrue("trusted for counting, by distance", robot.storage.hasEntranceSensor());
        assertTrue(robot.macros.isCountKnown());
        assertEquals(0, robot.macros.piecesOnBoard());
        assertTrue(robot.sensingSummary(), robot.sensingSummary().startsWith("entrance=count by distance (hues not measured"));

        robot.intake.intakeCommand().schedule();
        tick();
        assertEquals(0, robot.storage.count());
        entrance.distanceInches = 1.0;
        tick();
        assertEquals("one piece, by distance alone", 1, robot.storage.count());
        entrance.distanceInches = 100;
        tick();
        entrance.distanceInches = 1.0;
        tick();
        assertEquals(2, robot.storage.count());
        assertFalse("no measured hues: no G408 reject is wired", robot.intake.isRejecting());
    }

    @Test
    public void aHueOnlyEntranceSensorIsNotTrustedUntilCalibrated() {
        // In the configuration is not the same as measured. A sensor that can only match hues nobody
        // has tuned is "not fitted" to the count, so the shooting macros fire blind, not NO_TARGET.
        FakeColorOnlySensor entrance = new FakeColorOnlySensor().showing(0.9f, 0.9f, 0.1f);   // yellow POLLEN
        robot = robotWithSensors(entrance, null, null, null);
        assertTrue(robot.storageEntranceSensor.isAvailable());
        assertFalse(robot.storageEntranceSensor.hasDistance());
        assertFalse("present but not trusted", robot.storage.hasEntranceSensor());
        assertFalse(robot.macros.isCountKnown());
        assertEquals(Storage.CAPACITY, robot.macros.piecesOnBoard());
        assertTrue(robot.sensingSummary(), robot.sensingSummary().contains("NOT trusted"));

        PieceType.HUES_CALIBRATED = true;
        robot = robotWithSensors(entrance, null, null, null);
        assertTrue("measured hues are trusted", robot.storage.hasEntranceSensor());
        assertTrue(robot.sensingSummary(), robot.sensingSummary().startsWith("entrance=count by hue, G408 reject wired"));
        robot.intake.intakeCommand().schedule();
        tick();
        assertEquals(1, robot.storage.count());
    }

    @Test
    public void presenceSensorsAreReadInRotationAndTheEntranceEveryLoop() {
        // fixthese R2-A1: four fitted colour sensors were eight I2C transactions per loop. The
        // entrance is an edge and keeps every sample; the presence points share the bus.
        FakeColorRangeSensor entrance = new FakeColorRangeSensor();
        FakeColorRangeSensor full = new FakeColorRangeSensor();
        FakeColorRangeSensor transfer = new FakeColorRangeSensor();
        FakeColorRangeSensor feed = new FakeColorRangeSensor();
        robot = robotWithSensors(entrance, full, transfer, feed);
        for (int i = 0; i < 6; i++) robot.readSensors();
        assertEquals(6, entrance.colorReads);
        assertEquals(2, full.colorReads);
        assertEquals(2, transfer.colorReads);
        assertEquals(2, feed.colorReads);
        assertEquals("distance rides along with each colour read", 2, feed.distanceReads);

        // With one presence sensor fitted it is read every loop, so nothing changes on today's robot.
        FakeColorRangeSensor onlyFull = new FakeColorRangeSensor();
        robot = robotWithSensors(null, onlyFull, null, null);
        for (int i = 0; i < 6; i++) robot.readSensors();
        assertEquals(6, onlyFull.colorReads);
    }

    @Test
    public void aFullSensorIsKnownToTheStorage() {
        // fixthese R2-A2: the full sensor is the first one to fit. Storage must know it exists so a
        // timed intake can tell "ended because full" from "no way to know".
        FakeColorRangeSensor full = new FakeColorRangeSensor();
        robot = robotWithSensors(null, full, null, null);
        assertTrue(robot.storage.hasFullSensor());
        assertTrue(robot.storage.canDetectFull());
        assertFalse(robot.storage.hasEntranceSensor());
        assertTrue(robot.sensingSummary().contains("full=fitted"));
        robot.intake.intakeCommand().schedule();
        tick();
        assertEquals(Intake.INTAKE_TICKS_PER_SEC, intakeMotor.commandedVelocity, EPS);
        full.distanceInches = 1.0;
        tick();
        assertTrue("the last slot is seen: full, count or no count", robot.storage.isFull());
        assertEquals("G407: roller held still", 0, intakeMotor.commandedVelocity, EPS);
    }

    @Test
    public void presenceUsesDistanceWhenTheSensorHasIt() {
        // fixthese C1: the interlocks used to be a hue match against unmeasured windows, which fails
        // silently to "no piece". A REV V3's proximity read does not care about colour or lighting.
        FakeColorRangeSensor transferPoint = new FakeColorRangeSensor();     // black, far away
        robot = robotWithSensors(null, null, transferPoint, null);
        assertTrue("the transfer sensor is the storage's exit sensor", robot.storage.hasExitSensor());

        robot.readSensors();
        assertFalse(robot.transfer.hasPieceInLift());

        transferPoint.distanceInches = Robot.PRESENCE_DISTANCE_INCHES - 0.5;   // a piece, any colour
        robot.readSensors();
        assertTrue("near enough counts, whatever the hue", robot.transfer.hasPieceInLift());

        transferPoint.distanceInches = Robot.PRESENCE_DISTANCE_INCHES + 3;
        robot.readSensors();
        assertFalse(robot.transfer.hasPieceInLift());
    }

    @Test
    public void opponentNectarTriggersRejectOnlyForTheOtherAlliance() {
        Intake.REJECT_ENABLED = true;
        PieceType.HUES_CALIBRATED = true;              // the reject is wired only once the hues are measured
        FakeColorRangeSensor entrance = new FakeColorRangeSensor().showing(0.1f, 0.1f, 0.9f);   // blue NECTAR
        entrance.distanceInches = 1.0;                 // and presence is by distance
        robot = robotWithSensors(entrance, null, null, null);
        robot.setAlliance(Alliance.RED);
        robot.intake.intakeCommand().schedule();
        tick();
        assertTrue("red robot, blue piece: throw it back", robot.intake.isRejecting());
        assertEquals(Intake.EJECT_TICKS_PER_SEC, intakeMotor.commandedVelocity, EPS);
        assertEquals("a rejected piece is not counted into the queue", 0, robot.storage.count());

        robot = robotWithSensors(entrance, null, null, null);
        robot.setAlliance(Alliance.BLUE);
        robot.intake.intakeCommand().schedule();
        tick();
        assertFalse("blue robot, blue piece: ours", robot.intake.isRejecting());
        assertEquals(Intake.INTAKE_TICKS_PER_SEC, intakeMotor.commandedVelocity, EPS);
        assertEquals("and it is counted", 1, robot.storage.count());
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
        robot.shooter.holdSpeedCommand().schedule();
        tick();
        assertEquals(Shooter.rpmToTicksPerSec(Shooter.SHOOT_RPM), shooterMotor.commandedVelocity, EPS);
        assertEquals("drivetrain ticked once per loop", 1, follower.updateCalls);
    }

    @Test
    public void headingHoldCorrectsThroughTheFullLoop() {
        // fixthese B1: the per-loop pose write-back used to release the hold every loop, so it never
        // produced a correction on the robot. This runs the real loop order.
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

}
