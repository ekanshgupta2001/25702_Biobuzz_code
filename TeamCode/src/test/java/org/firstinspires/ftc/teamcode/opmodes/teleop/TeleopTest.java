package org.firstinspires.ftc.teamcode.opmodes.teleop;

import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertFalse;
import static org.junit.Assert.assertTrue;
import static org.junit.Assert.fail;

import com.pedropathing.follower.Follower;
import com.pedropathing.math.Pose;
import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.commands.Macros;
import org.firstinspires.ftc.teamcode.game.Field;
import org.firstinspires.ftc.teamcode.game.FieldPoses;
import org.firstinspires.ftc.teamcode.opmodes.FakeTelemetry;
import org.firstinspires.ftc.teamcode.opmodes.RobotTunables;
import org.firstinspires.ftc.teamcode.subsystems.ColorSensor;
import org.firstinspires.ftc.teamcode.subsystems.Drivetrain;
import org.firstinspires.ftc.teamcode.subsystems.FakeDcMotorEx;
import org.firstinspires.ftc.teamcode.subsystems.FakePathFollower;
import org.firstinspires.ftc.teamcode.subsystems.FakePedroDrivetrain;
import org.firstinspires.ftc.teamcode.subsystems.Intake;
import org.firstinspires.ftc.teamcode.subsystems.Limelight;
import org.firstinspires.ftc.teamcode.subsystems.OpenLoopDrive;
import org.firstinspires.ftc.teamcode.subsystems.PathFollower;
import org.firstinspires.ftc.teamcode.subsystems.Shooter;
import org.firstinspires.ftc.teamcode.subsystems.Storage;
import org.firstinspires.ftc.teamcode.subsystems.Transfer;
import org.firstinspires.ftc.teamcode.util.diagnostics.MatchLogger;
import org.firstinspires.ftc.teamcode.util.diagnostics.Tunables;
import org.firstinspires.ftc.teamcode.util.field.Alliance;
import org.firstinspires.ftc.teamcode.util.field.PoseStorage;
import org.firstinspires.ftc.teamcode.util.field.StartPosition;
import org.firstinspires.ftc.teamcode.util.hardware.Hardware;
import org.firstinspires.ftc.teamcode.util.math.DriveScaling;
import org.firstinspires.ftc.teamcode.util.time.FakeClock;
import org.junit.After;
import org.junit.Before;
import org.junit.Test;

import com.pedropathing.ivy.Scheduler;

import java.util.function.Consumer;

/**
 * The real {@link Teleop} through its real SDK lifecycle ({@code init}, {@code init_loop},
 * {@code start}, {@code loop}) on the JVM: a robot made of fakes through the {@code buildRobot()}
 * seam, no log file, a recording telemetry, and real {@link Gamepad}s fed with {@code copy()} the
 * way the robot controller feeds them. Time is the injected fake clock; one loop is 20 ms.
 */
public class TeleopTest {
    private static final double EPS = 1e-6;
    private static final int MAX_LOOPS = 3000;

    private FakeClock clock;
    private FakePathFollower follower;
    private FakeDcMotorEx intakeMotor;
    private FakeDcMotorEx storageMotor;
    private FakeDcMotorEx transferMotor;
    private FakeDcMotorEx shooterMotor;
    private Robot robot;
    private FakeTelemetry telemetry;
    private TestTeleop op;

    /** The real Teleop with its two hardware seams overridden. */
    private final class TestTeleop extends Teleop {
        @Override
        protected Robot buildRobot() {
            return TeleopTest.this.robot;   // the fixture, not the inherited (still null) field
        }

        @Override
        protected MatchLogger openLogger() {
            return null;
        }
    }

    @Before
    public void setUp() {
        Scheduler.reset();
        Hardware.reset();
        PoseStorage.clear();
        Tunables.resetForTests();
        Teleop.DEBUG_TELEMETRY = false;
        Storage.CAPACITY = 4;
        Drivetrain.MIN_PATH_MS = 60;
        Limelight.MOUNT_CALIBRATED = true;      // most tests exercise the camera macros as if measured

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
                new Limelight(null),
                new ColorSensor(null), new ColorSensor(null),
                new ColorSensor(null), new ColorSensor(null),
                clock);
        telemetry = new FakeTelemetry();
        op = new TestTeleop();
        op.telemetry = telemetry;
        op.gamepad1 = new Gamepad();
        op.gamepad2 = new Gamepad();
    }

    @After
    public void tearDown() {
        Scheduler.reset();
        Hardware.reset();
        PoseStorage.clear();
        Tunables.resetForTests();
        Teleop.DEBUG_TELEMETRY = false;
        Limelight.MOUNT_CALIBRATED = false;
    }

    // ---- Helpers ----

    /** The pad's state changes to what {@code set} describes; edges fire as on the robot. */
    private static void press(Gamepad pad, Consumer<Gamepad> set) {
        Gamepad source = new Gamepad();
        set.accept(source);
        pad.copy(source);
    }

    private static void release(Gamepad pad) {
        pad.copy(new Gamepad());
    }

    private void loop() {
        op.loop();
        clock.advance(20);
    }

    private void initAndStart() {
        op.init();
        op.init_loop();
        op.start();
    }

    /** Rebuilds the fixture as the first-event robot: no follower, the sticks reach the motors open loop. */
    private FakePedroDrivetrain useUntunedRobot() {
        FakePedroDrivetrain motors = new FakePedroDrivetrain();
        robot = new Robot(
                new Drivetrain((PathFollower) null, clock),
                new OpenLoopDrive(motors, clock),
                new Intake(intakeMotor, clock),
                new Storage(storageMotor, null, clock),
                new Transfer(transferMotor, clock),
                new Shooter(shooterMotor, null, clock),
                new Limelight(null),
                new ColorSensor(null), new ColorSensor(null),
                new ColorSensor(null), new ColorSensor(null),
                clock);
        return motors;
    }

    private void loopUntilMacroDone() {
        int loops = 0;
        while (robot.macros.isRunning()) {
            loop();
            if (++loops > MAX_LOOPS) fail("macro did not finish: " + robot.macros.getStatus());
        }
    }

    // ---- Driving ----

    @Test
    public void stickForwardDrivesThroughThePedroFollower() {
        initAndStart();                                     // blue by default
        assertEquals("start() hands the follower to the sticks", Follower.Mode.MANUAL, follower.mode);
        follower.pose = new Pose(0, 0, Math.PI);           // facing away from the blue wall (+X)
        press(op.gamepad1, g -> g.left_stick_y = -1f);
        loop();
        assertEquals(Follower.Mode.MANUAL, follower.mode);
        assertEquals("full stick forward is full forward power", 1.0, follower.lastForward, EPS);
        assertEquals(0, follower.lastStrafe, EPS);
        assertEquals(0, follower.lastTurn, EPS);

        follower.pose = new Pose(0, 0, 0);                 // now facing the blue driver
        loop();
        assertEquals("field-centric: stick up still drives away from the driver", -1.0, follower.lastForward, EPS);
    }

    @Test
    public void aLightTurnStickIsNotSwallowedByTheHold() {
        initAndStart();
        loop();                                             // centred sticks: the hold captures
        assertTrue(robot.drivetrain.isHeadingHoldActive());
        press(op.gamepad1, g -> g.right_stick_x = -0.2f);   // a small, deliberate turn to the left
        loop();
        assertFalse("the driver is steering", robot.drivetrain.isHeadingHoldActive());
        assertEquals("the shaped stick reaches the follower unchanged",
                DriveScaling.shape(0.2), follower.lastTurn, 1e-6);
    }

    @Test
    public void redForwardIsFieldPlusX() {
        PoseStorage.save(new Pose(0, 0, 0), Alliance.RED, StartPosition.FACING_HIVE);
        initAndStart();
        press(op.gamepad1, g -> g.left_stick_y = -1f);
        loop();
        assertEquals("red drivers face +X", 1.0, follower.lastForward, EPS);
        assertEquals(0, robot.drivetrain.getDriverHeadingOffset(), EPS);
    }

    @Test
    public void dpadOverridesTheAllianceLeftByAuto() {
        PoseStorage.save(null, Alliance.RED, StartPosition.FACING_HIVE);
        op.init();
        op.init_loop();
        assertTrue(telemetry.contains("RED (from auto)"));
        press(op.gamepad1, g -> g.dpad_right = true);      // the practice field left the wrong one
        op.init_loop();
        assertTrue(telemetry.joined(), telemetry.contains("BLUE (dpad, overrode auto)"));
        assertTrue("blue's up-CELL tags", telemetry.contains("tags 42-45"));
        assertEquals("driver frame followed the alliance", Math.PI, robot.drivetrain.getDriverHeadingOffset(), EPS);
    }

    @Test
    public void stickForwardDrivesOpenLoopWhenPedroIsNotTuned() {
        FakePedroDrivetrain motors = useUntunedRobot();
        initAndStart();
        assertTrue(telemetry.contains("OPEN LOOP (Pedro not tuned)"));
        press(op.gamepad1, g -> g.left_stick_y = -1f);
        loop();
        assertTrue(motors.moving);
        assertEquals("full stick forward is full forward power", 1.0, motors.lastPowers.forward(), EPS);
        assertTrue("through Pedro's manual write", motors.lastManual);
        release(op.gamepad1);
        loop();
        assertFalse("centred sticks stop the motors", motors.moving);

        press(op.gamepad1, g -> g.b = true);              // DRIVE_TO_SHOOT: nothing to follow a path
        loop();
        assertEquals("no drive macro without a follower", "idle", robot.macros.getActiveName());
        assertEquals(Macros.Outcome.IDLE, robot.macros.getOutcome());
        assertTrue("the driver is told", op.gamepad1.isRumbling());
    }

    @Test
    public void bLaunchesAPedroPathToTheShootingSpot() {
        initAndStart();
        press(op.gamepad1, g -> g.b = true);
        loop();
        loop();
        assertEquals("driveTo", robot.macros.getActiveName());
        assertEquals(Macros.Outcome.RUNNING, robot.macros.getOutcome());
        assertEquals(1, follower.followCalls);
        assertEquals(Follower.Mode.FOLLOW, follower.mode);
        Pose end = follower.lastPath.endPose();
        assertEquals(FieldPoses.BLUE_SHOOTING_SPOT.x(), end.x(), EPS);
        assertEquals(FieldPoses.BLUE_SHOOTING_SPOT.y(), end.y(), EPS);
    }

    @Test
    public void arrivingReportsSuccessAndRumblesTheDriver() {
        initAndStart();
        press(op.gamepad1, g -> g.b = true);
        loop();
        release(op.gamepad1);
        loop();
        follower.pose = FieldPoses.BLUE_SHOOTING_SPOT;     // the robot arrives
        follower.finishPath();
        loopUntilMacroDone();
        assertEquals(Macros.Outcome.SUCCESS, robot.macros.getOutcome());
        assertTrue("one blip for a success", op.gamepad1.isRumbling());
        assertEquals("sticks are back", Follower.Mode.MANUAL, follower.mode);
    }

    @Test
    public void driverAStartsCollectAndAStickAbortsIt() {
        initAndStart();
        press(op.gamepad1, g -> g.a = true);
        loop();
        assertEquals("collect", robot.macros.getActiveName());
        assertEquals(Macros.Outcome.RUNNING, robot.macros.getOutcome());
        release(op.gamepad1);
        loop();
        assertEquals(Macros.Outcome.RUNNING, robot.macros.getOutcome());

        follower.pose = new Pose(0, 0, Math.PI);           // blue robot facing away from its driver
        press(op.gamepad1, g -> g.left_stick_y = -1f);   // the driver grabs the sticks
        loop();
        assertEquals(Macros.Outcome.CANCELLED, robot.macros.getOutcome());
        assertEquals("idle", robot.macros.getActiveName());
        loop();
        assertEquals("driver control resumed by itself", Follower.Mode.MANUAL, follower.mode);
        assertEquals(1.0, follower.lastForward, EPS);
    }

    @Test
    public void cameraMacrosAreIgnoredUntilTheMountIsMeasured() {
        Limelight.MOUNT_CALIBRATED = false;
        op.init();
        op.init_loop();
        assertTrue(telemetry.joined(), telemetry.contains("Camera macros"));
        op.start();
        press(op.gamepad1, g -> g.a = true);                // COLLECT
        loop();
        assertEquals("no macro from unmeasured geometry", "idle", robot.macros.getActiveName());
        assertTrue("the driver is told", op.gamepad1.isRumbling());
        press(op.gamepad1, g -> g.b = true);                // a Pedro path needs no camera
        loop();
        assertEquals("driveTo", robot.macros.getActiveName());
    }

    @Test
    public void aStickDoesNotAbortAShotInProgress() {
        initAndStart();
        robot.storage.setCount(2);
        press(op.gamepad2, g -> g.right_trigger = 1f);     // SHOOT_ONE
        loop();
        assertEquals("shootOne", robot.macros.getActiveName());
        follower.pose = new Pose(0, 0, Math.PI);           // blue robot facing away from its driver
        press(op.gamepad1, g -> g.left_stick_y = -1f);
        loop();
        assertEquals("the shot does not own the drivetrain", Macros.Outcome.RUNNING, robot.macros.getOutcome());
        assertEquals("and the driver keeps driving meanwhile", 1.0, follower.lastForward, EPS);
    }

    // ---- Operator ----

    @Test
    public void operatorShootOneFeedsOnASensorlessRobot() {
        // No entrance sensor, nothing from auto: the count is unknown, and the button must still work.
        initAndStart();
        shooterMotor.measuredVelocity = Shooter.rpmToTicksPerSec(Shooter.SHOOT_RPM);
        loop();
        assertTrue(telemetry.joined(), telemetry.contains("Pieces: ?"));
        press(op.gamepad2, g -> g.right_trigger = 1f);     // SHOOT_ONE
        loop();
        assertEquals("shootOne", robot.macros.getActiveName());
        release(op.gamepad2);
        loopUntilMacroDone();
        assertEquals(Macros.Outcome.SUCCESS, robot.macros.getOutcome());
        assertEquals(1, robot.macros.getShotsFired());
        assertEquals(Transfer.FEED_TICKS_PER_SEC, transferMotor.maxCommandedVelocity, EPS);
    }

    @Test
    public void operatorCanMarkTheRobotFullWithoutASensor() {
        // fixthese R2-A2/A3: on the sensorless robot the operator is the entrance sensor. Four on
        // board holds the roller (the G407 system) and makes Shoot All fire exactly four; dpad left
        // forgets it and the card is back to "?".
        initAndStart();
        assertTrue(telemetry.joined(), telemetry.contains("Sensors: entrance=none"));
        press(op.gamepad2, g -> g.right_bumper = true);          // INTAKE
        loop();
        release(op.gamepad2);
        loop();
        assertEquals(Intake.INTAKE_TICKS_PER_SEC, intakeMotor.commandedVelocity, EPS);

        press(op.gamepad2, g -> g.dpad_up = true);               // MARK_FULL
        loop();
        release(op.gamepad2);
        telemetry.clear();
        loop();
        assertTrue(robot.storage.isFull());
        assertTrue(robot.macros.isCountKnown());
        assertEquals("roller held still at four", 0, intakeMotor.commandedVelocity, EPS);
        assertTrue(telemetry.joined(), telemetry.contains("Pieces: 4/4  FULL"));

        shooterMotor.measuredVelocity = Shooter.rpmToTicksPerSec(Shooter.SHOOT_RPM);
        press(op.gamepad2, g -> g.a = true);                     // SHOOT_ALL
        loop();
        release(op.gamepad2);
        loopUntilMacroDone();
        assertEquals(Macros.Outcome.SUCCESS, robot.macros.getOutcome());
        assertEquals("exactly the four the operator declared", 4, robot.macros.getShotsFired());

        robot.storage.setCount(2);
        press(op.gamepad2, g -> g.dpad_left = true);             // MARK_EMPTY
        loop();
        release(op.gamepad2);
        telemetry.clear();
        loop();
        assertFalse(robot.macros.isCountKnown());
        assertTrue(telemetry.joined(), telemetry.contains("Pieces: ?  (assumes 4"));
    }

    @Test
    public void operatorIntakeButtonsGoThroughCommands() {
        initAndStart();
        press(op.gamepad2, g -> g.right_bumper = true);
        loop();
        assertEquals(Intake.INTAKE_TICKS_PER_SEC, intakeMotor.commandedVelocity, EPS);
        release(op.gamepad2);
        loop();
        assertEquals("keeps running until told otherwise", Intake.INTAKE_TICKS_PER_SEC, intakeMotor.commandedVelocity, EPS);
        press(op.gamepad2, g -> g.x = true);
        loop();
        assertEquals(0, intakeMotor.commandedVelocity, EPS);
    }

    @Test
    public void operatorStopCancelsAMechanismMacro() {
        initAndStart();
        press(op.gamepad2, g -> g.y = true);               // INTAKE_UNTIL_FULL
        loop();
        assertEquals("no sensor can end it early", "intake (timed)", robot.macros.getActiveName());
        release(op.gamepad2);
        loop();
        assertTrue(intakeMotor.commandedVelocity > 0);
        press(op.gamepad2, g -> g.x = true);
        loop();
        assertEquals(Macros.Outcome.CANCELLED, robot.macros.getOutcome());
        assertEquals(0, intakeMotor.commandedVelocity, EPS);
    }

    @Test
    public void rightTriggerHoldsAnAimLockOnTheUpCell() {
        initAndStart();                                     // blue, at the origin facing +X
        press(op.gamepad1, g -> g.right_trigger = 1f);
        loop();
        assertTrue(robot.drivetrain.isAimLocked());
        Pose far = Field.cell(Alliance.BLUE, Field.CellSide.FAR);
        double wanted = robot.macros.aimHeading(far, 42, 45);
        assertEquals("rear toward the far CELL", Math.PI + Math.atan2(far.y(), far.x()), wanted, 1e-9);
        assertEquals(wanted, robot.drivetrain.getHeldHeading(), 1e-9);
        assertTrue("turning the short way, clockwise", follower.lastTurn < 0);
        assertTrue(telemetry.contains("LOCKED on BLUE FAR CELL"));

        release(op.gamepad1);
        loop();
        assertFalse("released with the trigger", robot.drivetrain.isAimLocked());
    }

    @Test
    public void hiveTippedFlipsTheAimTarget() {
        initAndStart();
        press(op.gamepad1, g -> g.right_trigger = 1f);
        loop();
        double far = robot.drivetrain.getHeldHeading();
        press(op.gamepad2, g -> g.dpad_down = true);        // our HIVE tipped
        loop();
        Pose audience = Field.cell(Alliance.BLUE, Field.CellSide.AUDIENCE);
        assertEquals(robot.macros.aimHeading(audience, 38, 41), robot.drivetrain.getHeldHeading(), 1e-9);
        assertTrue(robot.drivetrain.getHeldHeading() != far);
        assertTrue(telemetry.contains("LOCKED on BLUE AUDIENCE CELL"));
    }

    @Test
    public void aimLockSurvivesAShot() {
        initAndStart();
        robot.storage.setCount(2);
        shooterMotor.measuredVelocity = Shooter.rpmToTicksPerSec(Shooter.SHOOT_RPM);
        press(op.gamepad1, g -> g.right_trigger = 1f);
        loop();
        press(op.gamepad2, g -> g.a = true);                // SHOOT_ALL
        loop();
        release(op.gamepad2);
        assertEquals("shootAll", robot.macros.getActiveName());
        assertTrue("the shot never takes the drivetrain", robot.drivetrain.isAimLocked());
        loopUntilMacroDone();
        assertEquals(Macros.Outcome.SUCCESS, robot.macros.getOutcome());
        assertEquals(2, robot.macros.getShotsFired());
        assertTrue(robot.drivetrain.isAimLocked());
        assertTrue("still steering toward the CELL", follower.lastTurn < 0);
    }

    @Test
    public void leftTriggerArmsAndDisarmsTheFlywheel() {
        initAndStart();
        press(op.gamepad2, g -> g.left_trigger = 1f);
        loop();
        assertEquals(Shooter.SHOOT_RPM, robot.shooter.getTargetRpm(), EPS);
        release(op.gamepad2);
        loop();
        assertEquals("still armed", Shooter.SHOOT_RPM, robot.shooter.getTargetRpm(), EPS);
        press(op.gamepad2, g -> g.left_trigger = 1f);
        loop();
        assertEquals(0, robot.shooter.getTargetRpm(), EPS);
    }

    @Test
    public void debugToggleFlipsTheReadout() {
        initAndStart();
        loop();
        assertFalse(telemetry.contains("Loop: "));
        press(op.gamepad2, g -> g.back = true);
        loop();
        assertTrue(Teleop.DEBUG_TELEMETRY);
        assertTrue(telemetry.contains("Loop: "));
    }

    // ---- Init and hand-off ----

    @Test
    public void inheritsPoseAndAllianceFromAutonomous() {
        Pose left = new Pose(30, 40, Math.PI);
        PoseStorage.save(left, Alliance.RED, StartPosition.FACING_HIVE);
        initAndStart();
        assertEquals(30, follower.pose.x(), EPS);
        assertEquals(40, follower.pose.y(), EPS);
        assertEquals(Math.PI, follower.pose.heading(), EPS);
        assertTrue(telemetry.contains("RED (from auto)"));
        assertTrue("red starts on the audience-side CELL", telemetry.contains("tags 34-37"));
    }

    @Test
    public void initCardListsValuesTunedOnABench() {
        // fixthese R2-A5: a static bumped on a bench is what this OpMode runs with. The card says so.
        RobotTunables.snapshot();                  // what the bench's own init recorded
        Shooter.SHOOT_RPM = 3100;                  // what its dpad then did
        try {
            op.init();
            op.init_loop();
            assertTrue(telemetry.joined(), telemetry.contains("TUNED THIS SESSION"));
            assertTrue(telemetry.contains("Shooter.SHOOT_RPM = 3100.0 (default 3000.0)"));
        } finally {
            Shooter.SHOOT_RPM = 3000;
        }
    }

    @Test
    public void inheritsThePieceCountLeftByAuto() {
        PoseStorage.save(null, Alliance.RED, StartPosition.FACING_HIVE, 2);   // a cut auto: two left
        op.init();
        op.init_loop();
        assertEquals(2, robot.storage.count());
        assertTrue(robot.macros.isCountKnown());
        assertTrue(telemetry.joined(), telemetry.contains("Pieces: 2/4  (from auto)"));
    }

    @Test
    public void initLoopDpadPicksTheAllianceWhenAutoDidNotRun() {
        op.init();
        assertTrue(telemetry.contains("All hardware present") || telemetry.contains("MISSING HARDWARE"));
        press(op.gamepad1, g -> g.dpad_left = true);
        op.init_loop();
        assertTrue(telemetry.joined(), telemetry.contains("Alliance: RED"));
        assertTrue(telemetry.contains("tags 34-37"));
        assertTrue("help card shown", telemetry.contains("DRIVER (gamepad 1)"));
    }

    @Test
    public void stopReleasesEverythingWithoutHardware() {
        initAndStart();
        loop();
        op.stop();
        assertFalse(follower.mode == Follower.Mode.FOLLOW);
    }
}
