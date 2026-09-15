package org.firstinspires.ftc.teamcode.opmodes.auto;

import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertFalse;
import static org.junit.Assert.assertTrue;
import static org.junit.Assert.fail;

import com.pedropathing.ivy.Scheduler;
import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.commands.Macros;
import org.firstinspires.ftc.teamcode.game.FieldPoses;
import org.firstinspires.ftc.teamcode.opmodes.FakeTelemetry;
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
import org.firstinspires.ftc.teamcode.util.field.Alliance;
import org.firstinspires.ftc.teamcode.util.field.FieldConstants;
import org.firstinspires.ftc.teamcode.util.field.PoseStorage;
import org.firstinspires.ftc.teamcode.util.hardware.Hardware;
import org.firstinspires.ftc.teamcode.util.time.FakeClock;
import org.firstinspires.ftc.teamcode.util.time.MatchClock;
import org.junit.After;
import org.junit.Before;
import org.junit.Test;

import java.util.function.Consumer;

/** The real MainAuto through its SDK lifecycle on the JVM, with no follower and no sensors. */
public class MainAutoTest {
    private static final double EPS = 1e-9;
    private static final int MAX_LOOPS = 2000;

    private FakeClock clock;
    private FakePedroDrivetrain motors;
    private FakeDcMotorEx intakeMotor;
    private FakeDcMotorEx shooterMotor;
    private Robot robot;
    private FakeTelemetry telemetry;
    private TestAuto op;

    private final class TestAuto extends MainAuto {
        @Override
        protected Robot buildRobot() {
            return MainAutoTest.this.robot;
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
        MatchClock.AUTONOMOUS_MS = 30_000;
        clock = new FakeClock();
        motors = new FakePedroDrivetrain();
        intakeMotor = new FakeDcMotorEx();
        shooterMotor = new FakeDcMotorEx();
        robot = new Robot(
                new Drivetrain((PathFollower) null, clock),
                new OpenLoopDrive(motors, clock),
                new Intake(intakeMotor, clock),
                new Storage(new FakeDcMotorEx(), null, clock),
                new Transfer(new FakeDcMotorEx(), clock),
                new Shooter(shooterMotor, null, clock),
                new Limelight(null),
                new ColorSensor(null), new ColorSensor(null),
                new ColorSensor(null), new ColorSensor(null),
                clock);
        telemetry = new FakeTelemetry();
        op = new TestAuto();
        op.telemetry = telemetry;
        op.gamepad1 = new Gamepad();
        op.gamepad2 = new Gamepad();
        shooterMotor.measuredVelocity = Shooter.rpmToTicksPerSec(Shooter.SHOOT_RPM);
    }

    @After
    public void tearDown() {
        Scheduler.reset();
        Hardware.reset();
        PoseStorage.clear();
        MatchClock.AUTONOMOUS_MS = 30_000;
    }

    private static void press(Gamepad pad, Consumer<Gamepad> set) {
        Gamepad source = new Gamepad();
        set.accept(source);
        pad.copy(source);
    }

    private void loop() {
        op.loop();
        clock.advance(20);
    }

    private void loopUntil(java.util.function.BooleanSupplier done) {
        int loops = 0;
        while (!done.getAsBoolean()) {
            loop();
            if (++loops > MAX_LOOPS) fail("condition never met: " + robot.macros.getStatus());
        }
    }

    @Test
    public void initMenuPicksTheAllianceAndShowsThePlacementCard() {
        op.init();
        press(op.gamepad1, g -> g.dpad_left = true);
        op.init_loop();
        assertTrue(telemetry.contains("Setup: RED / Facing HIVE"));
        assertTrue(telemetry.contains("REAR (shooter) toward the up CELL"));
        assertTrue(telemetry.contains("Pre-loads: 4 POLLEN"));
        assertTrue(telemetry.contains("Sensors: entrance=none"));
        assertTrue(telemetry.contains("open loop (Pedro not tuned)"));
        press(op.gamepad1, g -> g.a = true);
        op.init_loop();
        assertTrue(telemetry.contains("LOCKED"));
    }

    @Test
    public void runsTheRoutineAndHandsTheAllianceToTeleop() {
        op.init();
        press(op.gamepad1, g -> g.dpad_left = true);
        op.init_loop();
        op.start();
        loopUntil(() -> telemetry.contains("Phase: done"));
        assertEquals(4, robot.macros.getShotsFired());
        assertEquals(Macros.Outcome.SUCCESS, robot.macros.getOutcome());
        assertEquals(0, robot.storage.count());
        assertTrue("it drove off the wall", motors.driveCalls >= 1);
        assertFalse(motors.moving);
        assertEquals(0, shooterMotor.commandedVelocity, EPS);
        assertEquals(0, intakeMotor.commandedVelocity, EPS);
        assertEquals("teleop inherits the alliance", Alliance.RED, PoseStorage.getAlliance());
        assertFalse("no pose without a localizer", PoseStorage.hasPose());
        op.stop();
    }

    @Test
    public void startSeedsTheStartPoseForTheSelectedAlliance() {
        FakePathFollower follower = new FakePathFollower();          // Pedro tuned: a follower exists
        robot = new Robot(
                new Drivetrain(follower, clock),
                new OpenLoopDrive((com.pedropathing.drivetrain.Drivetrain) null, clock),
                new Intake(intakeMotor, clock),
                new Storage(new FakeDcMotorEx(), null, clock),
                new Transfer(new FakeDcMotorEx(), clock),
                new Shooter(shooterMotor, null, clock),
                new Limelight(null),
                new ColorSensor(null), new ColorSensor(null),
                new ColorSensor(null), new ColorSensor(null),
                clock);
        op.init();
        press(op.gamepad1, g -> g.dpad_left = true);                 // RED
        op.init_loop();
        assertTrue("built a moment ago: still calibrating", telemetry.contains("Localizer calibrating"));
        assertTrue(telemetry.contains("Drive: Pedro follower"));
        op.start();
        com.pedropathing.math.Pose expected = FieldConstants.forAlliance(FieldPoses.BLUE_START_FACING_HIVE, Alliance.RED);
        assertEquals(expected.x(), follower.pose.x(), EPS);
        assertEquals(expected.y(), follower.pose.y(), EPS);
        assertEquals(expected.heading(), follower.pose.heading(), 1e-9);
        assertTrue("repeated once the Pinpoint has calibrated", robot.drivetrain.isPoseReapplyPending());
        loop();
        assertTrue("teleop inherits a real pose", PoseStorage.hasPose());
    }

    @Test
    public void handsThePieceCountToTeleop() {
        op.init();
        op.init_loop();
        op.start();
        loop();
        assertTrue(PoseStorage.hasPieceCount());
        assertEquals("four pre-loads on record from the first loop", 4, PoseStorage.getPieceCount());
        loopUntil(() -> robot.macros.getShotsFired() >= 2);
        assertEquals("kept current every loop", robot.storage.count(), PoseStorage.getPieceCount());
        op.stop();                                           // a cut auto
        assertEquals(robot.storage.count(), PoseStorage.getPieceCount());
        assertTrue(PoseStorage.getPieceCount() < 4);
    }

    @Test
    public void theBuzzerStopsARunningRoutine() {
        MatchClock.AUTONOMOUS_MS = 1000;             // a one-second "period"
        op.init();
        op.init_loop();
        op.start();
        loopUntil(() -> telemetry.contains("STOPPED at the buzzer"));
        assertEquals(Macros.Outcome.CANCELLED, robot.macros.getOutcome());
        assertFalse(motors.moving);
        assertEquals(0, shooterMotor.commandedVelocity, EPS);
        loop();
        assertEquals("nothing restarts after the buzzer", 0, shooterMotor.commandedVelocity, EPS);
    }
}
