package org.firstinspires.ftc.teamcode.opmodes.test;

import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertFalse;
import static org.junit.Assert.assertTrue;
import static org.junit.Assert.fail;

import com.pedropathing.ivy.Scheduler;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.commands.Macros;
import org.firstinspires.ftc.teamcode.opmodes.FakeTelemetry;
import org.firstinspires.ftc.teamcode.subsystems.ColorSensor;
import org.firstinspires.ftc.teamcode.subsystems.Drivetrain;
import org.firstinspires.ftc.teamcode.subsystems.FakeDcMotorEx;
import org.firstinspires.ftc.teamcode.subsystems.FakePedroDrivetrain;
import org.firstinspires.ftc.teamcode.subsystems.Intake;
import org.firstinspires.ftc.teamcode.subsystems.Limelight;
import org.firstinspires.ftc.teamcode.subsystems.OpenLoopDrive;
import org.firstinspires.ftc.teamcode.subsystems.PathFollower;
import org.firstinspires.ftc.teamcode.subsystems.Shooter;
import org.firstinspires.ftc.teamcode.subsystems.Storage;
import org.firstinspires.ftc.teamcode.subsystems.Transfer;
import org.firstinspires.ftc.teamcode.util.diagnostics.Tunables;
import org.firstinspires.ftc.teamcode.util.hardware.Hardware;
import org.firstinspires.ftc.teamcode.util.time.FakeClock;
import org.junit.After;
import org.junit.Before;
import org.junit.Test;

import java.util.function.Consumer;

/**
 * Every bench OpMode through its real SDK lifecycle on the JVM: the first-event robot (no follower,
 * open-loop motors, no sensors, no camera) built from fakes through {@code buildRobot()}, real
 * gamepads driven by {@code copy()}, one targeted assertion per bench. These prove the benches run
 * and reach the mechanism they name; the numbers they exist to measure need the robot.
 */
public class BenchOpModesTest {
    private static final double EPS = 1e-6;
    private static final int MAX_LOOPS = 2000;

    private FakeClock clock;
    private FakePedroDrivetrain motors;
    private FakeDcMotorEx intakeMotor;
    private FakeDcMotorEx storageMotor;
    private FakeDcMotorEx transferMotor;
    private FakeDcMotorEx shooterMotor;
    private Robot robot;
    private FakeTelemetry telemetry;

    @Before
    public void setUp() {
        Scheduler.reset();
        Hardware.reset();
        Tunables.resetForTests();
        clock = new FakeClock();
        motors = new FakePedroDrivetrain();
        intakeMotor = new FakeDcMotorEx();
        storageMotor = new FakeDcMotorEx();
        transferMotor = new FakeDcMotorEx();
        shooterMotor = new FakeDcMotorEx();
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
        telemetry = new FakeTelemetry();
        Macros.SENSORLESS_FEED_PULSE_MS = 600;
        Shooter.SHOOT_RPM = 3000;
        Shooter.CUSTOM_PIDF = false;
        Shooter.SECOND_MOTOR_DIRECTION = DcMotorSimple.Direction.FORWARD;
        Intake.ANTI_JAM_ENABLED = true;
    }

    @After
    public void tearDown() {
        Scheduler.reset();
        Hardware.reset();
        Tunables.resetForTests();
        Macros.SENSORLESS_FEED_PULSE_MS = 600;
        Shooter.SHOOT_RPM = 3000;
        Shooter.CUSTOM_PIDF = false;
        Shooter.SECOND_MOTOR_DIRECTION = DcMotorSimple.Direction.FORWARD;
        Intake.ANTI_JAM_ENABLED = true;
    }

    /** Wires the fixture robot and telemetry into a bench and runs init / init_loop / start. */
    private <T extends BenchOpMode> T started(T op) {
        op.telemetry = telemetry;
        op.gamepad1 = new Gamepad();
        op.gamepad2 = new Gamepad();
        op.init();
        op.init_loop();
        op.start();
        return op;
    }

    private static void press(Gamepad pad, Consumer<Gamepad> set) {
        Gamepad source = new Gamepad();
        set.accept(source);
        pad.copy(source);
    }

    private void loop(BenchOpMode op) {
        op.loop();
        clock.advance(20);
    }

    // Each bench with the fixture's robot through the buildRobot() seam.
    private final class TestIntakeBench extends IntakeBench {
        @Override protected Robot buildRobot() { return BenchOpModesTest.this.robot; }
    }
    private final class TestStorageBench extends StorageBench {
        @Override protected Robot buildRobot() { return BenchOpModesTest.this.robot; }
    }
    private final class TestTransferBench extends TransferBench {
        @Override protected Robot buildRobot() { return BenchOpModesTest.this.robot; }
    }
    private final class TestShooterBench extends ShooterBench {
        @Override protected Robot buildRobot() { return BenchOpModesTest.this.robot; }
    }
    private final class TestColorSensorBench extends ColorSensorBench {
        @Override protected Robot buildRobot() { return BenchOpModesTest.this.robot; }
    }
    private final class TestLimelightBench extends LimelightBench {
        @Override protected Robot buildRobot() { return BenchOpModesTest.this.robot; }
    }
    private final class TestSelfTest extends SelfTest {
        @Override protected Robot buildRobot() { return BenchOpModesTest.this.robot; }
    }

    @Test
    public void intakeBenchRunsTheRollerWhileTheBumperIsHeld() {
        IntakeBench op = started(new TestIntakeBench());
        assertTrue(telemetry.contains("BENCH: INTAKE"));
        press(op.gamepad1, g -> g.right_bumper = true);
        loop(op);
        assertEquals(Intake.INTAKE_TICKS_PER_SEC, intakeMotor.commandedVelocity, EPS);
        press(op.gamepad1, g -> g.left_bumper = true);
        loop(op);
        assertEquals(Intake.OUTTAKE_TICKS_PER_SEC, intakeMotor.commandedVelocity, EPS);
        press(op.gamepad1, g -> { });
        loop(op);
        assertEquals("release stops it", 0, intakeMotor.commandedVelocity, EPS);
        press(op.gamepad1, g -> g.y = true);
        loop(op);
        assertFalse("Y toggles anti-jam", Intake.ANTI_JAM_ENABLED);
        assertTrue(telemetry.contains("Anti-jam: off"));
    }

    @Test
    public void storageBenchPulsesTheTransportForTheMacroPulseLength() {
        StorageBench op = started(new TestStorageBench());
        press(op.gamepad1, g -> g.a = true);
        loop(op);
        assertEquals(Storage.ADVANCE_TICKS_PER_SEC, storageMotor.commandedVelocity, EPS);
        press(op.gamepad1, g -> { });
        loop(op);
        assertEquals(0, storageMotor.commandedVelocity, EPS);

        press(op.gamepad1, g -> g.right_bumper = true);                 // pulse + 50 ms
        loop(op);
        assertEquals(650, Macros.SENSORLESS_FEED_PULSE_MS);
        press(op.gamepad1, g -> g.x = true);                            // one pulse
        int advancing = 0;
        for (int i = 0; i < 60; i++) {
            loop(op);
            if (i == 0) press(op.gamepad1, g -> { });
            if (storageMotor.commandedVelocity > 0) advancing++;
        }
        assertTrue("pulsed for about " + Macros.SENSORLESS_FEED_PULSE_MS + " ms: " + advancing * 20,
                Math.abs(advancing * 20 - Macros.SENSORLESS_FEED_PULSE_MS) <= 40);
        assertEquals("and stopped", 0, storageMotor.commandedVelocity, EPS);

        press(op.gamepad1, g -> g.dpad_up = true);
        loop(op);
        assertEquals(1, robot.storage.count());
        assertTrue(telemetry.contains("Count: 1 / 4"));
    }

    @Test
    public void transferBenchLiftsWhileHeld() {
        TransferBench op = started(new TestTransferBench());
        press(op.gamepad1, g -> g.a = true);
        loop(op);
        assertEquals(Transfer.LIFT_TICKS_PER_SEC, transferMotor.commandedVelocity, EPS);
        press(op.gamepad1, g -> g.b = true);
        loop(op);
        assertEquals(Transfer.FEED_TICKS_PER_SEC, transferMotor.commandedVelocity, EPS);
        press(op.gamepad1, g -> { });
        loop(op);
        assertEquals(0, transferMotor.commandedVelocity, EPS);
        assertTrue(telemetry.contains("NO (timed pulses)"));
    }

    @Test
    public void shooterBenchSpinsOnAAndFiresOnY() {
        ShooterBench op = started(new TestShooterBench());
        press(op.gamepad1, g -> g.dpad_up = true);
        loop(op);
        assertEquals(3100, Shooter.SHOOT_RPM, EPS);
        press(op.gamepad1, g -> g.a = true);
        loop(op);
        assertEquals(Shooter.rpmToTicksPerSec(3100), shooterMotor.commandedVelocity, EPS);

        shooterMotor.measuredVelocity = Shooter.rpmToTicksPerSec(3100);
        press(op.gamepad1, g -> { });
        for (int i = 0; i <= Shooter.AT_SPEED_LOOPS; i++) loop(op);
        assertTrue(robot.shooter.atSpeed());
        assertTrue(telemetry.joined(), telemetry.contains("Spin-up:"));

        press(op.gamepad1, g -> g.y = true);
        loop(op);
        assertEquals("the real storage + transfer pulse", Transfer.FEED_TICKS_PER_SEC, transferMotor.commandedVelocity, EPS);
        assertEquals(Storage.ADVANCE_TICKS_PER_SEC, storageMotor.commandedVelocity, EPS);
        press(op.gamepad1, g -> { });
        for (int i = 0; i < 40; i++) loop(op);
        assertEquals(0, transferMotor.commandedVelocity, EPS);
        assertTrue("recovery recorded: " + op.getLastRecoveryMs(),
                op.getLastRecoveryMs() > Shooter.SHOT_RECOVERY_MIN_MS && op.getLastRecoveryMs() < 400);
        assertEquals("the fake never dipped", 3100, op.getLastDipRpm(), EPS);

        press(op.gamepad1, g -> g.a = true);
        loop(op);
        assertEquals("A again stops it", 0, shooterMotor.commandedVelocity, EPS);
    }

    @Test
    public void shooterBenchFlipsTheSecondMotorLiveAndTogglesPidfBothWays() {
        // fixthese R2-A9 / R2-A6.
        FakeDcMotorEx second = new FakeDcMotorEx();
        robot = new Robot(
                new Drivetrain((PathFollower) null, clock),
                new OpenLoopDrive(motors, clock),
                new Intake(intakeMotor, clock),
                new Storage(storageMotor, null, clock),
                new Transfer(transferMotor, clock),
                new Shooter(shooterMotor, second, clock),
                new Limelight(null),
                new ColorSensor(null), new ColorSensor(null),
                new ColorSensor(null), new ColorSensor(null),
                clock);
        ShooterBench op = started(new TestShooterBench());
        press(op.gamepad1, g -> g.dpad_right = true);
        loop(op);
        assertEquals(DcMotorSimple.Direction.REVERSE, Shooter.SECOND_MOTOR_DIRECTION);
        assertEquals("applied to the motor the same loop", DcMotorSimple.Direction.REVERSE, second.direction);
        assertEquals(DcMotorSimple.Direction.FORWARD, shooterMotor.direction);
        assertTrue(telemetry.joined(), telemetry.contains("Second motor: REVERSE"));

        double sdkP = shooterMotor.pidf.p;
        press(op.gamepad1, g -> g.x = true);
        loop(op);
        assertTrue(Shooter.CUSTOM_PIDF);
        assertEquals(Shooter.PIDF_P, shooterMotor.pidf.p, EPS);
        press(op.gamepad1, g -> { });                 // release, so the next press is a new edge
        loop(op);
        press(op.gamepad1, g -> g.x = true);
        loop(op);
        assertFalse(Shooter.CUSTOM_PIDF);
        assertEquals("off really puts the SDK's back", sdkP, shooterMotor.pidf.p, EPS);
        assertTrue(telemetry.contains("SDK default  hub holds P 10.000"));
    }

    @Test
    public void benchBackRestoresTheTunablesAndTheCardListsThem() {
        // fixthese R2-A5: a bench edit lives for the app process; the card says so and BACK undoes it.
        ShooterBench op = started(new TestShooterBench());
        press(op.gamepad1, g -> g.dpad_up = true);
        loop(op);
        assertEquals(3100, Shooter.SHOOT_RPM, EPS);
        assertTrue(telemetry.joined(), telemetry.contains("Shooter.SHOOT_RPM = 3100.0 (default 3000.0)"));
        press(op.gamepad1, g -> g.back = true);
        loop(op);
        assertEquals("BACK restores the compiled default", 3000, Shooter.SHOOT_RPM, EPS);
        assertTrue(Tunables.changed().isEmpty());
    }

    @Test
    public void colorSensorBenchReportsAMissingPoint() {
        ColorSensorBench op = started(new TestColorSensorBench());
        loop(op);
        assertTrue(telemetry.contains("sensor_storage_entrance"));
        assertTrue(telemetry.contains("MISSING from the configuration"));
        press(op.gamepad1, g -> g.dpad_right = true);
        loop(op);
        assertTrue(telemetry.contains("sensor_storage_full"));
    }

    @Test
    public void limelightBenchReportsAMissingCamera() {
        LimelightBench op = started(new TestLimelightBench());
        loop(op);
        assertTrue(telemetry.joined(), telemetry.contains("MISSING from the configuration"));
    }

    @Test
    public void selfTestPassesTheMotorsAndSkipsWhatIsNotFitted() {
        intakeMotor.measuredVelocity = 500;
        storageMotor.measuredVelocity = 800;
        transferMotor.measuredVelocity = -400;          // wired backwards
        shooterMotor.measuredVelocity = Shooter.rpmToTicksPerSec(500);
        SelfTest op = started(new TestSelfTest());
        assertTrue(telemetry.contains("MECHANISMS WILL SPIN"));
        int loops = 0;
        while (!op.isDone()) {
            loop(op);
            if (++loops > MAX_LOOPS) fail("self test never finished: " + op.getResults());
        }
        loop(op);
        assertTrue(op.getResults().get("intake").startsWith("PASS"));
        assertTrue(op.getResults().get("storage").startsWith("PASS"));
        assertTrue(op.getResults().get("transfer"), op.getResults().get("transfer").contains("BACKWARD"));
        assertTrue(op.getResults().get("shooter").startsWith("PASS"));
        assertTrue(op.getResults().get("battery").startsWith("SKIP"));
        assertTrue(op.getResults().get("sensor entrance").startsWith("SKIP"));
        assertTrue(op.getResults().get("limelight").startsWith("SKIP"));
        assertTrue(op.getResults().get("drive").contains("open loop"));
        assertTrue(op.getResults().get("localizer").startsWith("SKIP"));
        assertEquals("everything stopped at the end", 0, intakeMotor.commandedVelocity, EPS);
        assertEquals(0, shooterMotor.commandedVelocity, EPS);
    }
}
