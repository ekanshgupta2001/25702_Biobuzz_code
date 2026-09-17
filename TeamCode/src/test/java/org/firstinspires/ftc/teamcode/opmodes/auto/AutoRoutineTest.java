package org.firstinspires.ftc.teamcode.opmodes.auto;

import static org.junit.Assert.assertArrayEquals;
import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertFalse;
import static org.junit.Assert.assertTrue;
import static org.junit.Assert.fail;

import com.pedropathing.drivetrain.DrivePowers;
import com.pedropathing.follower.Follower;
import com.pedropathing.ivy.Command;
import com.pedropathing.ivy.Scheduler;

import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.commands.Macros;
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
import org.firstinspires.ftc.teamcode.util.hardware.Hardware;
import org.firstinspires.ftc.teamcode.util.time.FakeClock;
import org.junit.After;
import org.junit.Before;
import org.junit.Test;

/**
 * The hardcoded autonomous on the JVM: four pre-loads fired on timed pulses (no sensors, as at the
 * first event), then a timed leave, everything stopped, inside the 30 s period. The Pedro follower
 * is absent, exactly as it will be before tuning.
 */
public class AutoRoutineTest {
    private static final double EPS = 1e-9;
    private static final int MAX_TICKS = 2000;   // 40 s of fake time

    private FakeClock clock;
    private FakePedroDrivetrain motors;
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
        motors = new FakePedroDrivetrain();
        intakeMotor = new FakeDcMotorEx();
        storageMotor = new FakeDcMotorEx();
        transferMotor = new FakeDcMotorEx();
        shooterMotor = new FakeDcMotorEx();
        robot = new Robot(
                new Drivetrain((PathFollower) null, clock),     // not tuned: no follower
                new OpenLoopDrive(motors, clock),
                new Intake(intakeMotor, clock),
                new Storage(storageMotor, null, clock),
                new Transfer(transferMotor, clock),
                new Shooter(shooterMotor, null, clock),
                new Limelight(null),
                new ColorSensor(null), new ColorSensor(null),
                new ColorSensor(null), new ColorSensor(null),
                clock);
        resetTunables();
    }

    @After
    public void tearDown() {
        Scheduler.reset();
        Hardware.reset();
        resetTunables();
    }

    private static void resetTunables() {
        AutoRoutine.SHOOT_BUDGET_MS = 20000;
        AutoRoutine.SETTLE_MS = 250;
        AutoRoutine.LEAVE_DIRECTION = AutoRoutine.LeaveDirection.BACKWARD;
        AutoRoutine.LEAVE_POWER = 0.3;
        AutoRoutine.LEAVE_MS = 800;
        Macros.SHOOT_ALL_TIMEOUT_MS = 20000;
        Macros.SENSORLESS_FEED_PULSE_MS = 600;
        Storage.CAPACITY = 4;
        Storage.ADVANCE_TIMEOUT_MS = 2500;
    }

    private void tick() {
        robot.readSensors();
        Scheduler.execute();
        robot.writeActuators();
        clock.advance(20);
    }

    /** Runs the routine to completion; {@code eachLoop} runs before every tick. Returns fake ms elapsed. */
    private long run(Command routine, Runnable eachLoop) {
        long started = clock.nowMs();
        int ticks = 0;
        routine.schedule();
        while (Scheduler.isScheduled(routine)) {
            if (eachLoop != null) eachLoop.run();
            tick();
            if (++ticks > MAX_TICKS) fail("routine did not finish: " + robot.macros.getStatus());
        }
        return clock.nowMs() - started;
    }

    @Test
    public void shootsAllFourThenLeavesInsideThePeriod() {
        shooterMotor.measuredVelocity = Shooter.rpmToTicksPerSec(Shooter.SHOOT_RPM);
        AutoRoutine auto = new AutoRoutine(robot);
        final int[] leaveTicks = {0};
        final DrivePowers[] lastMoving = {null};
        long elapsed = run(auto.build(), () -> {
            if (motors.moving) {
                leaveTicks[0]++;
                lastMoving[0] = motors.lastPowers;
                assertEquals("the wheel is idled before the leave, not left to stopMechanisms", 0, robot.shooter.getTargetRpm(), EPS);
            }
        });

        assertEquals("done", auto.getPhase());
        assertEquals(4, robot.macros.getShotsFired());
        assertEquals(Macros.Outcome.SUCCESS, robot.macros.getOutcome());
        assertEquals(0, robot.storage.count());
        assertEquals("leave went backward at LEAVE_POWER", -0.3, lastMoving[0].forward(), EPS);
        assertEquals(0, lastMoving[0].strafe(), EPS);
        assertEquals(0, lastMoving[0].turn(), EPS);
        assertTrue("moved for about LEAVE_MS", Math.abs(leaveTicks[0] - AutoRoutine.LEAVE_MS / 20) <= 2);
        assertFalse("stopped at the end", motors.moving);
        assertTrue(motors.stopCalls >= 1);
        assertEquals(0, shooterMotor.commandedVelocity, EPS);
        assertEquals(0, transferMotor.commandedVelocity, EPS);
        assertEquals(0, intakeMotor.commandedVelocity, EPS);
        assertTrue("well inside the 30 s period: " + elapsed + " ms", elapsed < 15000);
    }

    @Test
    public void storageNeverPushesIntoAStoppedTransferWithoutSensors() {
        shooterMotor.measuredVelocity = Shooter.rpmToTicksPerSec(Shooter.SHOOT_RPM);
        final boolean[] pushed = {false};
        final int[] advancingTicks = {0};
        run(new AutoRoutine(robot).build(), () -> {
            if (robot.storage.getMode() == Storage.Mode.ADVANCING) {
                advancingTicks[0]++;
                if (robot.transfer.getMode() != Transfer.Mode.FEEDING) pushed[0] = true;
            }
        });
        assertFalse("fixthese B2: the side wheels only run while the transfer takes the piece", pushed[0]);
        assertTrue("four metered pulses, not four runs to the 2.5 s timeout: " + advancingTicks[0] * 20 + " ms",
                advancingTicks[0] * 20 <= 4 * Macros.SENSORLESS_FEED_PULSE_MS + 200);
    }

    @Test
    public void leavesEvenWhenTheShootingTimedOut() {
        Macros.SHOOT_ALL_TIMEOUT_MS = 200;           // the flywheel never comes up to speed
        AutoRoutine auto = new AutoRoutine(robot);
        run(auto.build(), null);
        assertEquals("done", auto.getPhase());
        assertEquals(Macros.Outcome.TIMED_OUT, robot.macros.getOutcome());
        assertEquals(0, robot.macros.getShotsFired());
        assertTrue("still drove off the wall", motors.driveCalls >= 1);
        assertFalse(motors.moving);
    }

    @Test
    public void theBudgetCutsARunawayShootAndStillLeaves() {
        AutoRoutine.SHOOT_BUDGET_MS = 300;           // tighter than the macro's own timeout
        AutoRoutine auto = new AutoRoutine(robot);
        run(auto.build(), null);
        assertEquals("done", auto.getPhase());
        assertEquals("the budget interrupted the macro", Macros.Outcome.CANCELLED, robot.macros.getOutcome());
        assertTrue(motors.driveCalls >= 1);
        assertFalse(motors.moving);
    }

    @Test
    public void leavesThroughTheFollowerWhenPedroIsTuned() {
        FakePathFollower follower = new FakePathFollower();
        robot = new Robot(
                new Drivetrain(follower, clock),
                new OpenLoopDrive((com.pedropathing.drivetrain.Drivetrain) null, clock),   // Robot never fits both
                new Intake(intakeMotor, clock),
                new Storage(storageMotor, null, clock),
                new Transfer(transferMotor, clock),
                new Shooter(shooterMotor, null, clock),
                new Limelight(null),
                new ColorSensor(null), new ColorSensor(null),
                new ColorSensor(null), new ColorSensor(null),
                clock);
        shooterMotor.measuredVelocity = Shooter.rpmToTicksPerSec(Shooter.SHOOT_RPM);
        AutoRoutine auto = new AutoRoutine(robot);
        final double[] leaveForward = {0};
        run(auto.build(), () -> {
            if (follower.mode == Follower.Mode.MANUAL && follower.lastForward != 0) {
                leaveForward[0] = follower.lastForward;
            }
        });
        assertEquals("done", auto.getPhase());
        assertEquals(4, robot.macros.getShotsFired());
        assertEquals("LEAVE went through the follower at LEAVE_POWER", -0.3, leaveForward[0], EPS);
        assertEquals("handed back with zero power", 0, follower.lastForward, EPS);
        assertEquals(Follower.Mode.MANUAL, follower.mode);
        assertFalse("no open-loop layer on a tuned robot", robot.openLoopDrive.isAvailable());
    }

    @Test
    public void ownsEveryMechanismForTheWholeRun() {
        Command routine = new AutoRoutine(robot).build();
        assertTrue(routine.requirements().contains(robot.intake));
        assertTrue(routine.requirements().contains(robot.storage));
        assertTrue(routine.requirements().contains(robot.transfer));
        assertTrue(routine.requirements().contains(robot.shooter));
        assertTrue(routine.requirements().contains(robot.openLoopDrive));
        assertTrue(routine.requirements().contains(robot.drivetrain));
    }

    @Test
    public void leaveDirectionsMapToPedroPowers() {
        assertArrayEquals(new double[] {0.3, 0, 0}, AutoRoutine.leavePowers(AutoRoutine.LeaveDirection.FORWARD, 0.3), EPS);
        assertArrayEquals(new double[] {-0.3, 0, 0}, AutoRoutine.leavePowers(AutoRoutine.LeaveDirection.BACKWARD, 0.3), EPS);
        assertArrayEquals(new double[] {0, 0.3, 0}, AutoRoutine.leavePowers(AutoRoutine.LeaveDirection.LEFT, 0.3), EPS);
        assertArrayEquals(new double[] {0, -0.3, 0}, AutoRoutine.leavePowers(AutoRoutine.LeaveDirection.RIGHT, 0.3), EPS);
    }
}
