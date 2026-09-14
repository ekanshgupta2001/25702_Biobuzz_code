package org.firstinspires.ftc.teamcode.subsystems.templates;

import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertFalse;
import static org.junit.Assert.assertTrue;

import com.pedropathing.ivy.Command;
import com.pedropathing.ivy.Scheduler;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.teamcode.subsystems.FakeDcMotorEx;
import org.firstinspires.ftc.teamcode.util.time.FakeClock;
import org.junit.After;
import org.junit.Before;
import org.junit.Test;

public class PositionalMotorTest {
    private static final double EPS = 1e-9;

    private enum Level { DOWN, HIGH }

    private FakeDcMotorEx motor;
    private FakeClock clock;
    private PositionalMotor<Level> lift;

    @Before
    public void setUp() {
        Scheduler.reset();
        motor = new FakeDcMotorEx();
        clock = new FakeClock();
        lift = new PositionalMotor<>(motor, Level.class, DcMotorSimple.Direction.FORWARD, clock)
                .preset(Level.DOWN, 0)
                .preset(Level.HIGH, 1000)
                .limits(0, 1100);
        PositionalMotor.MIN_MOVE_MS = 60;
        PositionalMotor.MOVE_TIMEOUT_MS = 3000;
    }

    @After
    public void tearDown() {
        Scheduler.reset();
    }

    @Test
    public void constructorConfiguresWithoutApplyingPower() {
        assertEquals(DcMotor.RunMode.RUN_TO_POSITION, motor.mode);
        assertEquals(0, motor.targetPosition);
        assertEquals("no power during init", 0, motor.powerWrites);
        assertTrue(lift.isAvailable());
    }

    @Test
    public void softLimitsClampTheTarget() {
        lift.goToTicks(5000);
        assertEquals(1100, motor.targetPosition);
        assertEquals(1100, lift.getTarget());
        lift.goToTicks(-40);
        assertEquals(0, motor.targetPosition);
    }

    @Test
    public void aMoveCannotFinishOnItsFirstTick() {
        motor.currentPosition = 1000;               // already there
        Command move = lift.goTo(Level.HIGH);
        move.schedule();
        assertEquals(PositionalMotor.DEFAULT_POWER, motor.commandedPower, EPS);
        Scheduler.execute();
        assertTrue(Scheduler.isScheduled(move));

        clock.advance(PositionalMotor.MIN_MOVE_MS);
        Scheduler.execute();
        assertFalse(Scheduler.isScheduled(move));
        assertEquals("power cut on arrival", 0, motor.commandedPower, EPS);
    }

    @Test
    public void aMoveFinishesOnArrival() {
        Command move = lift.goTo(Level.HIGH);
        move.schedule();
        clock.advance(200);
        Scheduler.execute();
        assertTrue("not there yet", Scheduler.isScheduled(move));

        motor.currentPosition = 1000 - PositionalMotor.DEFAULT_TOLERANCE_TICKS;
        Scheduler.execute();
        assertFalse(Scheduler.isScheduled(move));
        assertEquals(Level.HIGH, lift.getState());
        assertTrue(lift.atState());
    }

    @Test
    public void aJammedMoveGivesUpAfterTheTimeout() {
        Command move = lift.goTo(Level.HIGH);
        move.schedule();
        clock.advance(PositionalMotor.MOVE_TIMEOUT_MS - 1);
        Scheduler.execute();
        assertTrue(Scheduler.isScheduled(move));
        clock.advance(1);
        Scheduler.execute();
        assertFalse("must release the resource", Scheduler.isScheduled(move));
        assertEquals(0, motor.commandedPower, EPS);
        assertFalse("and be honest that it never arrived", lift.atState());
    }

    @Test
    public void anInterruptedMoveCutsPower() {
        Command move = lift.goTo(Level.HIGH);
        move.schedule();
        assertTrue(motor.commandedPower > 0);
        Scheduler.cancel(move);
        assertEquals(0, motor.commandedPower, EPS);
    }

    @Test
    public void aStateWithoutAPresetIsIgnoredRatherThanThrowing() {
        PositionalMotor<Level> partial = new PositionalMotor<>(
                new FakeDcMotorEx(), Level.class, DcMotorSimple.Direction.FORWARD, clock)
                .preset(Level.DOWN, 0);
        partial.goToState(Level.HIGH);
        assertEquals(0, partial.getTarget());
    }
}
