package org.firstinspires.ftc.teamcode.subsystems;

import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertFalse;
import static org.junit.Assert.assertTrue;

import com.pedropathing.ivy.Command;
import com.pedropathing.ivy.Scheduler;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.teamcode.util.time.FakeClock;
import org.junit.After;
import org.junit.Before;
import org.junit.Test;

/** Flywheel spin-up, at-speed, and ownership semantics against a fake motor. */
public class ShooterTest {
    private static final double EPS = 1e-6;

    private FakeDcMotorEx motor;
    private FakeClock clock;
    private Shooter shooter;

    @Before
    public void setUp() {
        Scheduler.reset();
        motor = new FakeDcMotorEx();
        clock = new FakeClock();
        shooter = new Shooter(motor, null, clock);
        Shooter.TICKS_PER_REV = 28;
        Shooter.SHOOT_RPM = 3000;
        Shooter.IDLE_RPM = 0;
        Shooter.AT_SPEED_TOLERANCE_RPM = 100;
        Shooter.SPINUP_TIMEOUT_MS = 3000;
    }

    @After
    public void tearDown() {
        Scheduler.reset();
    }

    private void tick() {
        Scheduler.execute();
        shooter.update();
        clock.advance(20);
    }

    @Test
    public void rpmConversionRoundTrips() {
        assertEquals(1400, Shooter.rpmToTicksPerSec(3000), EPS);
        assertEquals(3000, Shooter.ticksPerSecToRpm(1400), EPS);
    }

    @Test
    public void spinUpFinishesOnceAtSpeedAndKeepsSpinningWhileOwned() {
        Command spin = shooter.spinUpCommand();
        spin.schedule();
        tick();
        assertEquals(Shooter.rpmToTicksPerSec(Shooter.SHOOT_RPM), motor.commandedVelocity, EPS);
        assertEquals(Shooter.Mode.SPINNING_UP, shooter.getMode());
        assertTrue(Scheduler.isScheduled(spin));

        motor.measuredVelocity = Shooter.rpmToTicksPerSec(Shooter.SHOOT_RPM - 50);
        tick();
        assertFalse(Scheduler.isScheduled(spin));
        assertEquals(Shooter.Mode.READY, shooter.getMode());
        assertEquals("a natural end does not touch the target",
                Shooter.SHOOT_RPM, shooter.getTargetRpm(), EPS);
    }

    @Test
    public void spinUpGivesUpAfterTheTimeout() {
        Command spin = shooter.spinUpCommand();
        spin.schedule();
        clock.advance(Shooter.SPINUP_TIMEOUT_MS);
        Scheduler.execute();
        assertFalse(Scheduler.isScheduled(spin));
    }

    @Test
    public void holdSpeedIdlesTheWheelWhenInterruptedAndTheDefaultResumes() {
        shooter.defaultIdleCommand().schedule();
        tick();
        assertEquals(0, motor.commandedVelocity, EPS);

        Command hold = shooter.holdSpeedCommand();
        hold.schedule();
        tick();
        assertEquals(Shooter.rpmToTicksPerSec(Shooter.SHOOT_RPM), motor.commandedVelocity, EPS);

        Scheduler.cancel(hold);
        tick();
        assertEquals("idle again once nothing owns the shooter", 0, motor.commandedVelocity, EPS);
        assertEquals(Shooter.Mode.IDLE, shooter.getMode());
    }

    @Test
    public void atSpeedIsFalseForAZeroTargetEvenIfTheWheelIsTurning() {
        motor.measuredVelocity = 500;
        assertFalse(shooter.atSpeed());
        assertEquals(Shooter.Mode.IDLE, shooter.getMode());
    }

    @Test
    public void unavailableShooterFinishesSpinUpAtOnce() {
        Shooter none = new Shooter((DcMotorEx) null, null, clock);
        assertFalse(none.isAvailable());
        Command spin = none.spinUpCommand();
        spin.schedule();
        Scheduler.execute();
        assertFalse(Scheduler.isScheduled(spin));
        assertEquals(0, none.getRpm(), EPS);
    }
}
