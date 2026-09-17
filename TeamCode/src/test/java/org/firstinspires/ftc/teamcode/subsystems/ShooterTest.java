package org.firstinspires.ftc.teamcode.subsystems;

import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertFalse;
import static org.junit.Assert.assertTrue;

import com.pedropathing.ivy.Command;
import com.pedropathing.ivy.Scheduler;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;

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
        Shooter.AT_SPEED_TOLERANCE_RPM = 150;
        Shooter.AT_SPEED_LOOPS = 5;
        Shooter.SPINUP_TIMEOUT_MS = 3000;
        Shooter.CUSTOM_PIDF = false;
        Shooter.SECOND_MOTOR_DIRECTION = DcMotorSimple.Direction.FORWARD;
    }

    @After
    public void tearDown() {
        Scheduler.reset();
        Shooter.CUSTOM_PIDF = false;
        Shooter.SECOND_MOTOR_DIRECTION = DcMotorSimple.Direction.FORWARD;
    }

    private void ticks(int n) {
        for (int i = 0; i < n; i++) tick();
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
    public void waitForSpeedFinishesOnceTheBandIsHeldWhileAHoldOwnsTheWheel() {
        Command hold = shooter.holdSpeedCommand();
        Command wait = shooter.waitForSpeedCommand();
        hold.schedule();
        wait.schedule();
        tick();
        assertEquals(Shooter.rpmToTicksPerSec(Shooter.SHOOT_RPM), motor.commandedVelocity, EPS);
        assertEquals(Shooter.Mode.SPINNING_UP, shooter.getMode());
        assertTrue(Scheduler.isScheduled(wait));

        motor.measuredVelocity = Shooter.rpmToTicksPerSec(Shooter.SHOOT_RPM - 50);
        ticks(Shooter.AT_SPEED_LOOPS);                    // in band, held for the latch
        tick();
        assertFalse(Scheduler.isScheduled(wait));
        assertEquals(Shooter.Mode.READY, shooter.getMode());
        assertTrue("the wait requires nothing: the hold keeps the wheel", Scheduler.isScheduled(hold));
        assertEquals(Shooter.SHOOT_RPM, shooter.getTargetRpm(), EPS);
    }

    @Test
    public void waitForSpeedGivesUpAfterTheTimeout() {
        Command wait = shooter.waitForSpeedCommand();
        wait.schedule();
        clock.advance(Shooter.SPINUP_TIMEOUT_MS);
        Scheduler.execute();
        assertFalse(Scheduler.isScheduled(wait));
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
        ticks(Shooter.AT_SPEED_LOOPS + 1);
        assertFalse(shooter.atSpeed());
        assertEquals(Shooter.Mode.IDLE, shooter.getMode());
    }

    @Test
    public void atSpeedNeedsConsecutiveInBandLoops() {
        // fixthese C3: one noisy sample through the band must not release a shot.
        shooter.spinUp();
        motor.measuredVelocity = Shooter.rpmToTicksPerSec(Shooter.SHOOT_RPM);
        ticks(Shooter.AT_SPEED_LOOPS - 1);
        assertTrue(shooter.inBandNow());
        assertFalse("four in a row is not yet ready", shooter.atSpeed());
        tick();
        assertTrue("five in a row is", shooter.atSpeed());

        motor.measuredVelocity = Shooter.rpmToTicksPerSec(Shooter.SHOOT_RPM - 2 * Shooter.AT_SPEED_TOLERANCE_RPM);
        tick();
        assertFalse("one sample out of band resets the latch", shooter.atSpeed());
        motor.measuredVelocity = Shooter.rpmToTicksPerSec(Shooter.SHOOT_RPM);
        ticks(Shooter.AT_SPEED_LOOPS - 1);
        assertFalse(shooter.atSpeed());
    }

    @Test
    public void customPidfIsWrittenOnlyWhenEnabled() {
        assertEquals("defaults untouched", null, motor.lastVelocityPidf);
        Shooter.CUSTOM_PIDF = true;
        FakeDcMotorEx tuned = new FakeDcMotorEx();
        new Shooter(tuned, null, clock);
        assertEquals(Shooter.PIDF_P, tuned.lastVelocityPidf[0], EPS);
        assertEquals(Shooter.PIDF_I, tuned.lastVelocityPidf[1], EPS);
        assertEquals(Shooter.PIDF_D, tuned.lastVelocityPidf[2], EPS);
        assertEquals("F = 32767 / max ticks per second", 32767.0 / Shooter.MAX_TICKS_PER_SEC, tuned.lastVelocityPidf[3], EPS);
    }

    @Test
    public void turningCustomPidfOffRestoresTheSdkCoefficients() {
        // fixthese R2-A6: the bench's X used to flip the flag and leave the custom gains in the hub
        // while the card said "SDK default".
        FakeDcMotorEx m = new FakeDcMotorEx();
        PIDFCoefficients sdk = m.pidf;
        Shooter s = new Shooter(m, null, clock);
        assertTrue("off at construction: the hub keeps its own", sdk == m.pidf);

        Shooter.CUSTOM_PIDF = true;
        s.applyPidf();
        assertEquals(Shooter.PIDF_P, m.pidf.p, EPS);
        assertEquals(Shooter.PIDF_F, m.pidf.f, EPS);

        Shooter.CUSTOM_PIDF = false;
        s.applyPidf();
        assertEquals("the SDK's own coefficients are back", sdk.p, m.pidf.p, EPS);
        assertEquals(sdk.i, m.pidf.i, EPS);
        assertEquals(sdk.f, m.pidf.f, EPS);
        assertEquals(sdk.p, s.readFlywheelPidf().p, EPS);
    }

    @Test
    public void secondFlywheelDirectionFollowsTheStaticAtRuntime() {
        // fixthese R2-A9: the direction used to be read once in the constructor, so a bench flip
        // did nothing until re-INIT.
        FakeDcMotorEx first = new FakeDcMotorEx();
        FakeDcMotorEx second = new FakeDcMotorEx();
        Shooter pair = new Shooter(first, second, clock);
        assertTrue(pair.hasSecondMotor());
        pair.update();
        assertEquals(DcMotorSimple.Direction.FORWARD, second.direction);
        Shooter.SECOND_MOTOR_DIRECTION = DcMotorSimple.Direction.REVERSE;
        pair.update();
        assertEquals("applied on the next loop", DcMotorSimple.Direction.REVERSE, second.direction);
        assertEquals("the first is never touched", DcMotorSimple.Direction.FORWARD, first.direction);
        second.measuredVelocity = -700;
        assertEquals(-700, pair.getSecondVelocityTicksPerSec(), EPS);
    }

    @Test
    public void secondFlywheelTakesItsOwnDirection() {
        Shooter.SECOND_MOTOR_DIRECTION = DcMotorSimple.Direction.REVERSE;
        FakeDcMotorEx first = new FakeDcMotorEx();
        FakeDcMotorEx second = new FakeDcMotorEx();
        Shooter pair = new Shooter(first, second, clock);
        assertEquals(DcMotorSimple.Direction.FORWARD, first.direction);
        assertEquals("an opposed pair needs one reversed", DcMotorSimple.Direction.REVERSE, second.direction);
        pair.spinUp();
        pair.update();
        assertEquals("both get the same target", first.commandedVelocity, second.commandedVelocity, EPS);
    }

    @Test
    public void unavailableShooterFinishesTheWaitAtOnce() {
        Shooter none = new Shooter((DcMotorEx) null, null, clock);
        assertFalse(none.isAvailable());
        Command wait = none.waitForSpeedCommand();
        wait.schedule();
        Scheduler.execute();
        assertFalse(Scheduler.isScheduled(wait));
        assertEquals(0, none.getRpm(), EPS);
    }
}
