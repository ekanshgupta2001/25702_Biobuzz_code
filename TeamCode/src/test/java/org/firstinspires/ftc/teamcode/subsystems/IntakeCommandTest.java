package org.firstinspires.ftc.teamcode.subsystems;

import static com.pedropathing.ivy.commands.Commands.instant;
import static com.pedropathing.ivy.commands.Commands.waitMs;
import static com.pedropathing.ivy.groups.Groups.sequential;
import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertFalse;
import static org.junit.Assert.assertTrue;

import com.pedropathing.ivy.Command;
import com.pedropathing.ivy.Scheduler;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.teamcode.util.time.FakeClock;
import org.junit.After;
import org.junit.Before;
import org.junit.Test;

/**
 * The intake's commands, run through Ivy's real {@link Scheduler} against a fake motor. Each test
 * steps the loop the way {@code MatchOpMode} does: scheduler, then the actuator write.
 */
public class IntakeCommandTest {
    private static final double EPS = 1e-9;

    private FakeDcMotorEx motor;
    private FakeClock clock;
    private Intake intake;

    @Before
    public void setUp() {
        Scheduler.reset();
        motor = new FakeDcMotorEx();
        clock = new FakeClock();
        intake = new Intake(motor, clock);
        Intake.ANTI_JAM_ENABLED = true;
        Intake.REJECT_ENABLED = false;
        Intake.REJECT_MS = 400;
    }

    @After
    public void tearDown() {
        Scheduler.reset();
        Intake.REJECT_ENABLED = false;
    }

    private void tick() {
        Scheduler.execute();
        intake.update();
        clock.advance(20);
    }

    @Test
    public void constructorConfiguresTheMotorWithoutMovingIt() {
        assertEquals(DcMotor.RunMode.RUN_USING_ENCODER, motor.mode);
        assertEquals(DcMotor.ZeroPowerBehavior.FLOAT, motor.zeroPowerBehavior);
        assertEquals(0, motor.velocityWrites);
        assertTrue(intake.isAvailable());
    }

    @Test
    public void currentIsReadOnceALoop() {
        // fixthese R2-A1: the jam detector, the match log and the telemetry each used to read the
        // motor current themselves, and a current read is not in the bulk cache.
        motor.currentAmps = 2.5;
        intake.intakeCommand().schedule();
        tick();
        tick();
        assertEquals(2, motor.currentReads);
        assertEquals(2.5, intake.getCurrentAmps(), EPS);
        intake.getCurrentAmps();
        intake.getCurrentAmps();
        assertEquals("readers use the sample, not the bus", 2, motor.currentReads);
    }

    @Test
    public void defaultIdleStopsTheMotor() {
        intake.defaultIdleCommand().schedule();
        tick();
        assertEquals(0, motor.commandedVelocity, EPS);
        assertEquals(Intake.Mode.IDLE, intake.getMode());
    }

    @Test
    public void intakeCommandPreemptsIdleAndIdleResumesWhenItEnds() {
        intake.defaultIdleCommand().schedule();
        tick();

        Command run = intake.intakeCommand();
        run.schedule();
        tick();
        assertEquals(Intake.INTAKE_TICKS_PER_SEC, motor.commandedVelocity, EPS);

        Scheduler.cancel(run);
        tick();
        assertEquals("idle must come back on its own", 0, motor.commandedVelocity, EPS);
        assertEquals(Intake.Mode.IDLE, intake.getMode());
    }

    /** The default-command trap: an instant without a requirement is overwritten before the write. */
    @Test
    public void aDirectCallUnderTheDefaultCommandNeverReachesTheMotor() {
        intake.defaultIdleCommand().schedule();
        tick();

        instant(() -> intake.intake()).schedule();
        assertEquals("the instant ran immediately", Intake.Mode.INTAKING, intake.getMode());

        tick();
        assertEquals(Intake.Mode.IDLE, intake.getMode());
        assertEquals("the motor never saw the request", 0, motor.maxCommandedVelocity, EPS);
    }

    @Test
    public void aGroupThatRequiresTheIntakeMayCallItDirectly() {
        intake.defaultIdleCommand().schedule();
        tick();

        Command group = sequential(instant(intake::intake), waitMs(10_000)).requiring(intake);
        group.schedule();
        tick();
        assertEquals(Intake.INTAKE_TICKS_PER_SEC, motor.commandedVelocity, EPS);
        tick();
        assertEquals("still ours while the group runs", Intake.INTAKE_TICKS_PER_SEC,
                motor.commandedVelocity, EPS);
    }

    @Test
    public void fullStorageHoldsTheRollerStillButKeepsTheMode() {
        boolean[] full = {true};
        intake.setFullSupplier(() -> full[0]);
        intake.intakeCommand().schedule();
        tick();
        assertEquals(Intake.Mode.INTAKING, intake.getMode());
        assertTrue(intake.isBlockedByFullStorage());
        assertEquals("no fifth piece (G407)", 0, motor.commandedVelocity, EPS);

        full[0] = false;                             // a piece was shot
        tick();
        assertEquals(Intake.INTAKE_TICKS_PER_SEC, motor.commandedVelocity, EPS);
    }

    @Test
    public void sustainedOverCurrentWhileIntakingReversesTheMotor() {
        intake.intakeCommand().schedule();
        motor.currentAmps = Intake.STALL_CURRENT_AMPS + 3;

        tick();                                                     // t = 0: stall first seen
        assertEquals(Intake.INTAKE_TICKS_PER_SEC, motor.commandedVelocity, EPS);
        while (clock.nowMs() < Intake.STALL_TIMEOUT_MS) tick();    // inside the dwell
        assertEquals(Intake.INTAKE_TICKS_PER_SEC, motor.commandedVelocity, EPS);

        tick();                                                     // dwell elapsed
        assertEquals(Intake.UNJAM_TICKS_PER_SEC, motor.commandedVelocity, EPS);
        assertTrue(intake.isUnjamming());
        assertEquals(1, intake.getUnjamAttempts());
    }

    @Test
    public void aBlockedRollerNeverTriggersAntiJam() {
        intake.setFullSupplier(() -> true);
        intake.intakeCommand().schedule();
        motor.currentAmps = 20;
        for (int i = 0; i < 50; i++) tick();
        assertEquals(0, motor.commandedVelocity, EPS);
        assertEquals(0, intake.getUnjamAttempts());
    }

    @Test
    public void opponentPieceAtTheEntranceIsThrownBackOutForRejectMs() {
        // G408 scaffold: while intaking, the reject supplier reverses the roller for REJECT_MS and
        // the piece is not claimed as captured; then intaking resumes by itself.
        Intake.REJECT_ENABLED = true;
        final boolean[] opponent = {false};
        intake.setRejectSupplier(() -> opponent[0]);
        intake.intakeCommand().schedule();
        tick();
        assertEquals(Intake.INTAKE_TICKS_PER_SEC, motor.commandedVelocity, EPS);

        opponent[0] = true;
        long startedAt = clock.nowMs();
        tick();
        assertTrue(intake.isRejecting());
        assertEquals(Intake.EJECT_TICKS_PER_SEC, motor.commandedVelocity, EPS);
        assertEquals(1, intake.getRejections());
        opponent[0] = false;                                     // it left
        while (clock.nowMs() < startedAt + Intake.REJECT_MS) {
            tick();
            assertEquals("reverses for the whole pulse", Intake.EJECT_TICKS_PER_SEC, motor.commandedVelocity, EPS);
        }
        tick();
        assertFalse(intake.isRejecting());
        assertEquals("back to intaking on its own", Intake.INTAKE_TICKS_PER_SEC, motor.commandedVelocity, EPS);
        assertEquals(Intake.Mode.INTAKING, intake.getMode());
    }

    @Test
    public void rejectIsInertUntilEnabled() {
        intake.setRejectSupplier(() -> true);
        intake.intakeCommand().schedule();
        for (int i = 0; i < 10; i++) tick();
        assertFalse(intake.isRejecting());
        assertEquals(Intake.INTAKE_TICKS_PER_SEC, motor.commandedVelocity, EPS);
    }

    @Test
    public void anUnavailableIntakeNoOpsEverywhere() {
        Intake none = new Intake((DcMotorEx) null, clock);
        assertFalse(none.isAvailable());
        none.intake();
        none.update();
        assertEquals(0, none.getVelocityTicksPerSec(), EPS);
        assertEquals(0, none.getCurrentAmps(), EPS);
    }
}
