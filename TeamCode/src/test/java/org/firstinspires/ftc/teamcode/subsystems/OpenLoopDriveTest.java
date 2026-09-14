package org.firstinspires.ftc.teamcode.subsystems;

import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertFalse;
import static org.junit.Assert.assertTrue;

import com.pedropathing.ivy.Command;
import com.pedropathing.ivy.Scheduler;

import org.firstinspires.ftc.teamcode.util.time.FakeClock;
import org.junit.After;
import org.junit.Before;
import org.junit.Test;

/** Timed open-loop driving against a fake Pedro motor layer. */
public class OpenLoopDriveTest {
    private static final double EPS = 1e-9;

    private FakePedroDrivetrain motors;
    private FakeClock clock;
    private OpenLoopDrive drive;

    @Before
    public void setUp() {
        Scheduler.reset();
        motors = new FakePedroDrivetrain();
        clock = new FakeClock();
        drive = new OpenLoopDrive(motors, clock);
    }

    @After
    public void tearDown() {
        Scheduler.reset();
    }

    private void tick() {
        Scheduler.execute();
        drive.update();
        clock.advance(20);
    }

    @Test
    public void driveForMsWritesThePowersThenStops() {
        Command cmd = drive.driveForMsCommand(-0.3, 0, 0, 100);
        cmd.schedule();
        tick();
        assertEquals(1, motors.driveCalls);
        assertEquals(-0.3, motors.lastPowers.forward(), EPS);
        assertTrue("open-loop drive is Pedro's manual mode", motors.lastManual);
        tick();
        tick();
        assertTrue(Scheduler.isScheduled(cmd));
        assertEquals("unchanged intent is not rewritten", 1, motors.driveCalls);
        clock.advance(100);
        tick();
        assertFalse(Scheduler.isScheduled(cmd));
        assertEquals(1, motors.stopCalls);
        assertFalse(motors.moving);
    }

    @Test
    public void idleWritesNothingSoItNeverFightsTheFollower() {
        drive.defaultStopCommand().schedule();
        for (int i = 0; i < 5; i++) tick();
        assertEquals(0, motors.driveCalls);
        assertEquals("never asked to move, so never told to stop", 0, motors.stopCalls);
    }

    @Test
    public void defaultStopYieldsToADriveCommandAndResumes() {
        drive.defaultStopCommand().schedule();
        tick();
        Command cmd = drive.driveForMsCommand(0, 0.5, 0, 40);
        cmd.schedule();
        tick();
        assertEquals(0.5, motors.lastPowers.strafe(), EPS);
        clock.advance(40);
        tick();
        tick();
        assertFalse(Scheduler.isScheduled(cmd));
        assertEquals(1, motors.stopCalls);
        assertFalse(drive.isMoving());
    }

    @Test
    public void unavailableDriveNoOpsAndFinishesItsCommand() {
        OpenLoopDrive none = new OpenLoopDrive((com.pedropathing.drivetrain.Drivetrain) null, clock);
        assertFalse(none.isAvailable());
        Command cmd = none.driveForMsCommand(1, 0, 0, 40);
        cmd.schedule();
        none.update();
        Scheduler.execute();
        clock.advance(40);
        Scheduler.execute();
        assertFalse(Scheduler.isScheduled(cmd));
    }
}
