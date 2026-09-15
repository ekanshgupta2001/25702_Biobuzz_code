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

/** Storage counting and transport commands against a fake motor and fake sensors. */
public class StorageTest {
    private static final double EPS = 1e-9;

    private FakeDcMotorEx motor;
    private FakeClock clock;
    private Storage storage;
    private final boolean[] entrance = {false};
    private final boolean[] full = {false};
    private final boolean[] exit = {false};

    @Before
    public void setUp() {
        Scheduler.reset();
        motor = new FakeDcMotorEx();
        clock = new FakeClock();
        storage = new Storage(motor, null, clock);
        storage.setEntranceSupplier(() -> entrance[0]);
        storage.setFullSupplier(() -> full[0]);
        storage.setExitSupplier(() -> exit[0]);
        entrance[0] = full[0] = exit[0] = false;
        Storage.CAPACITY = 4;
        Storage.ADVANCE_TIMEOUT_MS = 2500;
    }

    @After
    public void tearDown() {
        Scheduler.reset();
    }

    private void tick() {
        Scheduler.execute();
        storage.update();
        clock.advance(20);
    }

    @Test
    public void countsRisingEdgesAtTheEntranceOnly() {
        entrance[0] = true;
        tick();
        tick();
        tick();
        assertEquals("one piece, however long it sits on the sensor", 1, storage.count());
        entrance[0] = false;
        tick();
        entrance[0] = true;
        tick();
        assertEquals(2, storage.count());
        assertEquals(2, storage.getEntryEvents());
    }

    @Test
    public void countClampsAtCapacityAndReportsFull() {
        storage.setCount(4);
        assertTrue(storage.isFull());
        storage.markEntered();
        assertEquals(Storage.CAPACITY, storage.count());
        storage.setCount(-3);
        assertEquals(0, storage.count());
        assertFalse(storage.hasPiece());
    }

    @Test
    public void exitEdgesCountOutAndNeverGoNegative() {
        storage.setCount(1);
        exit[0] = true;
        tick();
        tick();
        assertEquals(0, storage.count());
        exit[0] = false;
        tick();
        exit[0] = true;
        tick();
        assertEquals("cannot go below zero", 0, storage.count());
        assertEquals(2, storage.getExitEvents());
    }

    @Test
    public void fullSensorOverridesTheCount() {
        assertFalse(storage.isFull());
        full[0] = true;
        assertTrue(storage.isFull());
    }

    @Test
    public void fullSensorFittedIsRecorded() {
        assertTrue(storage.hasFullSensor());
        assertTrue(storage.canDetectFull());
        storage.setFullSupplier(null);
        storage.setEntranceSupplier(null);
        assertFalse(storage.hasFullSensor());
        assertFalse("no entrance, no full sensor: nothing can say full", storage.canDetectFull());
        assertFalse(storage.isFull());
        storage.setEntranceSupplier(() -> false);
        assertTrue("an entrance sensor counts up to full", storage.canDetectFull());
    }

    @Test
    public void advanceOneRunsUntilAPieceLeaves() {
        storage.setCount(2);
        Command cmd = storage.advanceOneCommand();
        cmd.schedule();
        tick();
        assertEquals(Storage.ADVANCE_TICKS_PER_SEC, motor.commandedVelocity, EPS);
        assertEquals(Storage.Mode.ADVANCING, storage.getMode());
        assertTrue(Scheduler.isScheduled(cmd));

        exit[0] = true;                              // the piece reaches the transfer
        tick();                                      // update counts it; next execute sees it
        tick();
        assertFalse(Scheduler.isScheduled(cmd));
        assertEquals(1, storage.count());
        assertEquals(0, motor.commandedVelocity, EPS);
        assertEquals(Storage.Mode.IDLE, storage.getMode());
    }

    @Test
    public void advanceOneGivesUpAfterTheTimeout() {
        storage.setCount(1);
        Command cmd = storage.advanceOneCommand();
        cmd.schedule();
        clock.advance(Storage.ADVANCE_TIMEOUT_MS - 1);
        Scheduler.execute();
        assertTrue(Scheduler.isScheduled(cmd));
        clock.advance(1);
        Scheduler.execute();
        assertFalse(Scheduler.isScheduled(cmd));
        assertEquals("count untouched: nothing was seen leaving", 1, storage.count());
    }

    @Test
    public void advanceOneSkipsAnEmptyChannel() {
        Command cmd = storage.advanceOneCommand();
        cmd.schedule();
        Scheduler.execute();
        assertFalse(Scheduler.isScheduled(cmd));
        storage.update();
        assertEquals("never asked the motor to move", 0, motor.commandedVelocity, EPS);
    }

    @Test
    public void advanceUntilStopsOnTheConditionAndInterruptStopsTheMotor() {
        boolean[] stop = {false};
        Command cmd = storage.advanceUntilCommand(() -> stop[0], 5000);
        cmd.schedule();
        tick();
        assertEquals(Storage.ADVANCE_TICKS_PER_SEC, motor.commandedVelocity, EPS);
        stop[0] = true;
        tick();
        assertFalse(Scheduler.isScheduled(cmd));
        assertEquals(0, motor.commandedVelocity, EPS);

        Command run = storage.advanceCommand();
        run.schedule();
        tick();
        Scheduler.cancel(run);
        storage.update();
        assertEquals(0, motor.commandedVelocity, EPS);
    }

    @Test
    public void defaultIdleKeepsTheTransportStillAndResumesAfterAMacro() {
        storage.defaultIdleCommand().schedule();
        tick();
        assertEquals(0, motor.commandedVelocity, EPS);

        Command run = storage.reverseCommand();
        run.schedule();
        tick();
        assertEquals(Storage.REVERSE_TICKS_PER_SEC, motor.commandedVelocity, EPS);
        Scheduler.cancel(run);
        tick();
        assertEquals(0, motor.commandedVelocity, EPS);
        assertEquals(Storage.Mode.IDLE, storage.getMode());
    }

    @Test
    public void unavailableStorageStillCountsButNeverMoves() {
        Storage none = new Storage((DcMotorEx) null, null, clock);
        none.setEntranceSupplier(() -> true);
        assertFalse(none.isAvailable());
        none.update();
        assertEquals(1, none.count());
        Command cmd = none.advanceOneCommand();
        cmd.schedule();
        Scheduler.execute();
        assertFalse(Scheduler.isScheduled(cmd));
    }
}
