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

/** Transfer lift/feed commands and their interlocks against a fake motor and fake sensors. */
public class TransferTest {
    private static final double EPS = 1e-9;

    private FakeDcMotorEx motor;
    private FakeClock clock;
    private Transfer transfer;
    private final boolean[] atFeed = {false};

    @Before
    public void setUp() {
        Scheduler.reset();
        motor = new FakeDcMotorEx();
        clock = new FakeClock();
        transfer = new Transfer(motor, clock);
        transfer.setAtFeedSupplier(() -> atFeed[0]);
        atFeed[0] = false;
        Transfer.LIFT_TIMEOUT_MS = 2000;
        Transfer.FEED_TIMEOUT_MS = 1000;
        Transfer.FEED_CLEAR_DWELL_MS = 100;
    }

    @After
    public void tearDown() {
        Scheduler.reset();
    }

    private void tick() {
        Scheduler.execute();
        transfer.update();
        clock.advance(20);
    }

    @Test
    public void liftOneRunsUntilAPieceIsStaged() {
        Command lift = transfer.liftOneCommand();
        lift.schedule();
        tick();
        assertEquals(Transfer.LIFT_TICKS_PER_SEC, motor.commandedVelocity, EPS);
        assertEquals(Transfer.Mode.LIFTING, transfer.getMode());

        atFeed[0] = true;
        tick();
        assertFalse(Scheduler.isScheduled(lift));
        assertEquals(0, motor.commandedVelocity, EPS);
    }

    @Test
    public void liftOneRefusesToDoubleFeed() {
        atFeed[0] = true;
        Command lift = transfer.liftOneCommand();
        lift.schedule();
        Scheduler.execute();
        assertFalse(Scheduler.isScheduled(lift));
        transfer.update();
        assertEquals("a staged piece must not be pushed by a second one", 0, motor.commandedVelocity, EPS);
    }

    @Test
    public void liftOneGivesUpAfterTheTimeout() {
        Command lift = transfer.liftOneCommand();
        lift.schedule();
        clock.advance(Transfer.LIFT_TIMEOUT_MS);
        Scheduler.execute();
        assertFalse(Scheduler.isScheduled(lift));
    }

    @Test
    public void withoutAFeedSensorLiftIsATimedPulse() {
        transfer.setAtFeedSupplier(null);
        assertFalse(transfer.hasFeedSensor());
        Command lift = transfer.liftOneCommand();
        lift.schedule();
        tick();
        assertEquals(Transfer.LIFT_TICKS_PER_SEC, motor.commandedVelocity, EPS);
        clock.advance(Transfer.LIFT_PULSE_MS);
        tick();
        assertFalse(Scheduler.isScheduled(lift));
        assertEquals(0, motor.commandedVelocity, EPS);
    }

    @Test
    public void feedRunsUntilTheSensorClearsPlusTheDwell() {
        atFeed[0] = true;
        Command feed = transfer.feedCommand();
        feed.schedule();
        tick();
        assertEquals(Transfer.FEED_TICKS_PER_SEC, motor.commandedVelocity, EPS);

        atFeed[0] = false;                           // the piece entered the wheels
        tick();
        assertTrue("dwell keeps pushing", Scheduler.isScheduled(feed));
        clock.advance(Transfer.FEED_CLEAR_DWELL_MS);
        tick();
        assertFalse(Scheduler.isScheduled(feed));
        assertEquals(0, motor.commandedVelocity, EPS);
    }

    @Test
    public void feedWithNothingStagedFinishesAtOnce() {
        Command feed = transfer.feedCommand();
        feed.schedule();
        Scheduler.execute();
        assertFalse(Scheduler.isScheduled(feed));
        transfer.update();
        assertEquals(0, motor.commandedVelocity, EPS);
    }

    @Test
    public void feedGivesUpAfterTheTimeout() {
        atFeed[0] = true;                            // never clears: a piece stuck at the feed
        Command feed = transfer.feedCommand();
        feed.schedule();
        clock.advance(Transfer.FEED_TIMEOUT_MS);
        Scheduler.execute();
        assertFalse(Scheduler.isScheduled(feed));
    }

    @Test
    public void withoutAFeedSensorFeedIsATimedPulse() {
        transfer.setAtFeedSupplier(null);
        Command feed = transfer.feedCommand();
        feed.schedule();
        tick();
        assertEquals(Transfer.FEED_TICKS_PER_SEC, motor.commandedVelocity, EPS);
        clock.advance(Transfer.FEED_PULSE_MS);
        tick();
        assertFalse(Scheduler.isScheduled(feed));
    }

    @Test
    public void defaultIdleStopsAndInterruptedLiftStops() {
        transfer.defaultIdleCommand().schedule();
        tick();
        Command lift = transfer.liftOneCommand();   // no feed sensor: a timed lift
        lift.schedule();
        tick();
        assertEquals(Transfer.LIFT_TICKS_PER_SEC, motor.commandedVelocity, EPS);
        Scheduler.cancel(lift);
        tick();
        assertEquals(0, motor.commandedVelocity, EPS);
        assertEquals(Transfer.Mode.IDLE, transfer.getMode());
    }
}
