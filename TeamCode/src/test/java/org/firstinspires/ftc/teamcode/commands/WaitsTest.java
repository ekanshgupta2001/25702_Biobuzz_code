package org.firstinspires.ftc.teamcode.commands;

import static org.junit.Assert.assertFalse;
import static org.junit.Assert.assertTrue;

import com.pedropathing.ivy.Command;
import com.pedropathing.ivy.Scheduler;

import org.firstinspires.ftc.teamcode.util.time.FakeClock;
import org.junit.After;
import org.junit.Before;
import org.junit.Test;

/** Clock-based waits are deterministic under a fake clock. */
public class WaitsTest {
    private FakeClock clock;

    @Before
    public void setUp() {
        Scheduler.reset();
        clock = new FakeClock(500);
    }

    @After
    public void tearDown() {
        Scheduler.reset();
    }

    @Test
    public void waitMsFinishesOnlyWhenTheClockHasAdvanced() {
        Command wait = Waits.waitMs(clock, 100);
        wait.schedule();
        Scheduler.execute();
        assertTrue(Scheduler.isScheduled(wait));
        clock.advance(99);
        Scheduler.execute();
        assertTrue("one ms short", Scheduler.isScheduled(wait));
        clock.advance(1);
        Scheduler.execute();
        assertFalse(Scheduler.isScheduled(wait));
    }

    @Test
    public void boundedInterruptsTheWorkWhenTheTimeoutWins() {
        final boolean[] interrupted = {false};
        Command forever = Command.build().setDone(() -> false).setEnd(ec -> interrupted[0] = true);
        Command bounded = Waits.bounded(clock, forever, 60);
        bounded.schedule();
        Scheduler.execute();
        assertTrue(Scheduler.isScheduled(bounded));
        clock.advance(60);
        Scheduler.execute();
        assertFalse(Scheduler.isScheduled(bounded));
        assertTrue("the never-ending work was ended by the timeout", interrupted[0]);
    }
}
