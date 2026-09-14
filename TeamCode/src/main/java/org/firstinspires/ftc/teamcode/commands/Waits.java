package org.firstinspires.ftc.teamcode.commands;

import static com.pedropathing.ivy.groups.Groups.race;

import com.pedropathing.ivy.Command;

import org.firstinspires.ftc.teamcode.util.time.Clock;

/**
 * Waits on the robot's injected {@link Clock} instead of Ivy's wall-clock {@code Commands.waitMs}.
 *
 * <p>Ivy's own {@code waitMs} reads {@code System.currentTimeMillis()}, which a test cannot fake
 * without real sleeps (docs/01 section B.5, trap 3). Everything else in this codebase already
 * takes time from one injected monotonic {@link Clock}, so macro and auto timeouts do the same:
 * {@code FakeClock.advance(20)} per tick makes an eight-second timeout deterministic in 400 loops.
 *
 * <p>Lives in {@code commands/} rather than {@code util/time} because {@code util} may not import
 * Ivy (docs/03 section 3).
 */
public final class Waits {
    private Waits() {}

    /** Done once {@code ms} have elapsed on {@code clock} since the command started. */
    public static Command waitMs(Clock clock, long ms) {
        final long[] startedAt = new long[1];
        return Command.build()
                .setStart(() -> startedAt[0] = clock.nowMs())
                .setDone(() -> clock.nowMs() - startedAt[0] >= ms);
    }

    /**
     * {@code work} raced against a clock timeout: finishes when either does, and interrupts the
     * other. The bounded-wait rule (docs/03 section 18) in one call.
     */
    public static Command bounded(Clock clock, Command work, long timeoutMs) {
        return race(work, waitMs(clock, timeoutMs));
    }
}
