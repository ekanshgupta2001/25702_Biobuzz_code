package org.firstinspires.ftc.teamcode.commands;

import static com.pedropathing.ivy.groups.Groups.race;

import com.pedropathing.ivy.Command;

/**
 * A wait, and the bounded-wait rule in one call.
 *
 * <p>Ivy ships {@code Commands.waitMs}, and on a real robot it is fine — this exists for
 * {@link #bounded}, which is the rule that matters: <b>every wait on a condition has a deadline</b>.
 * The reference codebase this robot's structure is modelled on writes
 * {@code Commands.waitUntil(shooter::atTarget)} with no timeout, so a dead flywheel or a jammed feed
 * hangs its autonomous for the rest of the match. One {@code race} against a timer is the whole fix.
 */
public final class Waits {
    private Waits() {}

    /** Done once {@code ms} have elapsed since the command started. */
    public static Command waitMs(long ms) {
        final long[] startedAt = new long[1];
        return Command.build()
                .setStart(() -> startedAt[0] = System.currentTimeMillis())
                .setDone(() -> System.currentTimeMillis() - startedAt[0] >= ms);
    }

    /** {@code work} raced against a timeout: finishes when either does, and interrupts the other. */
    public static Command bounded(Command work, long timeoutMs) {
        return race(work, waitMs(timeoutMs));
    }
}
