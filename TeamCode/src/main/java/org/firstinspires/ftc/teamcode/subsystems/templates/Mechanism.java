package org.firstinspires.ftc.teamcode.subsystems.templates;

import com.pedropathing.ivy.Command;

/**
 * A mechanism that moves between a fixed set of named positions.
 *
 * <h2>Why the state is an enum</h2>
 * Almost every FTC mechanism is a small state machine. Naming its states in an {@code enum} rather
 * than with strings means a typo is a <b>compile error</b> instead of a robot that silently does
 * nothing in the middle of a match, and the set of legal positions is discoverable.
 *
 * <pre>
 *   public enum Gate { OPEN, CLOSED }
 *   gate.goTo(Gate.OPEN).schedule();   // a typo here will not compile
 * </pre>
 *
 * {@link PositionalMotor} and {@link PositionalServo} implement it for the two common cases, so
 * most mechanisms need no new code, just an enum and some preset values.
 *
 * @param <S> the enum naming this mechanism's positions
 */
public interface Mechanism<S extends Enum<S>> {

    /** Builds a command that moves to {@code state}. Nothing happens until it is scheduled. */
    Command goTo(S state);

    /** The state most recently commanded, not necessarily the one reached. See {@link #atState()}. */
    S getState();

    /** True once the mechanism has physically arrived at {@link #getState()}. */
    boolean atState();

    /** Per-loop hardware tick, from {@code Robot.writeActuators()}. May legitimately do nothing. */
    void update();
}
