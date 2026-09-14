package org.firstinspires.ftc.teamcode.opmodes.teleop;

import com.qualcomm.robotcore.hardware.Gamepad;

import java.util.ArrayList;
import java.util.List;
import java.util.function.Predicate;
import java.util.function.ToDoubleFunction;

/**
 * Every gamepad binding, and the help card that describes them, from one definition.
 *
 * <h2>Why this is an enum and not a pile of if-statements</h2>
 * A binding written as {@code gamepad1.aWasPressed()} in the handler and again as a hand-typed
 * string on the help card has nothing connecting the two, and the card is the only thing a new
 * driver has at 9am on competition day. Here the button, the label and the description are one
 * object; {@link #helpLines()} renders the card from the same constants {@code Teleop} reads.
 *
 * <h2>Read once per loop</h2>
 * The SDK's {@code *WasPressed()} methods consume their flag on read: a second call in the same
 * loop returns false, and a press nobody reads stays latched until somebody does. {@code Teleop}
 * therefore takes one {@link Snapshot} per loop with {@link #read}, which reads every edge exactly
 * once, and decides from that. A button pressed while a macro is running is consumed that loop
 * instead of firing the moment the macro ends.
 *
 * <h2>Adding a control</h2>
 * Add an entry. It appears on the help card automatically; wire it in {@code Teleop} with
 * {@code in.pressed(CONTROL)} for a button or {@code CONTROL.axis(gamepad1, gamepad2)} for a stick
 * or trigger. Order here is the order shown on the card, so group related actions together.
 *
 * <p>Analog controls carry an axis function instead of an edge test. Sign conventions live in that
 * function: the SDK reports stick-up as negative, so {@link #DRIVE_FORWARD} negates it and every
 * caller gets "forward is positive" without remembering why.
 */
public enum Controls {
    // ---- Driver (gamepad 1): anything that moves the robot ----
    DRIVE_FORWARD(Pad.DRIVER, "L-stick up", "drive forward", null, gp -> -gp.left_stick_y),
    DRIVE_STRAFE(Pad.DRIVER, "L-stick left", "strafe left", null, gp -> -gp.left_stick_x),
    DRIVE_TURN(Pad.DRIVER, "R-stick left", "turn left", null, gp -> -gp.right_stick_x),
    SLOW_MODE(Pad.DRIVER, "L-trigger", "precision slow mode", null, gp -> gp.left_trigger),
    AIM_LOCK(Pad.DRIVER, "R-trigger", "hold: aim the shooter at the HIVE", null, gp -> gp.right_trigger),
    TOGGLE_DRIVE_FRAME(Pad.DRIVER, "LB", "field / robot centric", Gamepad::leftBumperWasPressed, null),
    RESET_HEADING(Pad.DRIVER, "Y", "re-zero field heading", Gamepad::yWasPressed, null),
    ABORT(Pad.DRIVER, "BACK", "abort macro (or just move a stick)", Gamepad::backWasPressed, null),

    COLLECT(Pad.DRIVER, "A", "collect a piece (camera)", Gamepad::aWasPressed, null),
    ALIGN(Pad.DRIVER, "X", "turn to face a piece (camera)", Gamepad::xWasPressed, null),
    DRIVE_TO_SHOOT(Pad.DRIVER, "B", "path to the shooting spot", Gamepad::bWasPressed, null),
    DRIVE_TO_PARK(Pad.DRIVER, "RB", "path to park", Gamepad::rightBumperWasPressed, null),

    SNAP_90(Pad.DRIVER, "dpad up", "snap to 90 deg", Gamepad::dpadUpWasPressed, null),
    SNAP_0(Pad.DRIVER, "dpad right", "snap to 0 deg", Gamepad::dpadRightWasPressed, null),
    SNAP_270(Pad.DRIVER, "dpad down", "snap to 270 deg", Gamepad::dpadDownWasPressed, null),
    SNAP_180(Pad.DRIVER, "dpad left", "snap to 180 deg", Gamepad::dpadLeftWasPressed, null),

    // ---- Operator (gamepad 2): mechanisms and diagnostics ----
    INTAKE(Pad.OPERATOR, "RB", "run intake", Gamepad::rightBumperWasPressed, null),
    OUTTAKE(Pad.OPERATOR, "LB", "run intake backwards", Gamepad::leftBumperWasPressed, null),
    EJECT(Pad.OPERATOR, "B", "eject at full speed", Gamepad::bWasPressed, null),
    STOP_INTAKE(Pad.OPERATOR, "X", "stop intake / cancel macro", Gamepad::xWasPressed, null),
    INTAKE_UNTIL_FULL(Pad.OPERATOR, "Y", "intake until storage is full", Gamepad::yWasPressed, null),

    SHOOT_ONE(Pad.OPERATOR, "R-trigger", "shoot one", Gamepad::rightTriggerWasPressed, null),
    SHOOT_ALL(Pad.OPERATOR, "A", "shoot everything", Gamepad::aWasPressed, null),
    ARM_FLYWHEEL(Pad.OPERATOR, "L-trigger", "flywheel on / off", Gamepad::leftTriggerWasPressed, null),

    HIVE_TIPPED(Pad.OPERATOR, "dpad down", "our HIVE tipped: aim at the other CELL", Gamepad::dpadDownWasPressed, null),

    TOGGLE_DEBUG(Pad.OPERATOR, "BACK", "toggle debug telemetry", Gamepad::backWasPressed, null);

    /** Which driver holds this control. */
    public enum Pad { DRIVER, OPERATOR }

    private final Pad pad;
    private final String button;
    private final String description;
    private final Predicate<Gamepad> edge;
    private final ToDoubleFunction<Gamepad> axis;

    /**
     * A button carries an {@code edge} test and a null {@code axis}; a stick or trigger the
     * reverse. One constructor rather than two overloads because an implicitly typed lambda is
     * ambiguous between {@code Predicate} and {@code ToDoubleFunction} in Java 8.
     */
    Controls(Pad pad, String button, String description,
             Predicate<Gamepad> edge, ToDoubleFunction<Gamepad> axis) {
        this.pad = pad;
        this.button = button;
        this.description = description;
        this.edge = edge;
        this.axis = axis;
    }

    /**
     * Whether this control was pressed since the last check. Consumes the SDK's edge flag, so call
     * it once per loop, or use {@link #read}. Takes both gamepads and picks the right one, so
     * moving a binding between pads is a one-line change here. Always false for analog controls.
     */
    public boolean wasPressed(Gamepad driver, Gamepad operator) {
        if (edge == null) return false;
        return edge.test(pad == Pad.DRIVER ? driver : operator);
    }

    /**
     * The current value of a stick or trigger, with the sign convention already applied. Not
     * consumed: safe to read as often as needed. Always 0 for button controls.
     */
    public double axis(Gamepad driver, Gamepad operator) {
        if (axis == null) return 0;
        return axis.applyAsDouble(pad == Pad.DRIVER ? driver : operator);
    }

    public boolean isAnalog() {
        return axis != null;
    }

    public Pad pad() {
        return pad;
    }

    public String button() {
        return button;
    }

    public String description() {
        return description;
    }

    /** Reads every control exactly once: all edges consumed, all axes sampled. */
    public static Snapshot read(Gamepad driver, Gamepad operator) {
        Controls[] all = values();
        boolean[] pressed = new boolean[all.length];
        double[] axes = new double[all.length];
        for (Controls c : all) {
            pressed[c.ordinal()] = c.wasPressed(driver, operator);
            axes[c.ordinal()] = c.axis(driver, operator);
        }
        return new Snapshot(pressed, axes);
    }

    /** The state of every control at one instant. Immutable. */
    public static final class Snapshot {
        private final boolean[] pressed;
        private final double[] axes;

        private Snapshot(boolean[] pressed, double[] axes) {
            this.pressed = pressed;
            this.axes = axes;
        }

        public boolean pressed(Controls control) {
            return pressed[control.ordinal()];
        }

        public double axis(Controls control) {
            return axes[control.ordinal()];
        }
    }

    /**
     * The init-phase help card, generated from the bindings above. Several controls share a line
     * where they form one group, because a driver reads "dpad = snap to heading" faster than four
     * separate lines.
     */
    public static List<String> helpLines() {
        List<String> lines = new ArrayList<>();
        for (Pad pad : Pad.values()) {
            lines.add(pad == Pad.DRIVER ? "DRIVER (gamepad 1)" : "OPERATOR (gamepad 2)");
            StringBuilder row = new StringBuilder("  ");
            for (Controls c : values()) {
                if (c.pad != pad) continue;
                String entry = c.button + "=" + c.description;
                if (row.length() + entry.length() > 70) {
                    lines.add(row.toString());
                    row = new StringBuilder("  ");
                }
                if (row.length() > 2) row.append("   ");
                row.append(entry);
            }
            if (row.length() > 2) lines.add(row.toString());
        }
        return lines;
    }
}
