package org.firstinspires.ftc.teamcode.subsystems.templates;

import static com.pedropathing.ivy.commands.Commands.instant;
import static com.pedropathing.ivy.commands.Commands.waitMs;
import static com.pedropathing.ivy.groups.Groups.sequential;

import com.pedropathing.ivy.Command;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.util.hardware.Hardware;

import java.util.EnumMap;
import java.util.Map;

/**
 * A servo driven to named positions.
 *
 * <pre>
 *   public enum Gate { OPEN, CLOSED }
 *   gate = new PositionalServo&lt;&gt;(hardwareMap, "shooter_feed", Gate.class)
 *           .preset(Gate.OPEN, 0.65).preset(Gate.CLOSED, 0.30);
 * </pre>
 *
 * <h2>Modelling travel time</h2>
 * A servo has no position feedback, so arrival cannot be measured, only waited out.
 * {@link #goTo} is {@code sequential(instant(set), waitMs(travel))}, which holds the subsystem's
 * Ivy resource for the whole travel time so the next command in a sequence cannot act on a
 * mechanism that is still moving. {@link #TRAVEL_MS_PER_UNIT} scales the wait with distance.
 *
 * @param <S> the enum naming this mechanism's positions
 */
public class PositionalServo<S extends Enum<S>> implements Mechanism<S> {
    /** Milliseconds per unit of servo travel (full sweep is 1.0). Measure this on the real servo. */
    public static long TRAVEL_MS_PER_UNIT = 500;
    /** Floor on the travel wait, covering fixed overheads on very small moves. */
    public static long MIN_TRAVEL_MS = 60;

    private final Servo servo;
    private final Map<S, Double> presets;
    private double currentPos;
    private S state = null;

    public PositionalServo(HardwareMap hardwareMap, String name, Class<S> stateType) {
        this(hardwareMap, name, stateType, Double.NaN);
    }

    /**
     * @param initialPosition position to command at init, or {@link Double#NaN} to leave the servo
     *                        where it is. Prefer NaN unless the path is known to be clear: a servo
     *                        commanded during init slams to that position before the match starts.
     */
    public PositionalServo(HardwareMap hardwareMap, String name, Class<S> stateType,
                           double initialPosition) {
        this(Hardware.get(hardwareMap, Servo.class, name), stateType, initialPosition);
    }

    /** Builds on an already-resolved servo ({@code null} for "not fitted"). */
    public PositionalServo(Servo servo, Class<S> stateType, double initialPosition) {
        this.presets = new EnumMap<>(stateType);
        this.servo = servo;
        if (servo == null) return;
        if (!Double.isNaN(initialPosition)) {
            setNow(initialPosition);
        } else {
            currentPos = servo.getPosition();
        }
    }

    /** False when the servo is missing from the robot configuration. All motion calls no-op. */
    public boolean isAvailable() {
        return servo != null;
    }

    /** Registers the position for one state. Chainable. */
    public PositionalServo<S> preset(S state, double position) {
        presets.put(state, Range.clip(position, 0.0, 1.0));
        return this;
    }

    /** Commands a raw position immediately, without waiting for travel. */
    public void setNow(double position) {
        if (servo == null) return;
        currentPos = Range.clip(position, 0.0, 1.0);
        servo.setPosition(currentPos);
    }

    /** States without a preset are ignored rather than throwing; this runs inside a command. */
    public void setNow(S state) {
        Double pos = presets.get(state);
        if (pos == null) return;
        this.state = state;
        setNow(pos);
    }

    /** The last commanded position, after clipping. Servos cannot report where they actually are. */
    public double getCommandedPosition() {
        return currentPos;
    }

    @Override
    public S getState() {
        return state;
    }

    /** Always true: a servo has no feedback, so arrival is assumed once the travel wait elapses. */
    @Override
    public boolean atState() {
        return true;
    }

    /** Intentionally empty: a servo is open-loop, with no per-loop work to do. */
    @Override
    public void update() {
    }

    /** Travel time for a move of the given distance, in servo units. */
    public long travelMsFor(double delta) {
        return Math.max(MIN_TRAVEL_MS, (long) (Math.abs(delta) * TRAVEL_MS_PER_UNIT));
    }

    @Override
    public Command goTo(S state) {
        Double target = presets.get(state);
        if (target == null) return Command.build().setDone(() -> true);
        return goToPosition(target, () -> setNow(state));
    }

    /** Same travel-time modelling as {@link #goTo}, for a raw position. */
    public Command goToCommand(double position) {
        double clipped = Range.clip(position, 0.0, 1.0);
        return goToPosition(clipped, () -> setNow(clipped));
    }

    private Command goToPosition(double target, Runnable apply) {
        // Distance is measured when the command is BUILT, which is the best estimate available:
        // only another command holding this same resource could move the servo first.
        long travel = travelMsFor(target - currentPos);
        return sequential(
                instant(apply).requiring(this),
                waitMs(travel)
        );
    }
}
