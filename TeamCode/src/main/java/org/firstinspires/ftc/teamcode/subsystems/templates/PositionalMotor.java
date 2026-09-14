package org.firstinspires.ftc.teamcode.subsystems.templates;

import com.pedropathing.ivy.Command;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.util.hardware.Hardware;
import org.firstinspires.ftc.teamcode.util.time.Clock;

import java.util.EnumMap;
import java.util.Map;

/**
 * A motor driven to named encoder positions using the hub's built-in RUN_TO_POSITION controller.
 *
 * <pre>
 *   public enum Level { DOWN, HIGH }
 *   lift = new PositionalMotor&lt;&gt;(hardwareMap, "lift", Level.class)
 *           .preset(Level.DOWN, 0).preset(Level.HIGH, 1200).limits(0, 1250);
 *   lift.goTo(Level.HIGH).schedule();
 * </pre>
 *
 * <h2>Encoder zero is wherever the mechanism sits at init</h2>
 * The constructor resets the encoder, so every preset is relative to the mechanism's position
 * when the OpMode started. Start with the mechanism in a repeatable place, ideally against a hard
 * stop, or the presets mean nothing.
 *
 * <h2>Power is deliberately not applied in the constructor</h2>
 * Doing so would drive the mechanism during {@code init()}, before the driver has pressed start.
 * Power is applied on the first move instead.
 *
 * @param <S> the enum naming this mechanism's positions
 */
public class PositionalMotor<S extends Enum<S>> implements Mechanism<S> {
    public static double DEFAULT_POWER = 0.7;
    public static int DEFAULT_TOLERANCE_TICKS = 15;
    /** Give up on a move after this long, so a jam cannot hold the resource for the whole match. */
    public static long MOVE_TIMEOUT_MS = 3000;
    /** Minimum time before a move may report complete, so it cannot finish on the first tick. */
    public static long MIN_MOVE_MS = 60;

    private final DcMotorEx motor;
    private final Clock clock;
    private final Map<S, Integer> presets;
    private double maxPower = DEFAULT_POWER;
    private int toleranceTicks = DEFAULT_TOLERANCE_TICKS;
    private int target = 0;
    private S state = null;
    private Integer minTicks = null;
    private Integer maxTicks = null;

    public PositionalMotor(HardwareMap hardwareMap, String name, Class<S> stateType) {
        this(hardwareMap, name, stateType, DcMotorSimple.Direction.FORWARD);
    }

    public PositionalMotor(HardwareMap hardwareMap, String name, Class<S> stateType,
                           DcMotorSimple.Direction direction) {
        this(hardwareMap, name, stateType, direction, Clock.system());
    }

    public PositionalMotor(HardwareMap hardwareMap, String name, Class<S> stateType,
                           DcMotorSimple.Direction direction, Clock clock) {
        this(Hardware.get(hardwareMap, DcMotorEx.class, name), stateType, direction, clock);
    }

    /** Builds on an already-resolved motor ({@code null} for "not fitted"). Tests inject a fake. */
    public PositionalMotor(DcMotorEx motor, Class<S> stateType,
                           DcMotorSimple.Direction direction, Clock clock) {
        this.presets = new EnumMap<>(stateType);
        this.motor = motor;
        this.clock = clock;
        if (motor == null) return;
        motor.setDirection(direction);
        motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        // Target before mode: the SDK expects a target to exist when RUN_TO_POSITION is entered,
        // otherwise the motor briefly runs toward whatever target was last latched.
        motor.setTargetPosition(0);
        motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motor.setTargetPositionTolerance(toleranceTicks);
    }

    /** False when the motor is missing from the robot configuration. All motion calls no-op. */
    public boolean isAvailable() {
        return motor != null;
    }

    /** Registers the encoder position for one state. Chainable. */
    public PositionalMotor<S> preset(S state, int ticks) {
        presets.put(state, ticks);
        return this;
    }

    /** Clamps every future target into [min, max]. Protects the mechanism from a bad preset. */
    public PositionalMotor<S> limits(int minTicks, int maxTicks) {
        this.minTicks = minTicks;
        this.maxTicks = maxTicks;
        return this;
    }

    public void setMaxPower(double power) {
        this.maxPower = Math.min(1, Math.abs(power));
    }

    public void setTolerance(int ticks) {
        this.toleranceTicks = ticks;
        if (motor != null) motor.setTargetPositionTolerance(ticks);
    }

    /** Moves to a raw encoder position, bypassing the presets. */
    public void goToTicks(int ticks) {
        if (motor == null) return;
        if (minTicks != null) ticks = Math.max(minTicks, Math.min(maxTicks, ticks));
        target = ticks;
        motor.setTargetPosition(ticks);
        motor.setPower(maxPower);
    }

    /**
     * Moves to a registered state. States without a preset are ignored rather than throwing: this
     * runs inside a command's {@code start()} on the OpMode thread, where an exception would stop
     * the robot mid-match.
     */
    public void goToState(S state) {
        Integer ticks = presets.get(state);
        if (ticks == null) return;
        this.state = state;
        goToTicks(ticks);
    }

    public int getCurrentTicks() {
        return motor == null ? 0 : motor.getCurrentPosition();
    }

    public int getTarget() {
        return target;
    }

    @Override
    public S getState() {
        return state;
    }

    @Override
    public boolean atState() {
        return atTarget();
    }

    public boolean atTarget() {
        return motor == null || Math.abs(motor.getCurrentPosition() - target) <= toleranceTicks;
    }

    /** Cuts power, leaving the mechanism held by the motor's BRAKE zero-power behaviour. */
    public void stop() {
        if (motor != null) motor.setPower(0);
    }

    /** Intentionally empty: RUN_TO_POSITION is closed-loop in the hub, with nothing to tick here. */
    @Override
    public void update() {
    }

    /**
     * Drives to a state, finishing on arrival or after {@link #MOVE_TIMEOUT_MS}. Three guards:
     * {@link #MIN_MOVE_MS} against instant false completion, the timeout against hanging on a jam,
     * and {@code setEnd} against an interrupted move leaving the motor driving.
     */
    @Override
    public Command goTo(S state) {
        return timedMove(() -> goToState(state));
    }

    /** Same guarantees as {@link #goTo}, for a raw encoder position. */
    public Command goToTicksCommand(int ticks) {
        return timedMove(() -> goToTicks(ticks));
    }

    private Command timedMove(Runnable start) {
        final long[] startedAt = new long[1];
        return Command.build()
                .setStart(() -> {
                    startedAt[0] = clock.nowMs();
                    start.run();
                })
                .setDone(() -> {
                    long elapsed = clock.nowMs() - startedAt[0];
                    if (elapsed < MIN_MOVE_MS) return false;
                    return atTarget() || elapsed >= MOVE_TIMEOUT_MS;
                })
                .setEnd(ec -> stop())
                .requiring(this);
    }
}
