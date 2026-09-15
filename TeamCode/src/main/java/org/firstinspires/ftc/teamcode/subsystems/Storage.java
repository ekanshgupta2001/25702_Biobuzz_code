package org.firstinspires.ftc.teamcode.subsystems;

import com.pedropathing.ivy.Command;
import com.pedropathing.ivy.behaviors.BlockedBehavior;
import com.pedropathing.ivy.behaviors.InterruptedBehavior;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.subsystems.templates.VelocityMotor;
import org.firstinspires.ftc.teamcode.util.hardware.Hardware;
import org.firstinspires.ftc.teamcode.util.hardware.HardwareNames;
import org.firstinspires.ftc.teamcode.util.time.Clock;

import java.util.function.BooleanSupplier;

/**
 * The low horizontal storage channel: up to four pieces moved rearward by compliant side wheels.
 *
 * <h2>The count is the product</h2>
 * BIOBUZZ G407 allows a robot to control at most four scoring elements, and every scoring macro
 * needs to know how many are aboard. So this subsystem's first job is bookkeeping: a piece enters
 * on a rising edge of the storage-entrance sensor and leaves on a rising edge of the transfer
 * sensor (both wired as suppliers by {@code Robot}). {@link #isFull()} is what stops the intake.
 * The count can be corrected by hand ({@link #setCount}) for the pre-load and for sensor misses.
 *
 * <h2>Transport</h2>
 * The side wheels run only on command: {@link #advanceOneCommand()} pushes the queue until one
 * piece leaves into the transfer, {@link #advanceUntilCommand} runs until any condition, and the
 * default command keeps the wheels still so pieces stay where they are between cycles. A second
 * transport motor is optional; both are written together.
 */
public class Storage {
    /** BIOBUZZ G407 limit and the channel's physical capacity. */
    public static int CAPACITY = 4;
    public static double ADVANCE_TICKS_PER_SEC = 1500;
    public static double REVERSE_TICKS_PER_SEC = -1000;
    /**
     * Second transport motor's direction. The side wheels face each other across the channel, so
     * two motors driving them from opposite sides usually need one REVERSE, or they fight.
     */
    public static DcMotorSimple.Direction SECOND_MOTOR_DIRECTION = DcMotorSimple.Direction.FORWARD;
    /** An advance that has not delivered a piece by then gives up (empty channel, or a jam). */
    public static long ADVANCE_TIMEOUT_MS = 2500;
    public static int DEFAULT_IDLE_PRIORITY = -1;

    public enum Mode { IDLE, ADVANCING, REVERSING }

    private final VelocityMotor transport;
    private final VelocityMotor transport2;
    private final Clock clock;

    private Mode mode = Mode.IDLE;
    private double targetVelocity = 0;
    private int count = 0;
    /** Monotonic event counters, so a command can ask "did a piece leave since I started?". */
    private int entryEvents = 0;
    private int exitEvents = 0;

    private BooleanSupplier entranceSupplier = () -> false;
    private BooleanSupplier fullSupplier = () -> false;
    private BooleanSupplier exitSupplier = () -> false;
    private boolean exitSensorFitted = false;
    private boolean entranceSensorFitted = false;
    private boolean fullSensorFitted = false;
    private boolean lastEntrance = false;
    private boolean lastExit = false;

    public Storage(HardwareMap hardwareMap) {
        this(hardwareMap, HardwareNames.STORAGE_MOTOR, HardwareNames.STORAGE_MOTOR_2, Clock.system());
    }

    public Storage(HardwareMap hardwareMap, String name, String secondName, Clock clock) {
        this(Hardware.get(hardwareMap, DcMotorEx.class, name),
                secondName == null ? null : Hardware.get(hardwareMap, DcMotorEx.class, secondName),
                clock);
    }

    /** Builds on resolved motors ({@code null} for "not fitted"; the second is optional). */
    public Storage(DcMotorEx motor, DcMotorEx secondMotor, Clock clock) {
        this.transport = new VelocityMotor(motor, DcMotorSimple.Direction.FORWARD,
                DcMotor.ZeroPowerBehavior.BRAKE);
        this.transport2 = new VelocityMotor(secondMotor, SECOND_MOTOR_DIRECTION,
                DcMotor.ZeroPowerBehavior.BRAKE);
        this.clock = clock;
    }

    /** False when the transport motor is missing. Counting still works; motion no-ops. */
    public boolean isAvailable() {
        return transport.isAvailable();
    }

    // ---- Sensors ----

    /**
     * "A piece is at the storage entrance." Counted on the rising edge. {@code null} means no
     * entrance sensor is fitted: the count then only changes through {@link #setCount} (auto's
     * pre-loads, the auto-to-teleop hand-off) and the shooting macro's dead-reckoning, so a
     * teleop robot without one cannot know how many pieces it holds (see {@link #hasEntranceSensor()}).
     */
    public void setEntranceSupplier(BooleanSupplier supplier) {
        this.entranceSensorFitted = supplier != null;
        this.entranceSupplier = supplier == null ? () -> false : supplier;
    }

    /**
     * False when no entrance sensor is wired: the count cannot rise on its own, so a zero count
     * means "unknown", not "empty". {@code Macros.piecesOnBoard()} shoots blind in that case.
     */
    public boolean hasEntranceSensor() {
        return entranceSensorFitted;
    }

    /**
     * "The last slot is occupied." Makes {@link #isFull()} true regardless of the count.
     * {@code null} means no full sensor is fitted: only the count (or a manual {@link #setCount})
     * can then say full (see {@link #canDetectFull()}).
     */
    public void setFullSupplier(BooleanSupplier supplier) {
        this.fullSensorFitted = supplier != null;
        this.fullSupplier = supplier == null ? () -> false : supplier;
    }

    public boolean hasFullSensor() {
        return fullSensorFitted;
    }

    /**
     * Whether anything on this robot can report "full" by itself: an entrance sensor counts to
     * {@link #CAPACITY}, a full sensor sees the last slot. False on the sensorless robot, where an
     * "intake until full" is only ever a timed run and the operator sets the count by hand.
     */
    public boolean canDetectFull() {
        return entranceSensorFitted || fullSensorFitted;
    }

    /**
     * "A piece has reached the transfer." Counted out on the rising edge. {@code null} means no
     * exit sensor is fitted: the count then only goes up, and a shooting macro dead-reckons it
     * down (see {@link #hasExitSensor()}).
     */
    public void setExitSupplier(BooleanSupplier supplier) {
        this.exitSensorFitted = supplier != null;
        this.exitSupplier = supplier == null ? () -> false : supplier;
    }

    /** False when no exit sensor is wired, so callers know the count cannot decrement itself. */
    public boolean hasExitSensor() {
        return exitSensorFitted;
    }

    // ---- Count ----

    public int count() {
        return count;
    }

    public boolean hasPiece() {
        return count > 0;
    }

    /** True at {@link #CAPACITY}, or whenever the storage-full sensor says so. */
    public boolean isFull() {
        return count >= CAPACITY || fullSupplier.getAsBoolean();
    }

    /** Overrides the count, e.g. to the pre-load at init. Clamped to {@code [0, CAPACITY]}. */
    public void setCount(int pieces) {
        count = Math.max(0, Math.min(CAPACITY, pieces));
    }

    /** Records a piece entering (a missed sensor, or a manual correction). */
    public void markEntered() {
        setCount(count + 1);
        entryEvents++;
    }

    /** Records a piece leaving into the transfer. */
    public void markExited() {
        setCount(count - 1);
        exitEvents++;
    }

    public int getEntryEvents() {
        return entryEvents;
    }

    public int getExitEvents() {
        return exitEvents;
    }

    // ---- Motion ----

    public void advance() {
        setMode(Mode.ADVANCING, ADVANCE_TICKS_PER_SEC);
    }

    public void reverse() {
        setMode(Mode.REVERSING, REVERSE_TICKS_PER_SEC);
    }

    public void stop() {
        setMode(Mode.IDLE, 0);
    }

    private void setMode(Mode newMode, double ticksPerSec) {
        mode = newMode;
        targetVelocity = ticksPerSec;
    }

    public Mode getMode() {
        return mode;
    }

    public double getTargetVelocity() {
        return targetVelocity;
    }

    public double getVelocityTicksPerSec() {
        return transport.getVelocity();
    }

    public double getCurrentAmps() {
        return transport.getCurrentAmps() + transport2.getCurrentAmps();
    }

    /**
     * Counts edges and writes the motors. Edge detection runs regardless of mode, because a piece
     * can enter while the intake pushes and the transport is idle.
     */
    public void update() {
        boolean entrance = entranceSupplier.getAsBoolean();
        if (entrance && !lastEntrance) markEntered();
        lastEntrance = entrance;

        boolean exit = exitSupplier.getAsBoolean();
        if (exit && !lastExit) markExited();
        lastExit = exit;

        transport.write(targetVelocity);
        transport2.write(targetVelocity);
    }

    // ---- Ivy commands ----

    /** Runs the transport rearward until interrupted. */
    public Command advanceCommand() {
        return Command.build()
                .setStart(this::advance)
                .setDone(() -> false)
                .setEnd(ec -> stop())
                .requiring(this);
    }

    /** Runs the transport forward (toward the intake) until interrupted, to clear a jam. */
    public Command reverseCommand() {
        return Command.build()
                .setStart(this::reverse)
                .setDone(() -> false)
                .setEnd(ec -> stop())
                .requiring(this);
    }

    public Command stopCommand() {
        return Command.build()
                .setStart(this::stop)
                .setDone(() -> true)
                .requiring(this);
    }

    /**
     * Advances until one piece leaves into the transfer (an exit event), or
     * {@link #ADVANCE_TIMEOUT_MS} passes. Finishes at once when the storage is empty or
     * unavailable, so a scoring sequence does not sit spinning an empty channel.
     */
    public Command advanceOneCommand() {
        final int[] exitsAtStart = new int[1];
        return advanceUntilCommand(() -> exitEvents > exitsAtStart[0], ADVANCE_TIMEOUT_MS,
                () -> exitsAtStart[0] = exitEvents, () -> !hasPiece());
    }

    /**
     * Runs the transport rearward for {@code ms} on the injected clock. The sensorless per-shot
     * advance: without an exit sensor there is no edge to end on, so {@code Macros} meters the feed
     * by time and runs this together with the transfer, never into a stopped one.
     */
    public Command advanceForMsCommand(long ms) {
        return advanceUntilCommand(() -> false, ms);
    }

    /** Advances until {@code stop} is true or {@code timeoutMs} passes on the injected clock. */
    public Command advanceUntilCommand(BooleanSupplier stop, long timeoutMs) {
        return advanceUntilCommand(stop, timeoutMs, () -> { }, () -> false);
    }

    private Command advanceUntilCommand(BooleanSupplier stop, long timeoutMs, Runnable onStart,
                                        BooleanSupplier skip) {
        final long[] startedAt = new long[1];
        final boolean[] skipped = new boolean[1];
        return Command.build()
                .setStart(() -> {
                    startedAt[0] = clock.nowMs();
                    onStart.run();
                    skipped[0] = !isAvailable() || skip.getAsBoolean();
                    if (!skipped[0]) advance();
                })
                .setDone(() -> skipped[0]
                        || stop.getAsBoolean()
                        || clock.nowMs() - startedAt[0] >= timeoutMs)
                .setEnd(ec -> stop())
                .requiring(this);
    }

    /**
     * Schedule once at OpMode init: keeps the transport still whenever nothing else owns it.
     * Priority -1, SUSPEND and QUEUE for the same reasons as every other default command.
     */
    public Command defaultIdleCommand() {
        return Command.build()
                .setExecute(this::stop)
                .setDone(() -> false)
                .setEnd(ec -> stop())
                .setPriority(DEFAULT_IDLE_PRIORITY)
                .setInterruptedBehavior(InterruptedBehavior.SUSPEND)
                .setBlockedBehavior(BlockedBehavior.QUEUE)
                .requiring(this);
    }
}
