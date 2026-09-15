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
 * The rear 90-degree vertical transfer: opposing compliant wheels that lift one piece from the
 * back of the storage directly into the shooter feed.
 *
 * <h2>Two jobs, two interlocks</h2>
 * {@link #liftOneCommand()} raises the next piece until it is staged at the shooter feed; it
 * refuses to start while a piece is already there, so the transfer never double-feeds.
 * {@link #feedCommand()} pushes the staged piece into the flywheel and finishes when the feed
 * sensor clears (plus a short dwell); the macro that composes it is responsible for spinning the
 * shooter up first. With no feed sensor wired, both fall back to a timed pulse so the robot still
 * cycles on a bench without sensors.
 */
public class Transfer {
    public static double LIFT_TICKS_PER_SEC = 1500;
    public static double FEED_TICKS_PER_SEC = 2000;
    public static double REVERSE_TICKS_PER_SEC = -1000;
    /** A lift that has not staged a piece by then gives up (nothing in the channel, or a jam). */
    public static long LIFT_TIMEOUT_MS = 2000;
    /** Open-loop lift duration when no feed sensor is wired. */
    public static long LIFT_PULSE_MS = 600;
    /** A feed that has not cleared the sensor by then gives up. */
    public static long FEED_TIMEOUT_MS = 1000;
    /** Open-loop feed duration when no feed sensor is wired. */
    public static long FEED_PULSE_MS = 300;
    /** Keep pushing this long after the feed sensor clears, so the piece fully enters the wheels. */
    public static long FEED_CLEAR_DWELL_MS = 100;
    public static int DEFAULT_IDLE_PRIORITY = -1;

    public enum Mode { IDLE, LIFTING, FEEDING, REVERSING }

    private final VelocityMotor lift;
    private final Clock clock;
    private Mode mode = Mode.IDLE;
    private double targetVelocity = 0;

    private BooleanSupplier inLiftSupplier = () -> false;
    private BooleanSupplier atFeedSupplier = null;

    public Transfer(HardwareMap hardwareMap) {
        this(hardwareMap, HardwareNames.TRANSFER_MOTOR, Clock.system());
    }

    public Transfer(HardwareMap hardwareMap, String name, Clock clock) {
        this(Hardware.get(hardwareMap, DcMotorEx.class, name), clock);
    }

    /** Builds on an already-resolved motor ({@code null} for "not fitted"). */
    public Transfer(DcMotorEx motor, Clock clock) {
        this.lift = new VelocityMotor(motor, DcMotorSimple.Direction.FORWARD,
                DcMotor.ZeroPowerBehavior.BRAKE);
        this.clock = clock;
    }

    public boolean isAvailable() {
        return lift.isAvailable();
    }

    /** "A piece is in the vertical channel." Telemetry and the storage exit count use it. */
    public void setInLiftSupplier(BooleanSupplier supplier) {
        this.inLiftSupplier = supplier == null ? () -> false : supplier;
    }

    /** "A piece is staged at the shooter feed." {@code null} means no sensor: use timed pulses. */
    public void setAtFeedSupplier(BooleanSupplier supplier) {
        this.atFeedSupplier = supplier;
    }

    public boolean hasFeedSensor() {
        return atFeedSupplier != null;
    }

    public boolean hasPieceInLift() {
        return inLiftSupplier.getAsBoolean();
    }

    /** False without a feed sensor. */
    public boolean pieceAtFeed() {
        return atFeedSupplier != null && atFeedSupplier.getAsBoolean();
    }

    public void liftPiece() {
        setMode(Mode.LIFTING, LIFT_TICKS_PER_SEC);
    }

    public void feed() {
        setMode(Mode.FEEDING, FEED_TICKS_PER_SEC);
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
        return lift.getVelocity();
    }

    public double getCurrentAmps() {
        return lift.getCurrentAmps();
    }

    public void update() {
        lift.write(targetVelocity);
    }

    // ---- Ivy commands ----

    /**
     * Lifts the next piece until it is staged at the feed, or {@link #LIFT_TIMEOUT_MS}. Finishes
     * at once if a piece is already staged (never double-feed) or the transfer is unavailable.
     * Without a feed sensor, runs for {@link #LIFT_PULSE_MS}.
     */
    public Command liftOneCommand() {
        final long[] startedAt = new long[1];
        final boolean[] skipped = new boolean[1];
        return Command.build()
                .setStart(() -> {
                    startedAt[0] = clock.nowMs();
                    skipped[0] = !isAvailable() || pieceAtFeed();
                    if (!skipped[0]) liftPiece();
                })
                .setDone(() -> {
                    if (skipped[0]) return true;
                    long elapsed = clock.nowMs() - startedAt[0];
                    if (!hasFeedSensor()) return elapsed >= LIFT_PULSE_MS;
                    return pieceAtFeed() || elapsed >= LIFT_TIMEOUT_MS;
                })
                .setEnd(ec -> stop())
                .requiring(this);
    }

    /**
     * Pushes the staged piece into the flywheel. With a feed sensor: runs until the sensor clears,
     * then {@link #FEED_CLEAR_DWELL_MS} more, or {@link #FEED_TIMEOUT_MS}; finishes at once if
     * nothing is staged. Without one: a {@link #FEED_PULSE_MS} pulse. The caller must hold the
     * shooter at speed for the whole command.
     */
    public Command feedCommand() {
        final long[] startedAt = new long[1];
        final long[] clearedAt = new long[1];
        final boolean[] skipped = new boolean[1];
        return Command.build()
                .setStart(() -> {
                    startedAt[0] = clock.nowMs();
                    clearedAt[0] = -1;
                    skipped[0] = !isAvailable() || (hasFeedSensor() && !pieceAtFeed());
                    if (!skipped[0]) feed();
                })
                .setDone(() -> {
                    if (skipped[0]) return true;
                    long now = clock.nowMs();
                    if (!hasFeedSensor()) return now - startedAt[0] >= FEED_PULSE_MS;
                    if (clearedAt[0] < 0 && !pieceAtFeed()) clearedAt[0] = now;
                    if (clearedAt[0] >= 0 && now - clearedAt[0] >= FEED_CLEAR_DWELL_MS) return true;
                    return now - startedAt[0] >= FEED_TIMEOUT_MS;
                })
                .setEnd(ec -> stop())
                .requiring(this);
    }

    /**
     * Runs at feed speed for {@code ms} on the injected clock, sensors ignored: the metered
     * sensorless feed ({@code Macros.SENSORLESS_FEED_PULSE_MS}). Finishes at once when not fitted.
     */
    public Command feedForMsCommand(long ms) {
        final long[] startedAt = new long[1];
        return Command.build()
                .setStart(() -> {
                    startedAt[0] = clock.nowMs();
                    if (isAvailable()) feed();
                })
                .setDone(() -> !isAvailable() || clock.nowMs() - startedAt[0] >= ms)
                .setEnd(ec -> stop())
                .requiring(this);
    }

    /** Runs the lift upward until interrupted. */
    public Command liftCommand() {
        return Command.build()
                .setStart(this::liftPiece)
                .setDone(() -> false)
                .setEnd(ec -> stop())
                .requiring(this);
    }

    /** Runs the lift downward until interrupted, to clear a jam. */
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

    /** Schedule once at OpMode init: keeps the transfer still whenever nothing else owns it. */
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
