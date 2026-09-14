package org.firstinspires.ftc.teamcode.subsystems;

import com.pedropathing.drivetrain.DrivePowers;
import com.pedropathing.ivy.Command;
import com.pedropathing.ivy.behaviors.BlockedBehavior;
import com.pedropathing.ivy.behaviors.InterruptedBehavior;
import com.pedropathing.revhub.drivetrains.Mecanum;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.pedro.Constants;
import org.firstinspires.ftc.teamcode.util.hardware.Hardware;
import org.firstinspires.ftc.teamcode.util.time.Clock;

/**
 * Open-loop, timed driving through Pedro's motor layer, for before the follower is tuned.
 *
 * <p>Pedro's {@code Mecanum} needs only motor names and directions ({@code Constants.drivetrainConfig}),
 * not a localizer or a tuned Foresight, so a hardcoded autonomous can move the robot without any
 * of the tuning the {@link Drivetrain} depends on. Powers are Pedro's robot-frame convention:
 * {@code +forward} ahead, {@code +strafe} left, {@code +turn} counter-clockwise.
 *
 * <p>Intent-then-write, and the write happens <b>only when the intent changes</b>: once stopped it
 * writes zero once and then stays silent, so an idle OpenLoopDrive never fights the tuned follower
 * that will one day share these motors. Fails soft like every subsystem.
 */
public class OpenLoopDrive {
    public static int DEFAULT_STOP_PRIORITY = -1;

    private final com.pedropathing.drivetrain.Drivetrain motors;
    private final Clock clock;
    private double forward = 0;
    private double strafe = 0;
    private double turn = 0;
    private boolean dirty = false;

    public OpenLoopDrive(HardwareMap hardwareMap, Clock clock) {
        com.pedropathing.drivetrain.Drivetrain built = null;
        try {
            built = new Mecanum(hardwareMap, Constants.drivetrainConfig);
        } catch (RuntimeException e) {
            // A missing drive motor name must not kill the OpMode; the auto then just shoots.
            Hardware.recordFailure("open_loop_drive", "Mecanum: " + e.getMessage());
        }
        this.motors = built;
        this.clock = clock;
    }

    /** Builds on an already-constructed Pedro drivetrain, or {@code null} for "not fitted". */
    public OpenLoopDrive(com.pedropathing.drivetrain.Drivetrain motors, Clock clock) {
        this.motors = motors;
        this.clock = clock;
    }

    public boolean isAvailable() {
        return motors != null;
    }

    /** Sets the requested powers; nothing reaches the motors until {@link #update()}. */
    public void drive(double forward, double strafe, double turn) {
        if (forward == this.forward && strafe == this.strafe && turn == this.turn) return;
        this.forward = forward;
        this.strafe = strafe;
        this.turn = turn;
        dirty = true;
    }

    public void stop() {
        drive(0, 0, 0);
    }

    public boolean isMoving() {
        return forward != 0 || strafe != 0 || turn != 0;
    }

    public double getForward() {
        return forward;
    }

    public double getStrafe() {
        return strafe;
    }

    public double getTurn() {
        return turn;
    }

    /** Writes the intent if it changed since the last write. From {@code Robot.writeActuators()}. */
    public void update() {
        if (motors == null || !dirty) return;
        dirty = false;
        if (isMoving()) {
            motors.drive(new DrivePowers(forward, strafe, turn), true);
        } else {
            motors.stop();
        }
    }

    // ---- Ivy commands ----

    /** Drives at fixed powers for {@code ms} on the injected clock, then stops. */
    public Command driveForMsCommand(double forward, double strafe, double turn, long ms) {
        final long[] startedAt = new long[1];
        return Command.build()
                .setStart(() -> {
                    startedAt[0] = clock.nowMs();
                    drive(forward, strafe, turn);
                })
                .setDone(() -> clock.nowMs() - startedAt[0] >= ms)
                .setEnd(ec -> stop())
                .requiring(this);
    }

    /** Schedule once at OpMode init: keeps the motors stopped whenever nothing else owns them. */
    public Command defaultStopCommand() {
        return Command.build()
                .setExecute(this::stop)
                .setDone(() -> false)
                .setEnd(ec -> stop())
                .setPriority(DEFAULT_STOP_PRIORITY)
                .setInterruptedBehavior(InterruptedBehavior.SUSPEND)
                .setBlockedBehavior(BlockedBehavior.QUEUE)
                .requiring(this);
    }
}
