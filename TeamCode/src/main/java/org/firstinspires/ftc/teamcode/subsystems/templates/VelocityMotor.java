package org.firstinspires.ftc.teamcode.subsystems.templates;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
import org.firstinspires.ftc.teamcode.util.hardware.Hardware;

/**
 * A velocity-controlled motor: the shared wrapper for every roller and flywheel on the robot
 * (intake, storage transport, vertical transfer, shooter).
 *
 * <h2>Intent, then write</h2>
 * {@link #setTarget} only records what the mechanism wants. {@link #update()} is the single place
 * the hardware is written, once per loop from the owning subsystem's {@code update()}, so the
 * motor stays under one authority no matter how many commands are fighting, and an anti-jam
 * override can replace the request on its way out with {@link #write}.
 *
 * <p>Velocities are in encoder ticks per second, the unit {@link DcMotorEx#setVelocity} uses.
 * Ticks per revolution differ per motor part and must be measured: pairing the 312 RPM part's
 * 537.7 ticks/rev with a 435 RPM motor overstates the ceiling by ~40%, saturates the velocity
 * loop and trips every stall detector during normal running.
 *
 * <h2>The bus is the budget</h2>
 * Every {@code setVelocity} is its own transaction to the hub, and six of these per loop for a
 * value that has not changed is the largest avoidable cost in {@code Robot.writeActuators()}
 * (fixthese R2-A1). {@link #write} therefore sends only when the value differs from the last one
 * sent, and re-sends an unchanged value every {@link #REFRESH_EVERY_N_WRITES} loops anyway, so a
 * hub that browned out and forgot its targets recovers within a quarter second.
 *
 * <p>Only the {@code Shooter} uses {@link #setTarget}/{@link #update()} and therefore
 * {@link #atSpeed}; the intake, storage and transfer keep their own target and call {@link #write}
 * directly, so on those {@code getTarget()} is 0 and {@code atSpeed} is always false.
 */
public final class VelocityMotor {
    /**
     * An unchanged velocity is still re-sent this often (in loops; about 250 ms at 20 ms loops).
     * A hub reset mid-match loses its targets; without the refresh a held speed would never come
     * back until it changed.
     */
    public static int REFRESH_EVERY_N_WRITES = 12;

    private final DcMotorEx motor;
    private double target = 0;
    private double lastWritten = Double.NaN;
    private int writesSinceSent = 0;

    /** Resolves {@code name} fail-soft; the result is unavailable when the name is missing. */
    public static VelocityMotor fromHardware(HardwareMap hardwareMap, String name,
                                             DcMotorSimple.Direction direction,
                                             DcMotor.ZeroPowerBehavior zeroPower) {
        return new VelocityMotor(Hardware.get(hardwareMap, DcMotorEx.class, name), direction, zeroPower);
    }

    public VelocityMotor(DcMotorEx motor) {
        this(motor, DcMotorSimple.Direction.FORWARD, DcMotor.ZeroPowerBehavior.FLOAT);
    }

    /** Builds on an already-resolved motor ({@code null} for "not fitted"). Tests inject a fake. */
    public VelocityMotor(DcMotorEx motor, DcMotorSimple.Direction direction,
                         DcMotor.ZeroPowerBehavior zeroPower) {
        this.motor = motor;
        if (motor == null) return;
        motor.setDirection(direction);
        motor.setZeroPowerBehavior(zeroPower);
        motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    /** False when the motor is missing from the robot configuration. All calls then no-op. */
    public boolean isAvailable() {
        return motor != null;
    }

    /** Records the requested velocity. Nothing reaches the motor until {@link #update()}. */
    public void setTarget(double ticksPerSec) {
        target = ticksPerSec;
    }

    public double getTarget() {
        return target;
    }

    public void stop() {
        target = 0;
    }

    /** Measured velocity in ticks per second, or 0 when unavailable. */
    public double getVelocity() {
        return motor == null ? 0 : motor.getVelocity();
    }

    public double getCurrentAmps() {
        return motor == null ? 0 : motor.getCurrent(CurrentUnit.AMPS);
    }

    /**
     * True when the measured velocity is within {@code toleranceTicksPerSec} of a non-zero target.
     * A zero target is never "at speed", so a spin-up wait cannot pass on a stopped motor.
     */
    public boolean atSpeed(double toleranceTicksPerSec) {
        return motor != null && target != 0
                && Math.abs(motor.getVelocity() - target) <= toleranceTicksPerSec;
    }

    /** Writes the recorded target. Call once per loop. */
    public void update() {
        write(target);
    }

    /**
     * Writes a specific velocity this loop instead of the target (anti-jam reversal). Reaches the
     * bus only when the value changed, or every {@link #REFRESH_EVERY_N_WRITES} calls regardless.
     */
    public void write(double ticksPerSec) {
        if (motor == null) return;
        boolean changed = Double.isNaN(lastWritten) || ticksPerSec != lastWritten;
        if (!changed && ++writesSinceSent < REFRESH_EVERY_N_WRITES) return;
        motor.setVelocity(ticksPerSec);
        lastWritten = ticksPerSec;
        writesSinceSent = 0;
    }

    /**
     * Replaces the hub's velocity-loop gains for this motor. The SDK's defaults depend on the motor
     * type chosen in the Robot Controller configuration and are tuned for geared drive motors; a
     * high-inertia flywheel usually needs its own. F is the feedforward: {@code 32767 / maxTicksPerSec}.
     */
    public void setVelocityPidf(double p, double i, double d, double f) {
        if (motor != null) motor.setVelocityPIDFCoefficients(p, i, d, f);
    }
}
