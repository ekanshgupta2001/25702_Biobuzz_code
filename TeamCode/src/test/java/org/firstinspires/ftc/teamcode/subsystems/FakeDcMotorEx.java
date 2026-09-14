package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.DcMotorController;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.PIDCoefficients;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.configuration.typecontainers.MotorConfigurationType;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;

/**
 * A {@link DcMotorEx} that remembers what it was told and reports what a test sets.
 *
 * <p>Writes are recorded (velocity, power, mode, target, direction); reads come from public fields
 * the test controls (position, measured velocity, current). Nothing simulates physics; that is the
 * test's job.
 */
public final class FakeDcMotorEx implements DcMotorEx {
    // ---- What the code under test wrote ----
    public double commandedVelocity = 0;
    public double commandedPower = 0;
    public int targetPosition = 0;
    public int targetTolerance = 0;
    public RunMode mode = RunMode.RUN_WITHOUT_ENCODER;
    public ZeroPowerBehavior zeroPowerBehavior = ZeroPowerBehavior.UNKNOWN;
    public Direction direction = Direction.FORWARD;
    public int velocityWrites = 0;
    public int powerWrites = 0;
    /** Most negative velocity ever commanded; tests use it to ask "did it ever run backwards?". */
    public double minCommandedVelocity = 0;
    /** Most positive velocity ever commanded. */
    public double maxCommandedVelocity = 0;

    // ---- What the test wants the motor to report ----
    public int currentPosition = 0;
    public double measuredVelocity = 0;
    public double currentAmps = 0;

    @Override
    public void setVelocity(double angularRate) {
        commandedVelocity = angularRate;
        velocityWrites++;
        minCommandedVelocity = Math.min(minCommandedVelocity, angularRate);
        maxCommandedVelocity = Math.max(maxCommandedVelocity, angularRate);
    }

    @Override
    public void setVelocity(double angularRate, AngleUnit unit) {
        setVelocity(angularRate);
    }

    @Override
    public double getVelocity() {
        return measuredVelocity;
    }

    @Override
    public double getVelocity(AngleUnit unit) {
        return measuredVelocity;
    }

    @Override
    public void setPower(double power) {
        commandedPower = power;
        powerWrites++;
    }

    @Override
    public double getPower() {
        return commandedPower;
    }

    @Override
    public void setTargetPosition(int position) {
        targetPosition = position;
    }

    @Override
    public int getTargetPosition() {
        return targetPosition;
    }

    @Override
    public void setTargetPositionTolerance(int tolerance) {
        targetTolerance = tolerance;
    }

    @Override
    public int getTargetPositionTolerance() {
        return targetTolerance;
    }

    @Override
    public int getCurrentPosition() {
        return currentPosition;
    }

    @Override
    public double getCurrent(CurrentUnit unit) {
        return currentAmps;
    }

    @Override
    public void setMode(RunMode mode) {
        this.mode = mode;
    }

    @Override
    public RunMode getMode() {
        return mode;
    }

    @Override
    public void setZeroPowerBehavior(ZeroPowerBehavior behavior) {
        zeroPowerBehavior = behavior;
    }

    @Override
    public ZeroPowerBehavior getZeroPowerBehavior() {
        return zeroPowerBehavior;
    }

    @Override
    public void setDirection(Direction direction) {
        this.direction = direction;
    }

    @Override
    public Direction getDirection() {
        return direction;
    }

    @Override
    public boolean isBusy() {
        return Math.abs(currentPosition - targetPosition) > targetTolerance;
    }

    // ---- Inert remainder of the interface ----

    @Override public void setMotorEnable() {}
    @Override public void setMotorDisable() {}
    @Override public boolean isMotorEnabled() { return true; }
    @Override public void setPIDCoefficients(RunMode mode, PIDCoefficients pid) {}
    @Override public void setPIDFCoefficients(RunMode mode, PIDFCoefficients pidf) {}
    @Override public void setVelocityPIDFCoefficients(double p, double i, double d, double f) {}
    @Override public void setPositionPIDFCoefficients(double p) {}
    @Override public PIDCoefficients getPIDCoefficients(RunMode mode) { return null; }
    @Override public PIDFCoefficients getPIDFCoefficients(RunMode mode) { return null; }
    @Override public double getCurrentAlert(CurrentUnit unit) { return 0; }
    @Override public void setCurrentAlert(double current, CurrentUnit unit) {}
    @Override public boolean isOverCurrent() { return false; }
    @Override public MotorConfigurationType getMotorType() { return null; }
    @Override public void setMotorType(MotorConfigurationType type) {}
    @Override public DcMotorController getController() { return null; }
    @Override public int getPortNumber() { return 0; }
    @Override public void setPowerFloat() {}
    @Override public boolean getPowerFloat() { return false; }
    @Override public Manufacturer getManufacturer() { return Manufacturer.Unknown; }
    @Override public String getDeviceName() { return "FakeDcMotorEx"; }
    @Override public String getConnectionInfo() { return "fake"; }
    @Override public int getVersion() { return 1; }
    @Override public void resetDeviceConfigurationForOpMode() {}
    @Override public void close() {}
}
