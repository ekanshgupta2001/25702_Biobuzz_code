package org.firstinspires.ftc.teamcode.subsystems;

import com.pedropathing.drivetrain.DrivePowers;

import java.util.Collections;
import java.util.Map;

/** Pedro's motor-layer interface, recording what it was told. */
public final class FakePedroDrivetrain implements com.pedropathing.drivetrain.Drivetrain {
    public DrivePowers lastPowers = null;
    public boolean lastManual = false;
    public int driveCalls = 0;
    public int stopCalls = 0;
    /** True between a non-zero drive and the next stop. */
    public boolean moving = false;

    @Override
    public void drive(DrivePowers powers, boolean manual) {
        lastPowers = powers;
        lastManual = manual;
        driveCalls++;
        moving = powers.forward() != 0 || powers.strafe() != 0 || powers.turn() != 0;
    }

    @Override
    public double maxScaling(DrivePowers current, DrivePowers delta) {
        return 1;
    }

    @Override
    public void stop() {
        stopCalls++;
        moving = false;
        lastPowers = DrivePowers.zero();
    }

    @Override
    public void stop(boolean brake) {
        stop();
    }

    @Override
    public Map<String, Object> debug() {
        return Collections.emptyMap();
    }

    @Override
    public double interpolateVelocity(double xRadius, double yRadius, double theta) {
        return 0;
    }
}
