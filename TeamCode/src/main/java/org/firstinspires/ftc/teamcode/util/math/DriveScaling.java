package org.firstinspires.ftc.teamcode.util.math;

/**
 * Stick shaping: deadband, exponential curve, and trigger-held slow mode.
 *
 * <p>Pure math with no hardware dependency, which is why it is the one part of the robot code
 * covered by unit tests.
 */
public final class DriveScaling {
    public static double DEFAULT_DEADBAND = 0.07;
    public static double DEFAULT_EXPO = 2.0;
    public static double SLOW_MIN_SCALE = 0.3;
    public static double SLOW_THRESHOLD = 0.1;

    private DriveScaling() {}

    /**
     * Zeroes small stick noise, then rescales so output still reaches full magnitude.
     *
     * <p>The {@code /(1 - deadband)} is the point of the function: without it, output would jump
     * from 0 straight to {@code deadband} at the edge and never reach 1.0 at full deflection.
     */
    public static double applyDeadband(double raw, double deadband) {
        if (deadband >= 1) return 0;
        if (deadband < 0) deadband = 0;
        if (Math.abs(raw) < deadband) return 0;
        double sign = raw < 0 ? -1 : 1;
        return sign * (Math.abs(raw) - deadband) / (1 - deadband);
    }

    public static double applyDeadband(double raw) {
        return applyDeadband(raw, DEFAULT_DEADBAND);
    }

    public static double applyExpo(double raw, double exponent) {
        double sign = raw < 0 ? -1 : 1;
        return sign * Math.pow(Math.abs(raw), exponent);
    }

    public static double applyExpo(double raw) {
        return applyExpo(raw, DEFAULT_EXPO);
    }

    public static double shape(double raw) {
        return applyExpo(applyDeadband(raw));
    }

    /**
     * Trigger-held precision mode: 1.0 below the threshold, ramping down to {@code minScale} at
     * full pull. Pulling harder makes the robot slower.
     */
    public static double slowScale(double trigger, double minScale, double threshold) {
        if (threshold >= 1) return 1.0;
        if (trigger < threshold) return 1.0;
        double t = (trigger - threshold) / (1.0 - threshold);
        return 1.0 - t * (1.0 - minScale);
    }

    public static double slowScale(double trigger) {
        return slowScale(trigger, SLOW_MIN_SCALE, SLOW_THRESHOLD);
    }
}
