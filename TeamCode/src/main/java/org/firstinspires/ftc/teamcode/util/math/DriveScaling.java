package org.firstinspires.ftc.teamcode.util.math;

/**
 * Stick shaping: deadband, exponential curve, and trigger-held slow mode.
 *
 * <p>Pure math, no hardware. Two entry points, because the teleop loop wants exactly two things
 * from a stick — a shaped axis and a slow-mode multiplier. The pieces used to be public and
 * separately overloaded, each with a no-argument twin that filled in the constants below; nothing
 * ever called the parameterised forms, so the curve now reads the tunables directly.
 *
 * <p>The tunables are {@code public static} so they can be turned on a dashboard between runs.
 * That is also why the guards against a nonsense deadband or threshold stay: the values are live.
 */
public final class DriveScaling {
    public static double DEFAULT_DEADBAND = 0.07;
    public static double DEFAULT_EXPO = 2.0;
    public static double SLOW_MIN_SCALE = 0.3;
    public static double SLOW_THRESHOLD = 0.1;

    private DriveScaling() {}

    /** Deadband then expo: what the drive gets from a raw stick axis. */
    public static double shape(double raw) {
        return applyExpo(applyDeadband(raw));
    }

    /**
     * Zeroes small stick noise, then rescales so output still reaches full magnitude.
     *
     * <p>The {@code /(1 - deadband)} is the point of the function: without it, output would jump
     * from 0 straight to {@code deadband} at the edge and never reach 1.0 at full deflection.
     */
    private static double applyDeadband(double raw) {
        double deadband = DEFAULT_DEADBAND;
        if (deadband >= 1) return 0;
        if (deadband < 0) deadband = 0;
        if (Math.abs(raw) < deadband) return 0;
        double sign = raw < 0 ? -1 : 1;
        return sign * (Math.abs(raw) - deadband) / (1 - deadband);
    }

    /** Signed power curve: fine control near centre, full authority at the ends. */
    private static double applyExpo(double raw) {
        double sign = raw < 0 ? -1 : 1;
        return sign * Math.pow(Math.abs(raw), DEFAULT_EXPO);
    }

    /**
     * Trigger-held precision mode: 1.0 below {@link #SLOW_THRESHOLD}, ramping down to
     * {@link #SLOW_MIN_SCALE} at full pull. Pulling harder makes the robot slower.
     */
    public static double slowScale(double trigger) {
        double threshold = SLOW_THRESHOLD;
        if (threshold >= 1) return 1.0;
        if (trigger < threshold) return 1.0;
        double t = (trigger - threshold) / (1.0 - threshold);
        return 1.0 - t * (1.0 - SLOW_MIN_SCALE);
    }
}
