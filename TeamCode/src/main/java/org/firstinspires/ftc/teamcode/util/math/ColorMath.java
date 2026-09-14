package org.firstinspires.ftc.teamcode.util.math;

/**
 * Colour conversion and matching. Pure math, no hardware — so it is unit-testable on a laptop.
 *
 * <p>Exists separately from the sensor wrapper because the FTC path
 * ({@code Color.colorToHSV(rgba.toColor(), ...)}) packs the normalized floats into an 8-bit ARGB
 * int first. That quantises to 1/255 steps and clips any channel above 1.0, so with a gain of 2.0
 * the value component pins at exactly 1.0 and stops discriminating right when the target is closest.
 */
public final class ColorMath {
    private ColorMath() {}

    /**
     * Converts normalized RGB to HSV.
     *
     * <p>Hue and saturation are ratios between channels, so they barely move with sensor gain.
     * Value is an absolute magnitude and does. Classify on hue; use saturation and value as gates.
     *
     * @param out receives {hue 0-360 degrees, saturation 0-1, value 0-1}
     */
    public static void toHsv(float r, float g, float b, float[] out) {
        float max = Math.max(r, Math.max(g, b));
        float min = Math.min(r, Math.min(g, b));
        float delta = max - min;

        if (delta <= 0 || max <= 0) {
            out[0] = 0;
        } else if (max == r) {
            out[0] = 60f * (((g - b) / delta) % 6f);
        } else if (max == g) {
            out[0] = 60f * (((b - r) / delta) + 2f);
        } else {
            out[0] = 60f * (((r - g) / delta) + 4f);
        }
        if (out[0] < 0) out[0] += 360f;

        out[1] = max <= 0 ? 0 : delta / max;
        out[2] = Math.min(1f, max);
    }

    /**
     * Shortest angular distance between two hues, in degrees (0-180).
     *
     * <p>Hue is circular: 350 and 10 are 20 degrees apart, not 340. Getting this wrong is why naive
     * red detection needs two separate windows.
     */
    public static float hueDistance(float a, float b) {
        float diff = Math.abs(a - b) % 360f;
        return Math.min(diff, 360f - diff);
    }

    /**
     * True when the HSV reading is close enough in hue and confident enough to act on.
     *
     * <p>The saturation and value floors are not decoration. Without a saturation floor a white or
     * grey surface passes any hue test, because hue is undefined when the channels are equal.
     * Without a value floor, shadow produces unstable hue from sensor noise.
     */
    public static boolean matches(float[] hsv, float hueDegrees, float tolerance,
                                  float minSaturation, float minValue) {
        if (hsv == null || hsv.length < 3) return false;
        if (hsv[1] < minSaturation || hsv[2] < minValue) return false;
        return hueDistance(hsv[0], hueDegrees) <= tolerance;
    }
}
