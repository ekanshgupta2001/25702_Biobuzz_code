package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;
import com.qualcomm.robotcore.hardware.SwitchableLight;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.util.hardware.Hardware;
import org.firstinspires.ftc.teamcode.util.hardware.HardwareNames;
import org.firstinspires.ftc.teamcode.util.math.ColorMath;

/**
 * Wrapper over {@link NormalizedColorSensor} that caches one reading per loop. Instantiable per
 * sensor point ({@code HardwareNames.SENSOR_*}); the default is the storage entrance, where a piece
 * is identified as POLLEN or NECTAR.
 *
 * <p>HSV is computed directly from the normalized floats rather than via
 * {@code Color.colorToHSV(rgba.toColor(), ...)}. That SDK path packs the channels into an 8-bit
 * ARGB int, which quantises to 1/255 steps and — with any gain above 1.0 — <em>clips</em> bright
 * readings to 255. The result is that {@code value} pins at exactly 1.0 and stops discriminating
 * precisely when the target is closest.
 *
 * <p>Hue and saturation are ratios between channels, so they are largely gain-invariant. Value is
 * an absolute magnitude, so it moves with gain and distance. <b>Classify on hue, gate on
 * saturation and value</b> — see {@link #matchesHue}.
 */
public class ColorSensor {
    /** Config name used by the one-argument constructor; other sensor points pass their own name. */
    public static final String DEFAULT_NAME = HardwareNames.SENSOR_STORAGE_ENTRANCE;

    /**
     * Multiplies the raw signal before normalisation. Practical range 1.0-4.0: raise it in dim
     * conditions, lower it if all channels read near 1.0 and colours stop separating.
     */
    public static float DEFAULT_GAIN = 2f;

    private final NormalizedColorSensor sensor;
    private final float[] hsv = new float[3];
    private NormalizedRGBA colors = new NormalizedRGBA();
    /** Cached once per {@link #update()} when the device is also a {@link DistanceSensor}. */
    private double distanceInches = Double.NaN;

    public ColorSensor(HardwareMap hardwareMap) {
        this(hardwareMap, DEFAULT_NAME, DEFAULT_GAIN, true);
    }

    public ColorSensor(HardwareMap hardwareMap, String name, float gain, boolean lightOn) {
        this(Hardware.get(hardwareMap, NormalizedColorSensor.class, name), gain, lightOn);
    }

    /** Builds on an already-resolved device ({@code null} for "not fitted"). Tests inject a fake. */
    public ColorSensor(NormalizedColorSensor device, float gain, boolean lightOn) {
        sensor = device;
        if (sensor == null) return;
        sensor.setGain(gain);
        if (sensor instanceof SwitchableLight) {
            ((SwitchableLight) sensor).enableLight(lightOn);
        }
    }

    /** False when the sensor is missing from the robot configuration. All reads then return 0. */
    public boolean isAvailable() {
        return sensor != null;
    }

    /**
     * One distance read on a device that has one and, when {@code colour} is true, one colour
     * read. Each is its own I2C transaction (external I2C is not in the hub's bulk cache), so the
     * loop asks only for what it consumes: presence is judged by distance wherever a sensor has
     * it, and the hue is read only where it classifies ({@code Robot.readSensors()}).
     */
    public void update(boolean colour) {
        if (sensor == null) return;
        if (colour) {
            NormalizedRGBA reading = sensor.getNormalizedColors();
            if (reading != null) {
                colors = reading;
                ColorMath.toHsv(colors.red, colors.green, colors.blue, hsv);
            }
        }
        if (sensor instanceof DistanceSensor) {
            distanceInches = ((DistanceSensor) sensor).getDistance(DistanceUnit.INCH);
        }
    }

    /** Both halves: what the benches read, and any caller that wants everything. */
    public void update() {
        update(true);
    }

    /**
     * True when the reading sits within {@code tolerance} degrees of {@code hueDegrees} and is
     * saturated and bright enough to be trustworthy.
     *
     * <p>The saturation and value floors are not optional. Without a saturation floor, a white or
     * grey surface passes <em>any</em> hue test — hue is meaningless when the channels are equal.
     * Without a value floor, deep shadow produces unstable hue from sensor noise.
     *
     * <p>Handles hue wraparound, so red (~0 degrees) works without two separate windows.
     */
    public boolean matchesHue(float hueDegrees, float tolerance, float minSaturation, float minValue) {
        return sensor != null
                && ColorMath.matches(hsv, hueDegrees, tolerance, minSaturation, minValue);
    }

    public float getRed() {
        return colors.red;
    }

    public float getGreen() {
        return colors.green;
    }

    public float getBlue() {
        return colors.blue;
    }

    public float getAlpha() {
        return colors.alpha;
    }

    public float getHue() {
        return hsv[0];
    }

    public float getSaturation() {
        return hsv[1];
    }

    public float getValue() {
        return hsv[2];
    }

    /** Returns a copy — the internal array is overwritten in place every {@link #update()}. */
    public float[] getHsv() {
        return new float[] {hsv[0], hsv[1], hsv[2]};
    }

    public void setGain(float gain) {
        if (sensor != null) sensor.setGain(gain);
    }

    public float getGain() {
        return sensor == null ? 0 : sensor.getGain();
    }

    public boolean hasLight() {
        return sensor instanceof SwitchableLight;
    }

    public void setLight(boolean on) {
        if (sensor instanceof SwitchableLight) {
            ((SwitchableLight) sensor).enableLight(on);
        }
    }

    public boolean isLightOn() {
        return sensor instanceof SwitchableLight && ((SwitchableLight) sensor).isLightOn();
    }

    public boolean hasDistance() {
        return sensor instanceof DistanceSensor;
    }

    /**
     * The distance cached by the last {@link #update()}, in inches; {@code NaN} without a distance
     * sensor or before the first read. This is the presence signal: a REV Color Sensor V3 reads
     * proximity without caring what colour the piece is or how the pit lighting drifted.
     */
    public double getDistanceInches() {
        return distanceInches;
    }

    /** A live read in any unit (one more I2C transaction). Prefer {@link #getDistanceInches()}. */
    public double getDistance(DistanceUnit unit) {
        return sensor instanceof DistanceSensor ? ((DistanceSensor) sensor).getDistance(unit) : Double.NaN;
    }
}
