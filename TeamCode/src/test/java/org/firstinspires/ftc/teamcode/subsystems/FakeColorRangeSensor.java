package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

/**
 * A colour sensor that is also a distance sensor, the way a REV Color Sensor V3 is. Reports what
 * the test sets and counts the reads, so a test can prove {@code ColorSensor} caches once per loop.
 */
public final class FakeColorRangeSensor implements NormalizedColorSensor, DistanceSensor {
    public float red = 0, green = 0, blue = 0, alpha = 1;
    /** Inches. Far away by default, so a fresh fake sees nothing. */
    public double distanceInches = 100;
    public float gain = 1;
    public int colorReads = 0;
    public int distanceReads = 0;

    /** Sets a saturated colour by normalised RGB. */
    public FakeColorRangeSensor showing(float r, float g, float b) {
        red = r;
        green = g;
        blue = b;
        return this;
    }

    @Override
    public NormalizedRGBA getNormalizedColors() {
        colorReads++;
        NormalizedRGBA c = new NormalizedRGBA();
        c.red = red;
        c.green = green;
        c.blue = blue;
        c.alpha = alpha;
        return c;
    }

    @Override public float getGain() { return gain; }
    @Override public void setGain(float gain) { this.gain = gain; }

    @Override
    public double getDistance(DistanceUnit unit) {
        distanceReads++;
        return unit.fromInches(distanceInches);
    }

    @Override public Manufacturer getManufacturer() { return Manufacturer.Unknown; }
    @Override public String getDeviceName() { return "FakeColorRangeSensor"; }
    @Override public String getConnectionInfo() { return "fake"; }
    @Override public int getVersion() { return 1; }
    @Override public void resetDeviceConfigurationForOpMode() {}
    @Override public void close() {}
}
