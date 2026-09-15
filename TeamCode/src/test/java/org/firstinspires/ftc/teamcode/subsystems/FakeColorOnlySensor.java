package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;

/**
 * A colour sensor with no distance reading, the way a beam-break-less colour-only device is. The
 * case {@code Robot} must not trust for counting until the hue windows are measured.
 */
public final class FakeColorOnlySensor implements NormalizedColorSensor {
    public float red = 0, green = 0, blue = 0, alpha = 1;
    public float gain = 1;
    public int colorReads = 0;

    /** Sets a saturated colour by normalised RGB. */
    public FakeColorOnlySensor showing(float r, float g, float b) {
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
    @Override public Manufacturer getManufacturer() { return Manufacturer.Unknown; }
    @Override public String getDeviceName() { return "FakeColorOnlySensor"; }
    @Override public String getConnectionInfo() { return "fake"; }
    @Override public int getVersion() { return 1; }
    @Override public void resetDeviceConfigurationForOpMode() {}
    @Override public void close() {}
}
