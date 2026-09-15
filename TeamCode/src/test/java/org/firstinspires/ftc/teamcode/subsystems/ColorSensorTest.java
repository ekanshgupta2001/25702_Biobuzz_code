package org.firstinspires.ftc.teamcode.subsystems;

import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertFalse;
import static org.junit.Assert.assertTrue;

import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;

import org.firstinspires.ftc.teamcode.game.PieceType;
import org.firstinspires.ftc.teamcode.util.field.Alliance;
import org.junit.Test;

/** The wrapper's caching contract and the colour maths it feeds, against a fake REV V3. */
public class ColorSensorTest {
    private static final double EPS = 1e-6;

    @Test
    public void distanceIsReadOncePerUpdateAndCached() {
        FakeColorRangeSensor device = new FakeColorRangeSensor();
        device.distanceInches = 1.5;
        ColorSensor sensor = new ColorSensor(device, 2f, true);
        assertTrue(sensor.hasDistance());
        assertTrue("nothing read before the first update", Double.isNaN(sensor.getDistanceInches()));

        sensor.update();
        assertEquals(1.5, sensor.getDistanceInches(), EPS);
        sensor.getDistanceInches();
        sensor.getDistanceInches();
        assertEquals("one I2C read per loop, however often it is asked", 1, device.distanceReads);
        assertEquals(1, device.colorReads);
        assertEquals("gain applied at construction", 2f, device.gain, EPS);
    }

    @Test
    public void noDistanceWithoutADistanceSensor() {
        NormalizedColorSensor colourOnly = new NormalizedColorSensor() {
            @Override public NormalizedRGBA getNormalizedColors() { return new NormalizedRGBA(); }
            @Override public float getGain() { return 1; }
            @Override public void setGain(float gain) {}
            @Override public Manufacturer getManufacturer() { return Manufacturer.Unknown; }
            @Override public String getDeviceName() { return "colour only"; }
            @Override public String getConnectionInfo() { return ""; }
            @Override public int getVersion() { return 1; }
            @Override public void resetDeviceConfigurationForOpMode() {}
            @Override public void close() {}
        };
        ColorSensor sensor = new ColorSensor(colourOnly, 2f, true);
        sensor.update();
        assertFalse(sensor.hasDistance());
        assertTrue(Double.isNaN(sensor.getDistanceInches()));
        assertFalse("no light to switch either", sensor.hasLight());
    }

    @Test
    public void classifiesNectarByAllianceFromTheHue() {
        FakeColorRangeSensor device = new FakeColorRangeSensor().showing(0.1f, 0.1f, 0.9f);   // blue
        ColorSensor sensor = new ColorSensor(device, 2f, true);
        sensor.update();
        assertEquals(PieceType.NECTAR, PieceType.classify(sensor));
        assertEquals(Alliance.BLUE, PieceType.nectarAllianceAt(sensor));

        device.showing(0.9f, 0.1f, 0.1f);                                                    // red
        sensor.update();
        assertEquals(Alliance.RED, PieceType.nectarAllianceAt(sensor));

        device.showing(0.9f, 0.8f, 0.1f);                                                    // yellow-ish
        sensor.update();
        assertEquals(PieceType.POLLEN, PieceType.classify(sensor));
        assertEquals("POLLEN belongs to nobody", null, PieceType.nectarAllianceAt(sensor));

        device.showing(0.5f, 0.5f, 0.5f);                                                    // grey tile
        sensor.update();
        assertEquals("unsaturated: no piece, whatever the hue", null, PieceType.classify(sensor));
    }
}
