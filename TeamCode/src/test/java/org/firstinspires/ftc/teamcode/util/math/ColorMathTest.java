package org.firstinspires.ftc.teamcode.util.math;

import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertFalse;
import static org.junit.Assert.assertTrue;

import org.junit.Test;

/** Unit tests for hue/saturation math, including the wraparound that naive code gets wrong. */
public class ColorMathTest {
    private static final float EPS = 0.01f;

    private static float[] hsvOf(float r, float g, float b) {
        float[] out = new float[3];
        ColorMath.toHsv(r, g, b, out);
        return out;
    }

    @Test
    public void primaryColoursLandOnKnownHues() {
        assertEquals(0f, hsvOf(1f, 0f, 0f)[0], EPS);     // red
        assertEquals(120f, hsvOf(0f, 1f, 0f)[0], EPS);   // green
        assertEquals(240f, hsvOf(0f, 0f, 1f)[0], EPS);   // blue
        assertEquals(60f, hsvOf(1f, 1f, 0f)[0], EPS);    // yellow
    }

    @Test
    public void greyHasZeroSaturation() {
        assertEquals(0f, hsvOf(0.5f, 0.5f, 0.5f)[1], EPS);
        assertEquals(0f, hsvOf(1f, 1f, 1f)[1], EPS);
    }

    @Test
    public void blackIsAllZero() {
        float[] hsv = hsvOf(0f, 0f, 0f);
        assertEquals(0f, hsv[1], EPS);
        assertEquals(0f, hsv[2], EPS);
    }

    @Test
    public void hueSurvivesGainWhereThe8BitPathWouldClip() {
        // Same colour, different gain. Channels above 1.0 would clip to 255 in the SDK's ARGB
        // round-trip and destroy the ratio; here the hue must stay put.
        float[] low = hsvOf(0.4f, 0.4f, 0.0f);
        float[] high = hsvOf(1.6f, 1.6f, 0.0f);
        assertEquals(low[0], high[0], EPS);
        assertEquals(low[1], high[1], EPS);
    }

    @Test
    public void valueIsClampedToOne() {
        assertEquals(1f, hsvOf(1.6f, 0.2f, 0.2f)[2], EPS);
    }

    @Test
    public void hueDistanceWrapsAroundTheCircle() {
        assertEquals(20f, ColorMath.hueDistance(350f, 10f), EPS);
        assertEquals(20f, ColorMath.hueDistance(10f, 350f), EPS);
        assertEquals(0f, ColorMath.hueDistance(0f, 360f), EPS);
        assertEquals(180f, ColorMath.hueDistance(0f, 180f), EPS);
    }

    @Test
    public void matchesAcceptsInToleranceColour() {
        float[] yellow = hsvOf(1f, 0.9f, 0.1f);
        assertTrue(ColorMath.matches(yellow, 55f, 25f, 0.45f, 0.20f));
    }

    @Test
    public void matchesRejectsWhiteRegardlessOfHue() {
        // The saturation floor is what stops a white field wall reading as a game piece.
        float[] white = hsvOf(1f, 1f, 1f);
        assertFalse(ColorMath.matches(white, 55f, 180f, 0.45f, 0.20f));
    }

    @Test
    public void matchesRejectsShadow() {
        float[] dark = hsvOf(0.05f, 0.045f, 0.005f);
        assertFalse(ColorMath.matches(dark, 55f, 25f, 0.45f, 0.20f));
    }

    @Test
    public void matchesHandlesRedAcrossTheWrapPoint() {
        float[] red = hsvOf(1f, 0.05f, 0.02f);
        assertTrue("red near hue 0 must match a target of 355", 
                ColorMath.matches(red, 355f, 20f, 0.45f, 0.20f));
    }
}
