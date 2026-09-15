package org.firstinspires.ftc.teamcode.game;

import org.firstinspires.ftc.teamcode.subsystems.ColorSensor;
import org.firstinspires.ftc.teamcode.util.field.Alliance;

/**
 * The BIOBUZZ scoring elements: POLLEN (~2.8 in, yellow, neutral) and NECTAR (~3.6 in, red or
 * blue, alliance-specific). Carries per-type colour signatures, diameter, the camera target height,
 * and the driver-facing name. The only place the code says "pollen" or "nectar"; subsystems speak
 * of "piece" and "blob". {@code Robot} is the only production reader.
 *
 * <p>Classification is on hue, gated on saturation and value (see {@link ColorSensor#matchesHue}).
 * NECTAR carries two hue windows because it comes in two colours; telling red from blue for G408
 * (never control the opponent's NECTAR) is a separate, alliance-aware question.
 *
 * <p><b>Every number here is a placeholder</b> until measured on real pieces under match lighting
 * (HANDOFF section 9).
 */
public enum PieceType {
    /** Yellow, neutral, about 2.8 in. Forty on the field. */
    POLLEN("Pollen", 2.8, new float[] {55f}),
    /** Red or blue, alliance-specific, about 3.6 in. Eight of each colour. */
    NECTAR("Nectar", 3.6, new float[] {0f, 220f});

    /** NECTAR's two colours, one hue centre each; {@link #NECTAR} matches either (G408 needs both). */
    public static float NECTAR_RED_HUE_DEGREES = 0f;
    public static float NECTAR_BLUE_HUE_DEGREES = 220f;
    /** Half-width of each hue window, degrees. */
    public static float HUE_TOLERANCE_DEGREES = 20f;
    /** Below this saturation the hue is meaningless (white, grey, bare tile). */
    public static float MIN_SATURATION = 0.35f;
    /** Below this value the reading is shadow noise. */
    public static float MIN_VALUE = 0.15f;

    private final String displayName;
    private final double diameterInches;
    private final float[] hueDegrees;

    PieceType(String displayName, double diameterInches, float[] hueDegrees) {
        this.displayName = displayName;
        this.diameterInches = diameterInches;
        this.hueDegrees = hueDegrees;
    }

    public String displayName() {
        return displayName;
    }

    public double diameterInches() {
        return diameterInches;
    }

    /**
     * Height of the piece's centre above the floor when it rests on a tile: the plane the camera
     * ray is intersected with when estimating where a seen piece is.
     */
    public double targetHeightInches() {
        return diameterInches / 2;
    }

    /**
     * Hue window centres, degrees. A copy. NECTAR's come from the live
     * {@link #NECTAR_RED_HUE_DEGREES} / {@link #NECTAR_BLUE_HUE_DEGREES} so one edit tunes both
     * the piece detection and the alliance check.
     */
    public float[] hueDegrees() {
        if (this == NECTAR) return new float[] {NECTAR_RED_HUE_DEGREES, NECTAR_BLUE_HUE_DEGREES};
        return hueDegrees.clone();
    }

    /** True when the sensor's cached reading matches one of this type's hue windows. */
    public boolean isAtSensor(ColorSensor sensor) {
        if (sensor == null) return false;
        for (float hue : hueDegrees()) {
            if (sensor.matchesHue(hue, HUE_TOLERANCE_DEGREES, MIN_SATURATION, MIN_VALUE)) return true;
        }
        return false;
    }

    /**
     * Which alliance's NECTAR the sensor sees, or {@code null} for POLLEN, nothing, or no sensor.
     * The G408 question ("is this the opponent's?") is answered by comparing this with our own
     * alliance; {@code Robot} does that and the {@code Intake} reject acts on it.
     */
    public static Alliance nectarAllianceAt(ColorSensor sensor) {
        if (sensor == null) return null;
        if (sensor.matchesHue(NECTAR_RED_HUE_DEGREES, HUE_TOLERANCE_DEGREES, MIN_SATURATION, MIN_VALUE)) {
            return Alliance.RED;
        }
        if (sensor.matchesHue(NECTAR_BLUE_HUE_DEGREES, HUE_TOLERANCE_DEGREES, MIN_SATURATION, MIN_VALUE)) {
            return Alliance.BLUE;
        }
        return null;
    }

    /** The type the sensor sees, POLLEN checked first, or {@code null} for nothing / no sensor. */
    public static PieceType classify(ColorSensor sensor) {
        for (PieceType type : values()) {
            if (type.isAtSensor(sensor)) return type;
        }
        return null;
    }

    /** True when any piece type is at the sensor. */
    public static boolean anyAtSensor(ColorSensor sensor) {
        return classify(sensor) != null;
    }
}
