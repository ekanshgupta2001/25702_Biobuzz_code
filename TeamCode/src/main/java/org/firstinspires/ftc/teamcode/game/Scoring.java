package org.firstinspires.ftc.teamcode.game;

/**
 * BIOBUZZ point values and ranking thresholds (Competition Manual V1, tables 10-2 to 10-4), for
 * strategy arithmetic in autonomous selection and telemetry. Championship RP thresholds are not
 * yet published.
 */
public final class Scoring {
    private Scoring() {}

    /** AUTO only: no longer touching the perimeter wall. */
    public static final int LEAVE = 3;
    /** AUTO and TELEOP: at least partially in the alliance's LOADING ZONE. */
    public static final int PARK = 5;
    public static final int HIVE_TIP = 20;
    /** TELEOP end: each POLLEN or NECTAR remaining in the up-facing CELL. */
    public static final int ELEMENT_IN_UP_CELL = 2;
    /** Each element in a FLOWER the alliance owns (its NECTAR is top-most). */
    public static final int ELEMENT_IN_OWNED_FLOWER = 2;
    /** The alliance whose NECTAR is bottom-most in a FLOWER. */
    public static final int FLOWER_BOTTOM_NECTAR_BONUS = 5;
    public static final int ELEMENT_IN_GARDEN = 1;

    public static final int MINOR_FOUL = 5;
    public static final int MAJOR_FOUL = 20;

    public static final int WIN_RP = 3;
    public static final int TIE_RP = 1;
    /** SWARM ranking point: LEAVE plus PARK points at or above this (ordinary events). */
    public static final int SWARM_LEAVE_PARK_POINTS = 16;
    public static final int POLLINATOR_1_TIPS = 4;
    public static final int POLLINATOR_2_TIPS = 7;

    /** LEAVE + first TIP + PARK: what a three-shot autonomous is worth. */
    public static int threeShotAutoPoints() {
        return LEAVE + HIVE_TIP + PARK;
    }
}
