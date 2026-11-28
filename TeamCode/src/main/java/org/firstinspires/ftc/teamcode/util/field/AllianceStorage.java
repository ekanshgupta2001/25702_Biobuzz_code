package org.firstinspires.ftc.teamcode.util.field;

/**
 * Remembers the alliance from one OpMode to the next, so it is picked once per match.
 *
 * <p>The autonomous saves the side picked on its init screen; Teleop's init starts on it. The driver
 * still sees the alliance on Teleop's init card and can change it with the dpad.
 *
 * <p>Static, so it survives between OpModes but not a Robot Controller restart or a power cycle:
 * after one of those it is BLUE again until someone picks.
 */
public final class AllianceStorage {
    private static volatile Alliance alliance = null;

    private AllianceStorage() {}

    public static void save(Alliance side) {
        alliance = side;
    }

    /** The side picked last, or {@code fallback} if none has been picked since the app started. */
    public static Alliance get(Alliance fallback) {
        return alliance == null ? fallback : alliance;
    }
}
