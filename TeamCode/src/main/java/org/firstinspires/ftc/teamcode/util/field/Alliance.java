package org.firstinspires.ftc.teamcode.util.field;

/**
 * Which alliance we are playing on this match.
 *
 * <p>Fixed by the OpMode class: {@code BlueTeleop}/{@code RedTeleop}/{@code BlueAuto}/{@code RedAuto}
 * each pass their side to the parent, so the alliance is chosen when the driver picks the OpMode and
 * is never carried between OpModes. {@link PoseStorage} carries the pose only.
 *
 * <p>Field poses are written once for {@link #BLUE} and mirrored on demand — see
 * {@link FieldConstants#forAlliance}. Keeping a second hand-written copy for red is how an
 * autonomous ends up working on one alliance and driving into a wall on the other.
 */
public enum Alliance {
    RED,
    BLUE;

    /** The other alliance. */
    public Alliance opposite() {
        return this == RED ? BLUE : RED;
    }
}
