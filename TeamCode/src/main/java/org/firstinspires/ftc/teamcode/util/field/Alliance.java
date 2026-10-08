package org.firstinspires.ftc.teamcode.util.field;

/**
 * Which alliance we are playing on this match.
 *
 * <p>Autonomous gets it from the OpMode class ({@code BlueAuto}/{@code RedAuto}); Teleop picks it on
 * the dpad during init, starting from the side the last autonomous ran ({@link PoseStorage}).
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
