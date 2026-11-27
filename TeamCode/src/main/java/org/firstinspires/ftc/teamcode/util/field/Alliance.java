package org.firstinspires.ftc.teamcode.util.field;

/**
 * Which alliance we are playing on this match.
 *
 * <p>Picked on the dpad on the init screen of the autonomous and of Teleop; each starts on the side
 * picked last ({@link AllianceStorage}).
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
