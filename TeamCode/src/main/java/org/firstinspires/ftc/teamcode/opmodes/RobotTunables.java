package org.firstinspires.ftc.teamcode.opmodes;

import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.commands.Macros;
import org.firstinspires.ftc.teamcode.game.PieceType;
import org.firstinspires.ftc.teamcode.subsystems.ColorSensor;
import org.firstinspires.ftc.teamcode.subsystems.Drivetrain;
import org.firstinspires.ftc.teamcode.subsystems.Intake;
import org.firstinspires.ftc.teamcode.subsystems.Limelight;
import org.firstinspires.ftc.teamcode.subsystems.Shooter;
import org.firstinspires.ftc.teamcode.subsystems.Storage;
import org.firstinspires.ftc.teamcode.subsystems.Transfer;
import org.firstinspires.ftc.teamcode.subsystems.templates.VelocityMotor;
import org.firstinspires.ftc.teamcode.util.diagnostics.Tunables;
import org.firstinspires.ftc.teamcode.util.field.PoseFusion;
import org.firstinspires.ftc.teamcode.util.math.DriveScaling;

/**
 * The classes whose {@code public static} fields are the robot's tunables, for
 * {@link Tunables}. Everything below the OpMode layer; the OpModes' own statics (debug telemetry,
 * telemetry rate) are not measurements and would only be noise on the card.
 */
public final class RobotTunables {
    public static final Class<?>[] CLASSES = {
            Shooter.class, Intake.class, Storage.class, Transfer.class, Macros.class,
            Drivetrain.class, Robot.class, PieceType.class, Limelight.class, ColorSensor.class,
            VelocityMotor.class, PoseFusion.class, DriveScaling.class,
    };

    private RobotTunables() {}

    /** Records the compiled defaults, once per app process. Every OpMode calls it first in init. */
    public static void snapshot() {
        Tunables.snapshot(CLASSES);
    }
}
