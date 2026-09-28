package org.firstinspires.ftc.teamcode.util.hardware;

/**
 * Every name this code expects to find in the Robot Controller configuration, and nothing else.
 *
 * <h2>Seven motors, one camera, one odometry computer</h2>
 * Read off the robot CAD (2026-09-27). BIOBUZZ R503 allows at most 8 DC motors, so there is exactly
 * one spare port.
 * <ul>
 *   <li>4 x drive, goBILDA 5203-2402-0014 (13.7:1, 435 RPM)</li>
 *   <li>1 x intake, goBILDA 5203-2402-0051 (50.9:1, 117 RPM) — drives the roller
 *       <b>and</b> the tunnel through the sprocket chain, so there is no separate transfer motor</li>
 *   <li>2 x shooter, a counter-rotating flywheel pair on GT2/HTD belts</li>
 * </ul>
 *
 * <h2>A name here that is not in the configuration is a crash, on purpose</h2>
 * Every lookup is a bare {@code hardwareMap.get}. A typo throws at OpMode init and names the device.
 * That is louder and faster to diagnose in the pit than a robot that silently runs on three wheels
 * with one mechanism quietly disabled, which is what the previous fail-soft wrapper produced.
 *
 * <h2>There are no piece sensors</h2>
 * The robot senses its own position (Pinpoint) and the HIVE's AprilTags (Limelight). Nothing counts
 * game pieces, so nothing here reserves a name for a sensor that would.
 */
public final class HardwareNames {
    private HardwareNames() {}

    // ---- Drivetrain, Control Hub motor ports 0-3 ----
    //
    // Consumed by MecanumConfig in pedro/Constants.java. Point that file at these rather than
    // retyping them, or the drivetrain becomes the one mechanism whose config names live elsewhere.

    public static final String FRONT_LEFT_MOTOR = "fl";
    public static final String FRONT_RIGHT_MOTOR = "fr";
    public static final String BACK_LEFT_MOTOR = "bl";
    public static final String BACK_RIGHT_MOTOR = "br";

    /** goBILDA Pinpoint odometry computer (I2C). Pedro's localizer; see pedro/Constants. */
    public static final String PINPOINT = "pinpoint";

    // ---- Intake: front roller and the tunnel behind it, on one motor ----

    public static final String INTAKE_MOTOR = "i";

    // ---- Shooter: two motors, opposed flywheels ----

    /** The flywheel whose encoder is the speed signal (see {@code Shooter.getVelocity()}). */
    public static final String SHOOTER_MOTOR = "sl";
    /** The opposing flywheel. Runs reversed; same surface speed. */
    public static final String SHOOTER_MOTOR_2 = "sr";

    // ---- Vision ----

    /** Limelight 3A, front-mounted, configured as an Ethernet device. AprilTag aiming only. */
    public static final String LIMELIGHT = "limelight";
}
