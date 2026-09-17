package org.firstinspires.ftc.teamcode.util.hardware;

/**
 * Every name this code expects to find in the Robot Controller configuration.
 *
 * <h2>Why these are not just literals in each constructor</h2>
 * Literals spread across the subsystems and Pedro's constants mean a dozen files to open to answer
 * "what does the config need to be called?", and a dozen to edit when someone renames a device in
 * the configuration app at a competition. Renaming a device is a one-line change here. The
 * subsystem constructors that take an explicit name still exist, so a second robot with a different
 * configuration is still possible without touching this file.
 *
 * <h2>Keep these matching the physical robot</h2>
 * A name here that does not exist in the active configuration does not crash anything: every lookup
 * goes through {@link Hardware}, which records it and lets the subsystem no-op. Run the
 * {@code SelfTest} OpMode and it will print exactly which of these could not be found.
 *
 * <h2>Layout</h2>
 * Grouped by the mechanism they belong to, following the game-piece path in
 * {@code docs/02-robot-physical-architecture.md}: drivetrain, intake, storage, transfer,
 * shooter, vision, then the five reserved sensor points. This is the V1 robot: a fixed shooter
 * (no turret), no Flower mechanism, extension or diverter (V1 spec section 13).
 *
 * <h2>Motor and servo budget</h2>
 * BIOBUZZ rule R503 allows <b>at most 8 DC motors and 8 servos</b> across all configurations
 * (Team Update 00 reduced servos from 12). The V1 mechanism list below names 8 to 10 motor slots
 * (4 drive, intake, storage x1-2, transfer, shooter x1-2): a single flywheel and a single storage
 * motor fit exactly, otherwise one mechanism must share a motor (storage and transfer geared
 * together); see {@code docs/04-biobuzz-season-analysis.md} section 9. The optional second-motor
 * names stay so a shared or unshared build is a one-line difference here, not a rename elsewhere.
 */
public final class HardwareNames {
    private HardwareNames() {}

    // ---- Drivetrain (Control Hub motor ports 0-3) ----
    //
    // Consumed by MecanumConfig inside pedro/Constants.java. Point that file at these rather than
    // retyping the names, otherwise the drivetrain becomes the one subsystem whose config names live
    // somewhere else.

    public static final String FRONT_LEFT_MOTOR = "front_left_drive";
    public static final String FRONT_RIGHT_MOTOR = "front_right_drive";
    public static final String BACK_LEFT_MOTOR = "back_left_drive";
    public static final String BACK_RIGHT_MOTOR = "back_right_drive";

    /** goBILDA Pinpoint odometry computer (I2C), the Pedro localizer. */
    public static final String PINPOINT = "pinpoint";

    // ---- Front intake (16 mm compliant roller above the ramp) ----

    /** Intake roller motor, velocity controlled. */
    public static final String INTAKE_MOTOR = "intake";

    // ---- Horizontal four-piece storage (Gecko side wheels) ----

    /** Storage transport motor driving the side wheels. */
    public static final String STORAGE_MOTOR = "storage";
    /** Second transport motor, if the two sides are driven separately. Absent otherwise. */
    public static final String STORAGE_MOTOR_2 = "storage_2";

    // ---- Rear 90-degree vertical transfer (opposing Gecko wheels) ----

    /** Vertical transfer motor, velocity controlled. */
    public static final String TRANSFER_MOTOR = "transfer";

    // ---- Shooter (compliant flywheel) ----

    /** Primary flywheel motor, velocity controlled. */
    public static final String SHOOTER_MOTOR = "shooter";
    /** Second flywheel motor, if fitted. Absent otherwise. */
    public static final String SHOOTER_MOTOR_2 = "shooter_2";

    // ---- Vision ----

    /** Limelight 3A, configured as an Ethernet device. */
    public static final String LIMELIGHT = "limelight";

    // ---- Reserved sensor points along the game-piece path ----
    //
    // The physical spec reserves room for a sensor at each of these. The type (REV Color Sensor V3,
    // 2 m distance sensor, beam break on a digital channel) is decided per point when it is fitted;
    // the name stays the same either way so the wrapper that resolves it is the only thing to change.

    /** A piece has entered the storage channel (counts pieces, identifies Pollen vs Nectar). */
    public static final String SENSOR_STORAGE_ENTRANCE = "sensor_storage_entrance";
    /** The fourth storage slot is occupied. */
    public static final String SENSOR_STORAGE_FULL = "sensor_storage_full";
    /** A piece is in the vertical transfer. */
    public static final String SENSOR_TRANSFER = "sensor_transfer";
    /** A piece is staged at the shooter feed. */
    public static final String SENSOR_SHOOTER_FEED = "sensor_shooter_feed";
}
