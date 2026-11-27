package org.firstinspires.ftc.teamcode.opmodes.auto;

import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.VoltageSensor;

import org.firstinspires.ftc.teamcode.commands.Shoot;
import org.firstinspires.ftc.teamcode.subsystems.Intake;
import org.firstinspires.ftc.teamcode.subsystems.Shooter;
import org.firstinspires.ftc.teamcode.util.field.Alliance;
import org.firstinspires.ftc.teamcode.util.field.AllianceStorage;
import org.firstinspires.ftc.teamcode.util.hardware.HardwareNames;

import java.util.List;

/**
 * The autonomous for when Pedro is not tuned: timers only, no paths and no odometry.
 *
 * <pre>
 *   START        wheels spin up (the robot's FRONT faces the target)
 *   3 s          intake + feeder run nonstop for FEED_MS: all four balls go in one after another
 *   then         shooter and intake off, drive AWAY (backwards) at 70% for DRIVE_MS
 *   then         stop for DRIVE_PAUSE_MS, then RETURN (forwards) at 70% for DRIVE_BACK_MS
 *   then         stop, nothing moves again
 * </pre>
 *
 * One OpMode for both alliances, because every move is relative to the robot. It builds only the
 * shooter, the intake and the four drive motors, so it does not touch {@code Robot}, Pedro, the
 * Limelight or {@code pedro/Constants} and keeps working whatever is (or is not) tuned there.
 *
 * <p><b>Pick the alliance on the init screen</b> (gamepad 1 dpad left = RED, right = BLUE). This auto
 * does not use it, but Teleop starts on the side picked here, and Teleop's camera only reads that
 * side's tags.
 *
 * <p>The balls are fed the way Teleop feeds them with RT held: the intake and the feeder run together
 * for {@link #FEED_MS} with no pause between balls (the wheels recover by themselves). Every step is
 * a timer, so a slow wheel or a jam cannot stop the robot from driving afterwards.
 *
 * <p>The wheels run at the fixed speeds, {@code Shooter.TARGET_TICKS_PER_SEC} and
 * {@code Shooter.ROLLER_TARGET_TICKS_PER_SEC}: there is no camera and no position here. The card shows
 * the speeds this run will use.
 *
 * <p>"Auto HARDCODED NO SHOOT" ({@link HardcodedNoShootAuto}) is this auto without the shooting: it
 * starts the drive at START. Both use the same DRIVE_ constants, so tuning one tunes both.
 *
 * <p>Set on the practice field: {@link #FEED_MS}, {@link #DRIVE_MS}, {@link #DRIVE_BACK_MS} (and
 * {@link #DRIVE_POWER}'s sign).
 */
@Autonomous(name = "Auto HARDCODED", group = "Main", preselectTeleOp = "Teleop")
public class HardcodedAuto extends OpMode {
    /** Time from START to the first ball. The shooter spins up during it. */
    public static long FIRST_SHOT_AT_MS = 3000;
    /**
     * How long the intake and the feeder run, nonstop. Long enough for all 4 pre-loaded balls to
     * leave. A first guess: time it on the practice field.
     */
    public static long FEED_MS = 2000;
    /**
     * Drive power after shooting, for both moves. AWAY drives at -DRIVE_POWER (backwards, away from
     * the target the front faces), RETURN at +DRIVE_POWER. Flipping its sign flips both moves. 70% on
     * 435 RPM motors covers several feet a second, so change the times below a little at a time on
     * the practice field until the robot stops where you want.
     */
    public static double DRIVE_POWER = 0.7;
    /** How long it drives away (backwards). */
    public static long DRIVE_MS = 1000;
    /** Stopped between the two moves, so the return starts from standstill instead of skidding. */
    public static long DRIVE_PAUSE_MS = 300;
    /**
     * How long it drives back toward the start. Shorter than {@link #DRIVE_MS} so it stops short of
     * where it started, where a wall or the goal may be.
     */
    public static long DRIVE_BACK_MS = 800;

    /** The autonomous period. Nothing may move after it (BIOBUZZ G403). */
    private static final long AUTO_PERIOD_MS = 30000;

    private enum State { SPIN_UP, FEED, DWELL, AWAY, PAUSE, RETURN, DONE }

    /** False in Auto HARDCODED NO SHOOT: the robot only drives. */
    private final boolean shoot;

    private List<LynxModule> hubs;
    private Shooter shooter;
    private Intake intake;
    private DcMotor frontLeft, frontRight, backLeft, backRight;
    /** For Shooter.TUNED_AT_VOLTS; read every {@link #VOLTAGE_SAMPLE_MS} (each read is its own bus transaction). */
    private VoltageSensor battery;
    private static final long VOLTAGE_SAMPLE_MS = 250;
    private long lastVoltageMs = 0;

    /** Picked on the init screen and handed to Teleop. This auto itself does not use it. */
    private Alliance alliance;

    private State state = State.DONE;
    private long startMs = 0;
    private long stateSinceMs = 0;
    /** The power the drive motors were last given, for the card. */
    private double drivePower = 0;

    public HardcodedAuto() {
        this(true);
    }

    /** For {@link HardcodedNoShootAuto}: {@code false} skips the shooting. */
    protected HardcodedAuto(boolean shoot) {
        this.shoot = shoot;
    }

    @Override
    public void init() {
        // MANUAL bulk caching: every encoder read in a loop comes from one bus transaction, cleared
        // at the top of each loop (the same as Robot.readSensors()).
        hubs = hardwareMap.getAll(LynxModule.class);
        for (LynxModule hub : hubs) {
            hub.setBulkCachingMode(LynxModule.BulkCachingMode.MANUAL);
        }

        shooter = new Shooter(hardwareMap);
        intake = new Intake(hardwareMap);
        // Starts on the side picked last time (BLUE after a power-up); the dpad changes it in init_loop.
        alliance = AllianceStorage.get(Alliance.BLUE);
        AllianceStorage.save(alliance);
        battery = hardwareMap.voltageSensor.iterator().next();

        // Same names and directions as pedro/Constants.drivetrainConfig. If the robot strafes or
        // spins instead of driving straight, one of these directions is wrong.
        frontLeft = driveMotor(HardwareNames.FRONT_LEFT_MOTOR, DcMotorSimple.Direction.REVERSE);
        backLeft = driveMotor(HardwareNames.BACK_LEFT_MOTOR, DcMotorSimple.Direction.REVERSE);
        frontRight = driveMotor(HardwareNames.FRONT_RIGHT_MOTOR, DcMotorSimple.Direction.FORWARD);
        backRight = driveMotor(HardwareNames.BACK_RIGHT_MOTOR, DcMotorSimple.Direction.FORWARD);

        telemetry.setMsTransmissionInterval(100);
    }

    private DcMotor driveMotor(String name, DcMotorSimple.Direction direction) {
        DcMotor motor = hardwareMap.get(DcMotor.class, name);
        motor.setDirection(direction);
        // BRAKE, so the robot stops when the drive time ends instead of coasting on.
        motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motor.setPower(0);
        return motor;
    }

    @Override
    public void init_loop() {
        clearBulkCache();

        if (gamepad1.dpadLeftWasPressed()) alliance = Alliance.RED;
        if (gamepad1.dpadRightWasPressed()) alliance = Alliance.BLUE;
        AllianceStorage.save(alliance);   // Teleop's init starts on this side

        telemetry.addLine("=========================");
        telemetry.addLine("   ALLIANCE:  " + alliance);
        telemetry.addLine("=========================");
        telemetry.addLine("dpad left = RED, right = BLUE  (Teleop starts on it)");
        telemetry.addLine();
        telemetry.addLine(shoot
                ? "HARDCODED AUTO: shoot, then drive away and part way back"
                : "HARDCODED AUTO, NO SHOOTING: at START it drives away and part way back");
        telemetry.addLine();
        if (shoot) telemetry.addLine("PLACE: FRONT (shooter + camera) toward the target");
        telemetry.addLine(String.format("Drives AWAY %.1f s at %+.0f%%, stops, then RETURNS %.1f s at %+.0f%%",
                DRIVE_MS / 1000.0, -DRIVE_POWER * 100, DRIVE_BACK_MS / 1000.0, DRIVE_POWER * 100));
        telemetry.addLine("  (- = backwards): make sure that path is clear.");
        if (shoot) {
            telemetry.addLine();
            telemetry.addData("Feeds", "from %.1f s, for %.1f s", FIRST_SHOT_AT_MS / 1000.0, FEED_MS / 1000.0);
            telemetry.addData("Speeds", "fly %.0f  roller %.0f t/s (fixed)",
                    shooter.getTarget(), shooter.getRollerTarget());
            telemetry.addData("Shooter", shooter.getStatusLine());
            if (Shooter.kV == 0 || Shooter.ROLLER_kV == 0) {
                telemetry.addLine("!! Shooter gains not tuned (kV = 0): the wheels will barely turn.");
                telemetry.addLine("!! Run Shooter Tuner and type its lines into Shooter.java first.");
            }
        }
    }

    @Override
    public void start() {
        startMs = nowMs();
        if (shoot) {
            shooter.arm();   // holds the fixed speeds, read when the Shooter was built
            enter(State.SPIN_UP, startMs);
        } else {
            enter(State.AWAY, startMs);
        }
    }

    @Override
    public void loop() {
        clearBulkCache();
        long now = nowMs();
        if (shoot && now - lastVoltageMs >= VOLTAGE_SAMPLE_MS) {
            lastVoltageMs = now;
            shooter.setBatteryVolts(battery.getVoltage());   // only used when Shooter.TUNED_AT_VOLTS is set
        }

        // Safety net: nothing may move once the 30 s period is over (BIOBUZZ G403). The steps below
        // end long before that unless the drive times have been set very high.
        if (now - startMs >= AUTO_PERIOD_MS && state != State.DONE) {
            enter(State.DONE, now);
        }

        long inState = now - stateSinceMs;
        switch (state) {
            case SPIN_UP:
                // A fixed wait: at FIRST_SHOT_AT_MS the balls go in even if the wheels are slow,
                // because a ball kept on the robot scores nothing.
                if (now - startMs >= FIRST_SHOT_AT_MS) enter(State.FEED, now);
                break;
            case FEED:
                if (inState >= FEED_MS) enter(State.DWELL, now);
                break;
            case DWELL:
                // The wheels stay on a moment so the last ball leaves.
                if (inState >= Shoot.FINAL_DWELL_MS) enter(State.AWAY, now);
                break;
            case AWAY:
                if (inState >= DRIVE_MS) enter(State.PAUSE, now);
                break;
            case PAUSE:
                if (inState >= DRIVE_PAUSE_MS) enter(State.RETURN, now);
                break;
            case RETURN:
                if (inState >= DRIVE_BACK_MS) enter(State.DONE, now);
                break;
            default:
                break;
        }

        intake.update();
        shooter.update();

        telemetry.addData("Step", state);
        telemetry.addData("Time", "%.1f s", (now - startMs) / 1000.0);
        if (shoot) {
            telemetry.addData("Shooter", shooter.getStatusLine());
        }
        telemetry.addData("Drive", "%.2f", drivePower);
    }

    // No stop(): the SDK zeroes every motor itself and rejects motor writes from there.

    /** Changes state and sets that state's outputs once, on entry. */
    private void enter(State next, long now) {
        state = next;
        stateSinceMs = now;
        switch (next) {
            case FEED:
                intake.feed();   // the intake and the feeder carry the ball into the wheels
                shooter.feederOn();
                break;
            case SPIN_UP:
            case DWELL:
                intake.stop();
                shooter.feederOff();
                break;
            case AWAY:
                intake.stop();
                shooter.disarm();   // the wheels and the feeder
                setDrive(-DRIVE_POWER);   // backwards: the front faced the target
                break;
            case PAUSE:
                setDrive(0);
                break;
            case RETURN:
                setDrive(DRIVE_POWER);
                break;
            case DONE:
            default:
                intake.stop();
                shooter.disarm();
                setDrive(0);
                break;
        }
    }

    private void setDrive(double power) {
        drivePower = power;
        frontLeft.setPower(power);
        frontRight.setPower(power);
        backLeft.setPower(power);
        backRight.setPower(power);
    }

    /**
     * Milliseconds from a clock that only ever counts up. Not System.currentTimeMillis(): the Driver
     * Station can set the Control Hub's wall clock, and a jump in the middle of a timed drive would
     * make that drive longer.
     */
    private static long nowMs() {
        return System.nanoTime() / 1_000_000;
    }

    private void clearBulkCache() {
        for (LynxModule hub : hubs) {
            hub.clearBulkCache();
        }
    }
}