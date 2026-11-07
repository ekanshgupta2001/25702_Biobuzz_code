package org.firstinspires.ftc.teamcode.opmodes.auto;

import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.teamcode.commands.Shoot;
import org.firstinspires.ftc.teamcode.subsystems.Intake;
import org.firstinspires.ftc.teamcode.subsystems.Shooter;
import org.firstinspires.ftc.teamcode.util.hardware.HardwareNames;
import org.firstinspires.ftc.teamcode.util.time.MatchClock;

import java.util.List;

/**
 * The autonomous for when Pedro is not tuned: timers only, no paths and no odometry.
 *
 * <pre>
 *   START        shooter spins up
 *   at speed     (or Shoot.SPIN_UP_TIMEOUT_MS, 3 s) intake + up-wheels run nonstop for
 *                Shoot.AUTO_FEED_MS, so the four balls go through one after another
 *   then         feed off, wheels on a moment longer (Shoot.FINAL_DWELL_MS) so the last ball leaves
 *   then         shooter and intake off, drive FORWARD (intake first) at 70% for DRIVE_MS
 *   then         stop for DRIVE_PAUSE_MS, then drive BACK at 70% for DRIVE_BACK_MS
 *   then         stop, nothing moves again
 * </pre>
 *
 * One OpMode for both alliances, because every move is relative to the robot. It builds only the
 * shooter, the intake and the four drive motors, so it does not touch {@code Robot}, Pedro or
 * {@code pedro/Constants} and keeps working whatever is (or is not) tuned there.
 *
 * <p>It shoots the same way Teleop's shot macro does ({@link Shoot}'s timings): wait once for both
 * wheels to reach speed, then feed nonstop. The wheels recover between balls by themselves
 * ({@code Shooter.update()}). Every step has a time limit, so a slow wheel or a jam cannot stop the
 * robot from driving afterwards.
 *
 * <p>The wheels run at the Shooter's fixed speeds, {@code Shooter.TARGET_TICKS_PER_SEC} and
 * {@code Shooter.ROLLER_TARGET_TICKS_PER_SEC}, the same as Teleop.
 *
 * <p>"Auto HARDCODED NO SHOOT" ({@link HardcodedNoShootAuto}) is this auto without the shooting: it
 * starts the drive at START. Both use the same DRIVE_ constants, so tuning one tunes both.
 *
 * <p>Set on the practice field: {@link #DRIVE_MS}, {@link #DRIVE_BACK_MS} (and
 * {@link #DRIVE_POWER}'s sign).
 */
@Autonomous(name = "Auto HARDCODED", group = "Main", preselectTeleOp = "Teleop")
public class HardcodedAuto extends OpMode {
    /**
     * Drive power after shooting, for both moves. Positive drives forward, intake first; flip the
     * sign if it goes the other way (the move back flips with it). 70% on 435 RPM motors covers
     * several feet a second, so change the times below a little at a time on the practice field
     * until the robot stops where you want.
     */
    public static double DRIVE_POWER = 0.7;
    /** How long it drives forward. */
    public static long DRIVE_MS = 1000;
    /** Stopped between the two moves, so the move back starts from standstill instead of skidding. */
    public static long DRIVE_PAUSE_MS = 300;
    /**
     * How long it drives back. Shorter than {@link #DRIVE_MS} so it stops short of where it started,
     * where a wall or the goal may be.
     */
    public static long DRIVE_BACK_MS = 800;

    private enum State { SPIN_UP, FEED, DWELL, FORWARD, PAUSE, BACK, DONE }

    /** False in Auto HARDCODED NO SHOOT: the robot only drives. */
    private final boolean shoot;

    private List<LynxModule> hubs;
    private Shooter shooter;
    private Intake intake;
    private DcMotor frontLeft, frontRight, backLeft, backRight;

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
        telemetry.addLine(shoot
                ? "HARDCODED AUTO: shoot, then drive forward and back"
                : "HARDCODED AUTO, NO SHOOTING: at START it drives forward and back");
        telemetry.addLine();
        if (shoot) telemetry.addLine("PLACE: shooter (rear) toward the target.");
        telemetry.addLine(String.format("Drives FORWARD (intake first) %.1f s, stops, then BACK %.1f s, at %.0f%%:",
                DRIVE_MS / 1000.0, DRIVE_BACK_MS / 1000.0, DRIVE_POWER * 100));
        telemetry.addLine("  make sure that path is clear.");
        if (shoot) {
            telemetry.addLine();
            telemetry.addData("Feed", "%.1f s nonstop once at speed (%.1f s max wait)",
                    Shoot.AUTO_FEED_MS / 1000.0, Shoot.SPIN_UP_TIMEOUT_MS / 1000.0);
            telemetry.addData("Speeds", "fly %.0f  roller %.0f t/s",
                    Shooter.TARGET_TICKS_PER_SEC, Shooter.ROLLER_TARGET_TICKS_PER_SEC);
            telemetry.addData("Shooter", shooter.getStatusLine());
            if (Shooter.kV == 0 || Shooter.ROLLER_kV == 0) {
                telemetry.addLine("!! Shooter gains not tuned (kV = 0): the wheels will barely turn.");
                telemetry.addLine("!! Run Shooter Tuner and type its lines into Shooter.java first.");
            }
        }
    }

    @Override
    public void start() {
        startMs = System.currentTimeMillis();
        if (shoot) {
            shooter.arm();   // holds Shooter.TARGET_TICKS_PER_SEC / ROLLER_TARGET_TICKS_PER_SEC
            enter(State.SPIN_UP, startMs);
        } else {
            enter(State.FORWARD, startMs);
        }
    }

    @Override
    public void loop() {
        clearBulkCache();
        long now = System.currentTimeMillis();

        // Safety net: nothing may move once the 30 s period is over (BIOBUZZ G403). The steps below
        // end long before that unless the drive times have been set very high.
        if (now - startMs >= MatchClock.AUTONOMOUS_MS && state != State.DONE) {
            enter(State.DONE, now);
        }

        long inState = now - stateSinceMs;
        switch (state) {
            case SPIN_UP:
                // Wait once for both wheels to be at speed. If they are slow, feed anyway at the
                // time limit, because a ball kept on the robot scores nothing.
                if (shooter.atTarget() || inState >= Shoot.SPIN_UP_TIMEOUT_MS) enter(State.FEED, now);
                break;
            case FEED:
                // Nonstop: the wheels recover between balls by themselves (Shooter.update()).
                if (inState >= Shoot.AUTO_FEED_MS) enter(State.DWELL, now);
                break;
            case DWELL:
                // The wheels stay on a moment so the last ball leaves.
                if (inState >= Shoot.FINAL_DWELL_MS) enter(State.FORWARD, now);
                break;
            case FORWARD:
                if (inState >= DRIVE_MS) enter(State.PAUSE, now);
                break;
            case PAUSE:
                if (inState >= DRIVE_PAUSE_MS) enter(State.BACK, now);
                break;
            case BACK:
                if (inState >= DRIVE_BACK_MS) enter(State.DONE, now);
                break;
            default:
                break;
        }

        intake.update();
        shooter.update();

        telemetry.addData("Step", state);
        telemetry.addData("Time", "%.1f s", (now - startMs) / 1000.0);
        if (shoot) telemetry.addData("Shooter", shooter.getStatusLine());
        telemetry.addData("Drive", "%.2f", drivePower);
    }

    // No stop(): the SDK zeroes every motor itself and rejects motor writes from there.

    /** Changes state and sets that state's outputs once, on entry. */
    private void enter(State next, long now) {
        state = next;
        stateSinceMs = now;
        switch (next) {
            case FEED:
                intake.feed();
                shooter.feederOn();
                break;
            case SPIN_UP:
            case DWELL:
                intake.stop();
                shooter.feederOff();
                break;
            case FORWARD:
                intake.stop();
                shooter.disarm();
                setDrive(DRIVE_POWER);
                break;
            case PAUSE:
                setDrive(0);
                break;
            case BACK:
                setDrive(-DRIVE_POWER);
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

    private void clearBulkCache() {
        for (LynxModule hub : hubs) {
            hub.clearBulkCache();
        }
    }
}
