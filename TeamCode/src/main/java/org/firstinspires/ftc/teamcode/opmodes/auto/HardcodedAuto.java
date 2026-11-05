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
 *   3 s          feed ball 1, wait for the wheels to recover, ball 2, ... ball 4
 *   then         shooter and intake off, drive FORWARD (intake first) at 70% for DRIVE_MS
 *   then         stop, nothing moves again
 * </pre>
 *
 * One OpMode for both alliances, because every move is relative to the robot. It builds only the
 * shooter, the intake and the four drive motors, so it does not touch {@code Robot}, Pedro or
 * {@code pedro/Constants} and keeps working whatever is (or is not) tuned there.
 *
 * <p>Each ball is fed the same way Teleop's "shoot 4" feeds it ({@link Shoot}'s timings): a
 * {@link Shoot#FEED_PULSE_MS} pulse of the intake and the up-wheels, then a pause until both wheels
 * are back at speed or {@link Shoot#RECOVER_TIMEOUT_MS} runs out. Every step has a time limit, so a
 * slow wheel or a jam cannot stop the robot from driving afterwards.
 *
 * <p>Set on the practice field, in this order: {@link #DRIVE_MS} (and {@link #DRIVE_POWER}'s sign),
 * then the two speeds for the starting spot. The speeds only mean something once the Shooter Tuner's
 * gains are typed into {@code Shooter.java}.
 */
@Autonomous(name = "Auto HARDCODED", group = "Main", preselectTeleOp = "Teleop")
public class HardcodedAuto extends OpMode {
    /** Time from START to the first ball. The shooter spins up during it. */
    public static long FIRST_SHOT_AT_MS = 3000;
    /** Shooter speeds for the starting spot, ticks per second. Defaults are Shooter's manual speeds. */
    public static double FLYWHEEL_TICKS_PER_SEC = 1300;
    public static double ROLLER_TICKS_PER_SEC = 1300;
    /** Feed pulses. The robot pre-loads 4 (BIOBUZZ G407). */
    public static int BALLS = 4;
    /**
     * Drive after shooting. Positive drives forward, intake first; flip the sign if it goes the other
     * way. 70% on 435 RPM motors covers several feet a second, so raise {@link #DRIVE_MS} a little at
     * a time on the practice field until the robot stops where you want.
     */
    public static double DRIVE_POWER = 0.7;
    public static long DRIVE_MS = 1000;

    private enum State { SPIN_UP, FEED, RECOVER, DWELL, DRIVE, DONE }

    private List<LynxModule> hubs;
    private Shooter shooter;
    private Intake intake;
    private DcMotor frontLeft, frontRight, backLeft, backRight;

    private State state = State.DONE;
    private long startMs = 0;
    private long stateSinceMs = 0;
    private int ballsFed = 0;

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
        telemetry.addLine("HARDCODED AUTO: shoot " + BALLS + ", then drive forward");
        telemetry.addLine();
        telemetry.addLine("PLACE: shooter (rear) toward the target.");
        telemetry.addLine(String.format("After shooting the robot drives FORWARD (intake first) at %.0f%% for %.1f s:",
                DRIVE_POWER * 100, DRIVE_MS / 1000.0));
        telemetry.addLine("  make sure that path is clear.");
        telemetry.addLine();
        telemetry.addData("First ball at", "%.1f s", FIRST_SHOT_AT_MS / 1000.0);
        telemetry.addData("Speeds", "fly %.0f  roller %.0f t/s", FLYWHEEL_TICKS_PER_SEC, ROLLER_TICKS_PER_SEC);
        telemetry.addData("Shooter", shooter.getStatusLine());
        if (Shooter.kV == 0 || Shooter.ROLLER_kV == 0) {
            telemetry.addLine("!! Shooter gains not tuned (kV = 0): the wheels will barely turn.");
            telemetry.addLine("!! Run Shooter Tuner and type its lines into Shooter.java first.");
        }
    }

    @Override
    public void start() {
        startMs = System.currentTimeMillis();
        ballsFed = 0;
        shooter.setTarget(FLYWHEEL_TICKS_PER_SEC);
        shooter.setRollerTarget(ROLLER_TICKS_PER_SEC);
        shooter.arm();
        enter(State.SPIN_UP, startMs);
    }

    @Override
    public void loop() {
        clearBulkCache();
        long now = System.currentTimeMillis();

        // Safety net: nothing may move once the 30 s period is over (BIOBUZZ G403). The steps below
        // end long before that unless DRIVE_MS has been set very high.
        if (now - startMs >= MatchClock.AUTONOMOUS_MS && state != State.DONE) {
            enter(State.DONE, now);
        }

        long inState = now - stateSinceMs;
        switch (state) {
            case SPIN_UP:
                // A fixed wait: at FIRST_SHOT_AT_MS the first ball goes in even if the wheels are
                // slow, because a ball kept on the robot scores nothing.
                if (now - startMs >= FIRST_SHOT_AT_MS) enter(State.FEED, now);
                break;
            case FEED:
                if (inState >= Shoot.FEED_PULSE_MS) {
                    ballsFed++;
                    enter(ballsFed >= BALLS ? State.DWELL : State.RECOVER, now);
                }
                break;
            case RECOVER:
                // Let the dip start, then wait for both wheels to be back at speed, or give up and
                // feed the next ball anyway.
                if (inState >= Shoot.SETTLE_AFTER_PULSE_MS
                        && (shooter.atTarget()
                            || inState >= Shoot.SETTLE_AFTER_PULSE_MS + Shoot.RECOVER_TIMEOUT_MS)) {
                    enter(State.FEED, now);
                }
                break;
            case DWELL:
                // The wheels stay on a moment so the last ball leaves.
                if (inState >= Shoot.FINAL_DWELL_MS) enter(State.DRIVE, now);
                break;
            case DRIVE:
                if (inState >= DRIVE_MS) enter(State.DONE, now);
                break;
            default:
                break;
        }

        intake.update();
        shooter.update();

        telemetry.addData("Step", state);
        telemetry.addData("Time", "%.1f s", (now - startMs) / 1000.0);
        telemetry.addData("Balls fed", "%d / %d", ballsFed, BALLS);
        telemetry.addData("Shooter", shooter.getStatusLine());
        telemetry.addData("Drive", "%.2f", state == State.DRIVE ? DRIVE_POWER : 0.0);
    }

    // No stop(): the SDK zeroes every motor itself and rejects motor writes from there.

    /** Changes state and sets that state's outputs once, on entry. */
    private void enter(State next, long now) {
        state = next;
        stateSinceMs = now;
        switch (next) {
            case FEED:
                intake.in();
                shooter.feederOn();
                break;
            case SPIN_UP:
            case RECOVER:
            case DWELL:
                intake.stop();
                shooter.feederOff();
                break;
            case DRIVE:
                intake.stop();
                shooter.disarm();
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
