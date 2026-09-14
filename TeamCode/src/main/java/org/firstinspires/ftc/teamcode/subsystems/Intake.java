package org.firstinspires.ftc.teamcode.subsystems;

import com.pedropathing.ivy.Command;
import com.pedropathing.ivy.behaviors.BlockedBehavior;
import com.pedropathing.ivy.behaviors.EndCondition;
import com.pedropathing.ivy.behaviors.InterruptedBehavior;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.subsystems.templates.VelocityMotor;
import org.firstinspires.ftc.teamcode.util.control.JamDetector;
import org.firstinspires.ftc.teamcode.util.hardware.Hardware;
import org.firstinspires.ftc.teamcode.util.hardware.HardwareNames;
import org.firstinspires.ftc.teamcode.util.time.Clock;

import java.util.function.BooleanSupplier;

/**
 * The front intake roller (16 mm compliant wheels above the ramp), driven by velocity control
 * with stall-triggered anti-jam.
 *
 * <p><b>The core pattern:</b> commands and buttons never touch the motor. They set a
 * {@link Mode} and a target velocity; {@link #update()} is the single place that writes to
 * hardware, once per loop, and the anti-jam logic can override the request on its way out.
 *
 * <h2>V1 responsibilities</h2>
 * Pull pieces off the floor and onto the ramp; that is all. The storage counts pieces and
 * decides when the robot is full ({@link #setFullSupplier}); while it is, an intaking command
 * keeps its mode and resource but the roller sits still, so the fifth piece that would break
 * BIOBUZZ G407 is never pulled in. Whether a piece was captured comes from the intake-entrance
 * or storage-entrance sensor through {@link #setCapturedSupplier}, wired by {@code Robot}.
 */
public class Intake {
    /**
     * goBILDA 5203 series, 435 RPM (13.7:1) = 384.5 ticks/rev, free speed ~2787 ticks/s.
     * Measure on the real motor with SelfTest before trusting any of the velocities below.
     */
    public static double MOTOR_FREE_SPEED_TICKS_PER_SEC = 2787;

    /** ~90% of free speed, leaving the velocity loop headroom to actually close. */
    public static double INTAKE_TICKS_PER_SEC = 2500;
    public static double OUTTAKE_TICKS_PER_SEC = -1400;
    public static double EJECT_TICKS_PER_SEC = -2500;

    public static double STALL_CURRENT_AMPS = 5.0;
    public static long STALL_TIMEOUT_MS = 200;
    public static double UNJAM_TICKS_PER_SEC = -2500;
    public static long UNJAM_DURATION_MS = 150;
    public static boolean ANTI_JAM_ENABLED = true;
    /** Consecutive unjam attempts before giving up, so a hard jam cannot cook the motor all match. */
    public static int MAX_UNJAM_ATTEMPTS = 3;

    public static int DEFAULT_IDLE_PRIORITY = -1;

    /** What the intake is being asked to do. Drives anti-jam eligibility; see {@link #update()}. */
    public enum Mode { IDLE, INTAKING, OUTTAKING, EJECTING }

    private final VelocityMotor motor;
    private final Clock clock;
    private double targetVelocity = 0;
    private Mode mode = Mode.IDLE;
    private boolean hasPiece = false;
    private BooleanSupplier capturedSupplier = () -> false;
    private BooleanSupplier fullSupplier = () -> false;

    /** The stall/un-jam state machine. Lives in {@code util/} so it can be unit tested. */
    private final JamDetector jamDetector = new JamDetector();

    public Intake(HardwareMap hardwareMap) {
        this(hardwareMap, HardwareNames.INTAKE_MOTOR, Clock.system());
    }

    public Intake(HardwareMap hardwareMap, String name, Clock clock) {
        this(Hardware.get(hardwareMap, DcMotorEx.class, name), clock);
    }

    /** Builds on an already-resolved motor, or {@code null} for "not fitted". Tests inject fakes. */
    public Intake(DcMotorEx motor, Clock clock) {
        this.motor = new VelocityMotor(motor, DcMotorSimple.Direction.FORWARD,
                DcMotor.ZeroPowerBehavior.FLOAT);
        this.clock = clock;
    }

    /** False when the motor is missing from the robot configuration. All calls then no-op. */
    public boolean isAvailable() {
        return motor.isAvailable();
    }

    /** "A piece has just come in": the intake-entrance or storage-entrance sensor. */
    public void setCapturedSupplier(BooleanSupplier supplier) {
        this.capturedSupplier = supplier == null ? () -> false : supplier;
    }

    /** "The storage is full": while true, intaking commands hold the roller still. */
    public void setFullSupplier(BooleanSupplier supplier) {
        this.fullSupplier = supplier == null ? () -> false : supplier;
    }

    /** Sets the raw velocity request. Prefer the named modes so anti-jam stays correct. */
    public void setVelocity(double ticksPerSec) {
        setMode(ticksPerSec > 0 ? Mode.INTAKING : ticksPerSec < 0 ? Mode.EJECTING : Mode.IDLE,
                ticksPerSec);
    }

    private void setMode(Mode newMode, double ticksPerSec) {
        // Any deliberate mode change abandons an in-progress unjam. Re-entering INTAKING keeps the
        // attempt count, so mashing the button cannot bypass MAX_UNJAM_ATTEMPTS against a hard jam.
        if (newMode != mode) {
            if (newMode == Mode.INTAKING) jamDetector.resetTiming();
            else jamDetector.reset();
        }
        mode = newMode;
        targetVelocity = ticksPerSec;
    }

    public void intake() {
        setMode(Mode.INTAKING, INTAKE_TICKS_PER_SEC);
    }

    public void outtake() {
        setMode(Mode.OUTTAKING, OUTTAKE_TICKS_PER_SEC);
        markEmpty();
    }

    public void eject() {
        setMode(Mode.EJECTING, EJECT_TICKS_PER_SEC);
        markEmpty();
    }

    public void stop() {
        setMode(Mode.IDLE, 0);
    }

    public Mode getMode() {
        return mode;
    }

    /** True once a piece has been seen entering while intaking, until cleared. */
    public boolean hasPiece() {
        return hasPiece;
    }

    public void markCaptured() {
        hasPiece = true;
    }

    public void markEmpty() {
        hasPiece = false;
    }

    public boolean isBlockedByFullStorage() {
        return mode == Mode.INTAKING && fullSupplier.getAsBoolean();
    }

    public double getCurrentAmps() {
        return motor.getCurrentAmps();
    }

    public double getVelocityTicksPerSec() {
        return motor.getVelocity();
    }

    public double getTargetVelocity() {
        return targetVelocity;
    }

    /** True while over-current is seen but has not lasted {@link #STALL_TIMEOUT_MS}: a suspicion. */
    public boolean isStallSuspected() {
        return jamDetector.isStallSuspected();
    }

    public boolean isUnjamming() {
        return jamDetector.isUnjamming(clock.nowMs());
    }

    public int getUnjamAttempts() {
        return jamDetector.getAttempts();
    }

    /** True once anti-jam has exhausted {@link #MAX_UNJAM_ATTEMPTS} and stopped trying. */
    public boolean hasGivenUpUnjamming() {
        return jamDetector.hasGivenUp();
    }

    public void update() {
        if (!motor.isAvailable()) return;
        long now = clock.nowMs();

        if (mode == Mode.INTAKING && !hasPiece && capturedSupplier.getAsBoolean()) {
            hasPiece = true;
        }

        // Pushed in every loop so edits to the public statics reach the detector.
        jamDetector.configure(
                STALL_CURRENT_AMPS, STALL_TIMEOUT_MS, UNJAM_DURATION_MS, MAX_UNJAM_ATTEMPTS);

        // Anti-jam applies while actively intaking only; a roller held still against a full
        // storage draws no current worth interpreting.
        boolean blocked = isBlockedByFullStorage();
        boolean antiJamEligible = ANTI_JAM_ENABLED && mode == Mode.INTAKING && !blocked;

        if (jamDetector.update(now, antiJamEligible, motor.getCurrentAmps())) {
            motor.write(UNJAM_TICKS_PER_SEC);
            return;
        }

        motor.write(blocked ? 0 : targetVelocity);
    }

    // ---- Ivy commands ----

    public Command intakeCommand() {
        return runUntilInterrupted(this::intake);
    }

    public Command outtakeCommand() {
        return runUntilInterrupted(this::outtake);
    }

    public Command ejectCommand() {
        return runUntilInterrupted(this::eject);
    }

    public Command stopCommand() {
        return Command.build()
                .setStart(this::stop)
                .setDone(() -> true)
                .requiring(this);
    }

    /** Runs at {@code ticksPerSec} for {@code ms} on the injected clock, then stops. */
    public Command runForMs(double ticksPerSec, long ms) {
        final long[] startedAt = new long[1];
        return Command.build()
                .setStart(() -> {
                    startedAt[0] = clock.nowMs();
                    setVelocity(ticksPerSec);
                })
                .setDone(() -> clock.nowMs() - startedAt[0] >= ms)
                .setEnd(ec -> stop())
                .requiring(this);
    }

    /**
     * Intakes until {@code captured} reports a piece, marking possession only when it finishes
     * naturally; an interrupted capture does not claim a piece.
     */
    public Command captureCommand(BooleanSupplier captured) {
        return Command.build()
                .setStart(this::intake)
                .setDone(captured)
                .setEnd(ec -> {
                    // Judge by the sensor, not the end condition: a deadline or parallel group
                    // that finishes forwards its own NATURALLY to unfinished children.
                    if (captured.getAsBoolean()) markCaptured();
                    stop();
                })
                .requiring(this);
    }

    /** {@link #captureCommand(BooleanSupplier)} on the wired captured supplier. */
    public Command captureCommand() {
        return captureCommand(() -> capturedSupplier.getAsBoolean());
    }

    /**
     * Schedule once at OpMode init. Suspends when a real intake command takes the resource and
     * resumes when that command ends, idling the roller. Logic in {@code setExecute} because the
     * Scheduler's resume path does not re-call {@code start()}; {@link BlockedBehavior#QUEUE} so
     * it is not silently dropped if something already holds the intake at init.
     */
    public Command defaultIdleCommand() {
        return Command.build()
                .setExecute(this::stop)
                .setDone(() -> false)
                .setEnd(ec -> stop())
                .setPriority(DEFAULT_IDLE_PRIORITY)
                .setInterruptedBehavior(InterruptedBehavior.SUSPEND)
                .setBlockedBehavior(BlockedBehavior.QUEUE)
                .requiring(this);
    }

    private Command runUntilInterrupted(Runnable start) {
        return Command.build()
                .setStart(start)
                .setDone(() -> false)
                .setEnd(ec -> stop())
                .requiring(this);
    }
}
