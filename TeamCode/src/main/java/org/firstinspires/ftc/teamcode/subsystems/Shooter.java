package org.firstinspires.ftc.teamcode.subsystems;

import com.pedropathing.ivy.Command;
import com.pedropathing.ivy.behaviors.BlockedBehavior;
import com.pedropathing.ivy.behaviors.InterruptedBehavior;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;

import org.firstinspires.ftc.teamcode.subsystems.templates.VelocityMotor;
import org.firstinspires.ftc.teamcode.util.hardware.Hardware;
import org.firstinspires.ftc.teamcode.util.hardware.HardwareNames;
import org.firstinspires.ftc.teamcode.util.time.Clock;

/**
 * The compliant flywheel shooter, fixed to the chassis and firing out the rear. One flywheel
 * motor, optionally two, under velocity control; feeding is the {@link Transfer}'s job and
 * aiming is the drivetrain's ({@link #HEADING_OFFSET_RAD}).
 *
 * <h2>Ready means at speed</h2>
 * A shot released before the wheel is at speed is a short shot. {@link #atSpeed()} is the
 * contract every shooting macro waits on, and {@link #holdSpeedCommand()} is how a macro keeps
 * the wheel owned (and spinning) for the whole feed: the default command idles the wheel the
 * moment nothing holds the resource, so a spin-up that ends before the feed would let the wheel
 * wind down under the piece.
 *
 * <p>Speeds are in RPM at the flywheel encoder, converted with {@link #TICKS_PER_REV}, which must
 * be measured for the motor fitted.
 */
public class Shooter {
    /** Encoder ticks per flywheel-motor revolution. 28 for a bare goBILDA 5203; measure it. */
    public static double TICKS_PER_REV = 28;
    /**
     * Direction the flywheel fires, relative to the robot's forward (intake) axis, radians CCW.
     * {@code Math.PI} = out the rear. The robot heading that aims at field bearing b is
     * {@code b - HEADING_OFFSET_RAD}; every aim in {@code Macros} goes through this one number.
     * Confirm on the built robot.
     */
    public static double HEADING_OFFSET_RAD = Math.PI;
    public static double SHOOT_RPM = 3000;
    /** Speed to hold between shots; 0 stops the wheel when nothing owns the shooter. */
    public static double IDLE_RPM = 0;
    /**
     * Second flywheel motor's direction. FORWARD for two wheels on one side turning the same way;
     * REVERSE for an opposed pair, or the two fight. Applied live by {@link #update()}, so
     * {@code Bench: Shooter} can flip it while the pair spins and show whether they fight.
     */
    public static DcMotorSimple.Direction SECOND_MOTOR_DIRECTION = DcMotorSimple.Direction.FORWARD;
    /**
     * At 3000 RPM on a 28-tick encoder (1400 t/s) the hub's velocity estimate wanders more than the
     * old 100 RPM (47 t/s) band, so the wait always ran to its timeout. 5 % of target, and it must
     * hold for {@link #AT_SPEED_LOOPS} consecutive loops before the wheel counts as ready.
     */
    public static double AT_SPEED_TOLERANCE_RPM = 150;
    public static int AT_SPEED_LOOPS = 5;
    /**
     * Velocity-loop gains for the flywheel motor(s), applied only when {@link #CUSTOM_PIDF} is true.
     * Off until measured with {@code Bench: Shooter}: the SDK's per-motor-type defaults may well be
     * better than a guessed F. F = 32767 / max ticks per second (2800 t/s = a 6000 RPM bare motor on
     * a 28-tick encoder); P is a tenth of F, I a tenth of P, as the SDK's own defaults are shaped.
     */
    public static boolean CUSTOM_PIDF = false;
    public static double MAX_TICKS_PER_SEC = 2800;
    public static double PIDF_F = 32767.0 / MAX_TICKS_PER_SEC;
    public static double PIDF_P = 0.1 * PIDF_F;
    public static double PIDF_I = 0.1 * PIDF_P;
    public static double PIDF_D = 0;
    /** A spin-up that has not reached speed by then reports done anyway (battery sag, wrong gain). */
    public static long SPINUP_TIMEOUT_MS = 3000;
    /**
     * After a piece goes through the wheel: wait at least this long (so the speed dip has begun)
     * and then until {@link #atSpeed()} again, or at most {@link #SHOT_RECOVERY_TIMEOUT_MS}. Measure
     * the dip and the recovery with {@code Bench: Shooter}.
     */
    public static long SHOT_RECOVERY_MIN_MS = 150;
    public static long SHOT_RECOVERY_TIMEOUT_MS = 1500;
    public static int DEFAULT_IDLE_PRIORITY = -1;

    /** Derived from the target and the measured speed; see {@link #getMode()}. */
    public enum Mode { IDLE, SPINNING_UP, READY }

    private final VelocityMotor flywheel;
    private final VelocityMotor flywheel2;
    private final Clock clock;
    private double targetRpm = 0;
    /** Consecutive {@link #update()} loops with the measured speed inside the tolerance band. */
    private int inBandLoops = 0;

    public Shooter(HardwareMap hardwareMap) {
        this(hardwareMap, HardwareNames.SHOOTER_MOTOR, HardwareNames.SHOOTER_MOTOR_2, Clock.system());
    }

    public Shooter(HardwareMap hardwareMap, String name, String secondName, Clock clock) {
        this(Hardware.get(hardwareMap, DcMotorEx.class, name),
                secondName == null ? null : Hardware.get(hardwareMap, DcMotorEx.class, secondName),
                clock);
    }

    /** Builds on resolved motors ({@code null} for "not fitted"; the second is optional). */
    public Shooter(DcMotorEx motor, DcMotorEx secondMotor, Clock clock) {
        this.flywheel = new VelocityMotor(motor, DcMotorSimple.Direction.FORWARD,
                DcMotor.ZeroPowerBehavior.FLOAT);
        this.flywheel2 = new VelocityMotor(secondMotor, SECOND_MOTOR_DIRECTION,
                DcMotor.ZeroPowerBehavior.FLOAT);
        this.clock = clock;
        if (CUSTOM_PIDF) applyPidf();     // off: the hub keeps its own, nothing to write
    }

    /**
     * Writes {@code PIDF_*} to both motors when {@link #CUSTOM_PIDF}, and puts the SDK's own
     * coefficients back when it is off, so the bench's toggle really toggles (fixthese R2-A6).
     */
    public void applyPidf() {
        if (CUSTOM_PIDF) {
            flywheel.setVelocityPidf(PIDF_P, PIDF_I, PIDF_D, PIDF_F);
            flywheel2.setVelocityPidf(PIDF_P, PIDF_I, PIDF_D, PIDF_F);
        } else {
            flywheel.restoreSdkPidf();
            flywheel2.restoreSdkPidf();
        }
    }

    /** The coefficients the hub holds for the first flywheel right now (a bus read; bench cards only). */
    public PIDFCoefficients readFlywheelPidf() {
        return flywheel.readPidf();
    }

    public boolean hasSecondMotor() {
        return flywheel2.isAvailable();
    }

    /** Measured speed of the second flywheel motor, or 0 when not fitted. */
    public double getSecondVelocityTicksPerSec() {
        return flywheel2.getVelocity();
    }

    /** Measured speed of the first flywheel motor in ticks per second. */
    public double getVelocityTicksPerSec() {
        return flywheel.getVelocity();
    }

    public boolean isAvailable() {
        return flywheel.isAvailable();
    }

    public static double rpmToTicksPerSec(double rpm) {
        return rpm * TICKS_PER_REV / 60.0;
    }

    public static double ticksPerSecToRpm(double ticksPerSec) {
        return ticksPerSec * 60.0 / TICKS_PER_REV;
    }

    public void setTargetRpm(double rpm) {
        targetRpm = rpm;
        double ticks = rpmToTicksPerSec(rpm);
        flywheel.setTarget(ticks);
        flywheel2.setTarget(ticks);
    }

    public void spinUp() {
        setTargetRpm(SHOOT_RPM);
    }

    public void idle() {
        setTargetRpm(IDLE_RPM);
    }

    public void stop() {
        setTargetRpm(0);
    }

    public double getTargetRpm() {
        return targetRpm;
    }

    /** Measured flywheel speed, or 0 when unavailable. */
    public double getRpm() {
        return ticksPerSecToRpm(flywheel.getVelocity());
    }

    public double getCurrentAmps() {
        return flywheel.getCurrentAmps() + flywheel2.getCurrentAmps();
    }

    /**
     * True once the measured speed has been within {@link #AT_SPEED_TOLERANCE_RPM} of a non-zero
     * target for {@link #AT_SPEED_LOOPS} consecutive loops (counted in {@link #update()}), so one
     * noisy sample through the band cannot release a shot.
     */
    public boolean atSpeed() {
        return inBandLoops >= AT_SPEED_LOOPS;
    }

    /** The raw, single-sample band check; {@link #atSpeed()} is the latched version macros use. */
    public boolean inBandNow() {
        return flywheel.atSpeed(rpmToTicksPerSec(AT_SPEED_TOLERANCE_RPM));
    }

    public Mode getMode() {
        if (targetRpm <= 0) return Mode.IDLE;
        return atSpeed() ? Mode.READY : Mode.SPINNING_UP;
    }

    public void update() {
        flywheel2.setDirection(SECOND_MOTOR_DIRECTION);   // no-op unless it changed (fixthese R2-A9)
        flywheel.update();
        flywheel2.update();
        inBandLoops = inBandNow() ? inBandLoops + 1 : 0;
    }

    // ---- Ivy commands ----

    /**
     * Spins up to {@link #SHOOT_RPM} and finishes once at speed (or after
     * {@link #SPINUP_TIMEOUT_MS}). The wheel keeps spinning after it finishes only while something
     * still owns the shooter, so follow it with {@link #holdSpeedCommand()} in a deadline or
     * parallel group rather than letting the default command idle the wheel mid-cycle.
     */
    public Command spinUpCommand() {
        final long[] startedAt = new long[1];
        return Command.build()
                .setStart(() -> {
                    startedAt[0] = clock.nowMs();
                    spinUp();
                })
                .setDone(() -> !isAvailable() || atSpeed()
                        || clock.nowMs() - startedAt[0] >= SPINUP_TIMEOUT_MS)
                .requiring(this);
    }

    /**
     * Finishes once the wheel is at speed, after {@link #SPINUP_TIMEOUT_MS}, or at once when no
     * shooter is fitted. Requires nothing: it is the wait inside a group in which
     * {@link #holdSpeedCommand()} owns the shooter and sets the target, so two siblings never both
     * claim the resource and the last-executed one silently wins (fixthese C7).
     */
    public Command waitForSpeedCommand() {
        final long[] startedAt = new long[1];
        return Command.build()
                .setStart(() -> startedAt[0] = clock.nowMs())
                .setDone(() -> !isAvailable() || atSpeed()
                        || clock.nowMs() - startedAt[0] >= SPINUP_TIMEOUT_MS);
    }

    /** Holds {@link #SHOOT_RPM} until interrupted, then idles. */
    public Command holdSpeedCommand() {
        return Command.build()
                .setStart(this::spinUp)
                .setDone(() -> false)
                .setEnd(ec -> idle())
                .requiring(this);
    }

    /** Drops to {@link #IDLE_RPM} immediately. */
    public Command idleCommand() {
        return Command.build()
                .setStart(this::idle)
                .setDone(() -> true)
                .requiring(this);
    }

    /** Schedule once at OpMode init: holds {@link #IDLE_RPM} whenever nothing else owns the shooter. */
    public Command defaultIdleCommand() {
        return Command.build()
                .setExecute(this::idle)
                .setDone(() -> false)
                .setEnd(ec -> idle())
                .setPriority(DEFAULT_IDLE_PRIORITY)
                .setInterruptedBehavior(InterruptedBehavior.SUSPEND)
                .setBlockedBehavior(BlockedBehavior.QUEUE)
                .requiring(this);
    }
}
