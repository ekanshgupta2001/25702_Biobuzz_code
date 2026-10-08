package org.firstinspires.ftc.teamcode.subsystems;

import com.pedropathing.ivy.Command;
import com.pedropathing.ivy.behaviors.BlockedBehavior;
import com.pedropathing.ivy.behaviors.InterruptedBehavior;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.util.hardware.HardwareNames;

import java.util.Locale;
import java.util.function.BooleanSupplier;

/**
 * The rear-firing shooter: two motors on GT2/HTD belts driving a counter-rotating pair of 2.9 in
 * flywheels. Feeding it is the {@link Intake}'s job — roller and tunnel are one motor — and aiming
 * it is the drivetrain's, through {@link #HEADING_OFFSET_RAD}.
 *
 * <h2>Everything here is ticks per second</h2>
 * {@code DcMotorEx.getVelocity()} reports encoder ticks per second, so that is the unit of the
 * target, the tolerance and the distance table. There is no {@code TICKS_PER_REV} and no RPM
 * anywhere: these are not goBILDA motors, nobody has counted their encoder's ticks per revolution,
 * and a guessed conversion would make every number on the bench card and in the table a lie. A
 * measured ticks-per-second figure cannot be wrong about itself.
 *
 * <h2>Feedforward and a proportional term, not the hub's velocity PID</h2>
 * {@link #update()} writes open-loop power, {@code kV * target + kP * error + kS}, with the motors
 * left in their default run mode. No {@code RUN_USING_ENCODER}, no {@code setVelocity}, no
 * {@code setPIDFCoefficients}: the hub's velocity loop integrates, and on a belted pair that is
 * unloaded except for the instant a piece crosses the wheels, the integrator is exactly what makes
 * recovery slow and then overshoot. {@code kV} carries the steady-state speed, {@code kP} closes
 * what is left of the gap, {@code kS} pays for belt and bearing drag. The three values are a
 * competition-proven reference robot's, for the same wheel size — a place to start measuring with
 * the Shooter Tuner on the Pedro tuning site, not an answer.
 *
 * <h2>Arming, not spinning up</h2>
 * The wheel is either holding {@link #getTarget()} or off; there is no idle speed, because a wheel
 * held at a speed nobody shoots at is heat and noise. {@link #armedCommand()} owns the resource for
 * as long as a shot needs and {@link #defaultIdleCommand()} disarms whenever nothing owns it.
 */
public class Shooter {
    /**
     * The one gain set: {@code kV} is power per tick per second, {@code kP} power per tick per second
     * of error, {@code kS} the power that just overcomes drag. Tune them with the Shooter Tuner on the
     * Pedro tuning site (http://192.168.43.1:10158), which reports the spin-up time for each set.
     */
    public static double kS = 0.08, kV = 0.00039, kP = 0.01;

    /** Nominal speed, and what a freshly built shooter holds until the table or the driver says otherwise. */
    public static double TARGET_TICKS_PER_SEC = 1300;
    /**
     * The driver's override, for when odometry or the table is not trusted: no pose, a CELL that has
     * moved, a shot from somewhere nobody measured.
     */
    public static double MANUAL_TICKS_PER_SEC = 1300;
    /** How close counts as at target. The reference robot shoots inside 50 t/s; measure the spread. */
    public static double TOLERANCE_TICKS_PER_SEC = 50;

    /**
     * The flywheels oppose each other across the piece, so one motor runs reversed and both then
     * take the same power. This is geometry, not an option: it is a constant only so that a swapped
     * pair of leads can be answered here instead of in two sign flips. Read once at construction —
     * changing it needs a restart, which is right for a fact about the gearbox.
     */
    public static boolean SECOND_MOTOR_REVERSED = true;

    /**
     * Direction the shooter fires, relative to the robot's forward (intake) axis, radians CCW.
     * {@code Math.PI} = out the rear, which is how V1 is built: aiming means turning the robot's back
     * to the HIVE. The robot heading that aims at field bearing b is {@code b - HEADING_OFFSET_RAD};
     * every aim in {@code Macros} goes through this one number. Confirm it on the built robot.
     */
    public static double HEADING_OFFSET_RAD = Math.PI;

    public static int DEFAULT_IDLE_PRIORITY = -1;

    /**
     * Distance to the CELL, inches, against the speed that scores from there. <b>Every number below
     * is a placeholder.</b> Measure them in Teleop's fixed-speed mode: park at a distance, trim the
     * speed until the piece drops in the middle of the CELL, write the pair down, move on.
     *
     * <p>A table, and not a fit, on purpose. The reference team fitted a line to their measurements,
     * then a quartic, then a quadratic, and shipped none of the three — all of them are still
     * commented out in their code. A fit is smooth where the shot is not, and it extrapolates with
     * total confidence past the last point anyone measured. {@link #interpolate} clamps at both ends
     * instead, which is the honest answer outside the measured range.
     */
    private static final double[] DISTANCES_INCHES = {24, 48, 72, 96};
    private static final double[] TICKS_PER_SEC = {1150, 1250, 1400, 1550};

    /** The flywheel whose encoder is the speed signal; see {@link #getVelocity()}. */
    private final DcMotorEx left;
    private final DcMotorEx right;

    private boolean armed = false;
    private double target = TARGET_TICKS_PER_SEC;
    private double lastWritten = Double.NaN;

    public Shooter(HardwareMap hardwareMap) {
        this(hardwareMap, HardwareNames.SHOOTER_MOTOR, HardwareNames.SHOOTER_MOTOR_2);
    }

    public Shooter(HardwareMap hardwareMap, String leftName, String rightName) {
        left = hardwareMap.get(DcMotorEx.class, leftName);
        right = hardwareMap.get(DcMotorEx.class, rightName);
        left.setDirection(DcMotorSimple.Direction.FORWARD);
        right.setDirection(SECOND_MOTOR_REVERSED
                ? DcMotorSimple.Direction.REVERSE : DcMotorSimple.Direction.FORWARD);
        // FLOAT: braking a flywheel to a stop is how a belt gets stripped. A disarmed wheel coasts.
        left.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        right.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        // No setMode: the default run mode takes the power update() computes. The encoder is still
        // wired, so getVelocity() reads it either way.
        write(0);
    }

    /** Begins holding {@link #getTarget()}; the power lands on the next {@link #update()}. */
    public void arm() {
        armed = true;
    }

    /** Stops the wheel now rather than next loop, so a disarm is never one loop of thrown piece. */
    public void disarm() {
        if (armed) {
            armed = false;
            write(0);
        }
    }

    public boolean isArmed() {
        return armed;
    }

    public void setTarget(double ticksPerSec) {
        target = ticksPerSec;
    }

    public double getTarget() {
        return target;
    }

    /**
     * Measured flywheel speed in ticks per second. One motor is the speed signal: the pair is belted
     * to the same piece, and averaging two encoders only buys a second bus read and a number that
     * belongs to neither wheel.
     */
    public double getVelocity() {
        return left.getVelocity();
    }

    /**
     * One symmetric comparison, no dwell and no latch. The old shooter had to see 100 ms in band
     * before releasing a shot because the hub's velocity PID kept wandering back out; this scheme is
     * back in band within a couple of loops, so a dwell would only add its own length to every shot.
     * It says nothing about being armed — a stopped wheel with a target of 0 is at target — so a
     * command that gates a shot on it checks {@link #isArmed()} as well.
     */
    public boolean atTarget() {
        return Math.abs(target - getVelocity()) < TOLERANCE_TICKS_PER_SEC;
    }

    /**
     * Sets the target from the measured table. A NaN distance (no pose to measure from) leaves the
     * target alone: the last trusted speed beats a speed derived from nothing.
     */
    public void setTargetForDistance(double inches) {
        if (!Double.isNaN(inches)) target = interpolate(inches);
    }

    /** The driver's override: shoot at {@link #MANUAL_TICKS_PER_SEC}, whatever the table thinks. */
    public void setManualTarget() {
        target = MANUAL_TICKS_PER_SEC;
    }

    /** Linear between the measured points, flat outside them. */
    private static double interpolate(double inches) {
        if (inches <= DISTANCES_INCHES[0]) return TICKS_PER_SEC[0];
        for (int i = 1; i < DISTANCES_INCHES.length; i++) {
            if (inches <= DISTANCES_INCHES[i]) {
                double span = DISTANCES_INCHES[i] - DISTANCES_INCHES[i - 1];
                double f = (inches - DISTANCES_INCHES[i - 1]) / span;
                return TICKS_PER_SEC[i - 1] + f * (TICKS_PER_SEC[i] - TICKS_PER_SEC[i - 1]);
            }
        }
        return TICKS_PER_SEC[TICKS_PER_SEC.length - 1];
    }

    /** Writes hardware. Called once per loop from {@code Robot.writeActuators()}, after commands run. */
    public void update() {
        if (armed) write(kV * target + kP * (target - getVelocity()) + kS);
        else write(0);
    }

    /** Both motors take the same power; {@link #SECOND_MOTOR_REVERSED} is what opposes them. */
    private void write(double power) {
        if (power != lastWritten) {     // an unchanged setPower is still two bus transactions
            left.setPower(power);
            right.setPower(power);
            lastWritten = power;
        }
    }

    /** Target, measured, and whether the wheel is ready. One line, for a telemetry card. */
    public String getStatusLine() {
        return String.format(Locale.US, "%s %.0f/%.0f t/s %s",
                armed ? "ARMED" : "off", getVelocity(), target, atTarget() ? "READY" : "...");
    }

    // ---- Ivy commands ----

    /**
     * Holds the target for as long as it owns the shooter, and never finishes: whatever runs the shot
     * decides when the wheel is no longer needed, by interrupting this or by ending the group it sits
     * in. Its end deliberately does not disarm — the next owner takes over inside the same
     * {@code Scheduler} pass, so an armed wheel is never zeroed across a shot, and when nothing
     * re-holds it {@link #defaultIdleCommand()} disarms on its next loop.
     */
    public Command armedCommand() {
        return Command.build()
                .setStart(this::arm)
                .setDone(() -> false)
                .requiring(this);
    }

    /**
     * The operator's flywheel toggle, as a default command: it re-reads {@code armed} every loop
     * rather than being re-scheduled on every press. Ivy has no duplicate guard, so re-scheduling
     * would re-run {@code start()} every loop; and because Ivy <em>ends</em> rather than suspends a
     * preempted priority-0 command, a hold scheduled on the press would have to be restored by hand
     * after every shot. At priority -1 with {@code SUSPEND} a shooting cycle's own hold preempts this
     * and it resumes by itself, still armed.
     */
    public Command armedControlCommand(BooleanSupplier armedWanted) {
        return Command.build()
                .setExecute(() -> {
                    if (armedWanted.getAsBoolean()) arm();
                    else disarm();
                })
                .setDone(() -> false)
                .setEnd(ec -> disarm())
                .setPriority(DEFAULT_IDLE_PRIORITY)
                .setInterruptedBehavior(InterruptedBehavior.SUSPEND)
                .setBlockedBehavior(BlockedBehavior.QUEUE)
                .requiring(this);
    }

    /**
     * Autonomous's default: disarms whenever nothing else owns the shooter. The logic is in
     * {@code setExecute} because the Scheduler's resume path does not re-call {@code start()};
     * {@link BlockedBehavior#QUEUE} so it is not silently dropped if something already holds the
     * shooter at init.
     */
    public Command defaultIdleCommand() {
        return Command.build()
                .setExecute(this::disarm)
                .setDone(() -> false)
                .setEnd(ec -> disarm())
                .setPriority(DEFAULT_IDLE_PRIORITY)
                .setInterruptedBehavior(InterruptedBehavior.SUSPEND)
                .setBlockedBehavior(BlockedBehavior.QUEUE)
                .requiring(this);
    }
}
