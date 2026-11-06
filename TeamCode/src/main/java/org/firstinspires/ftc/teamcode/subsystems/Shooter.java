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
 * The rear-firing shooter, three motors:
 * <ul>
 *   <li>the <b>flywheel</b> and the <b>counter-roller</b> opposite it, each with its own fixed
 *       speed and its own gains;</li>
 *   <li>the <b>up-wheels</b> that carry balls up into them, which are only ever on or off.
 *       {@code commands/Shoot} runs them together with the intake for each feed pulse.</li>
 * </ul>
 * Aiming is the drivetrain's job, through {@link #HEADING_OFFSET_RAD}.
 *
 * <h2>Everything here is ticks per second</h2>
 * {@code DcMotorEx.getVelocity()} reports encoder ticks per second, so that is the unit of the
 * targets and the tolerance. There is no {@code TICKS_PER_REV} and no RPM anywhere: these are not
 * goBILDA motors, nobody has counted their encoder's ticks per revolution, and a guessed conversion
 * would make every number on the card a lie.
 *
 * <h2>Feedforward and a proportional term, not the hub's velocity PID</h2>
 * {@link #update()} writes open-loop power to each wheel, {@code kV * target + kP * error + kS}, with
 * the motors left in their default run mode. No {@code RUN_USING_ENCODER}, no {@code setVelocity}:
 * the hub's velocity loop integrates, and on a wheel that is unloaded except for the instant a piece
 * crosses it, the integrator is exactly what makes recovery slow and then overshoot. {@code kV}
 * carries the steady-state speed, {@code kP} closes what is left of the gap, {@code kS} pays for belt
 * and bearing drag. Both wheels' gains (the counter-roller's are {@code ROLLER_*}) are tuned with the
 * {@code Shooter Tuner} OpMode on the Driver Station.
 *
 * <h2>Arming, not spinning up</h2>
 * The wheels are either holding their targets or off; there is no idle speed. {@link #armedCommand()}
 * owns the resource for as long as a shot needs and {@link #defaultIdleCommand()} disarms whenever
 * nothing owns it. Disarming also stops the up-wheels.
 */
public class Shooter {
    /**
     * The flywheel's gains: {@code kV} is power per tick per second, {@code kP} power per tick per
     * second of error, {@code kS} the power that just overcomes drag. Tune them with the
     * {@code Shooter Tuner} OpMode (group Tuning) and paste its two lines here.
     */
    public static double kS = 0.068, kV = 0.00039, kP = 0.005;
    /** The counter-roller's gains, same meaning, tuned the same way. */
    public static double ROLLER_kS = 0.045, ROLLER_kV = 0.00037, ROLLER_kP = 0.0011;

    /**
     * The one speed each wheel holds whenever it is armed, in Teleop and in every autonomous. There
     * is no distance table and no driver trim: change these two numbers and redeploy. The Shooter
     * Tuner edits them too, and a value typed there lasts until the Robot Controller app restarts.
     */
    public static double TARGET_TICKS_PER_SEC = 1500;
    public static double ROLLER_TARGET_TICKS_PER_SEC = 2100;
    /** How close counts as at target, for both wheels. */
    public static double TOLERANCE_TICKS_PER_SEC = 50;

    /**
     * The counter-roller spins against the flywheel across the piece, so it runs reversed. Read once
     * at construction. Check on TestHardware that the two wheels counter-rotate.
     */
    public static boolean ROLLER_REVERSED = true;

    /** Up-wheels power when on. Flip {@link #FEEDER_REVERSED} if they carry balls down. */
    public static double FEEDER_POWER = 1.0;
    public static boolean FEEDER_REVERSED = false;

    /**
     * Direction the shooter fires, relative to the robot's forward (intake) axis, radians CCW.
     * {@code Math.PI} = out the rear: aiming means turning the robot's back to the HIVE. The robot
     * heading that aims at field bearing b is {@code b - HEADING_OFFSET_RAD}.
     */
    public static double HEADING_OFFSET_RAD = Math.PI;

    public static int DEFAULT_IDLE_PRIORITY = -1;

    private final DcMotorEx flywheel;
    private final DcMotorEx roller;
    private final DcMotorEx feeder;

    private boolean armed = false;
    private boolean feederOn = false;
    private double target = TARGET_TICKS_PER_SEC;
    private double rollerTarget = ROLLER_TARGET_TICKS_PER_SEC;
    private double lastFlywheelPower = Double.NaN;
    private double lastRollerPower = Double.NaN;
    private double lastFeederPower = Double.NaN;

    public Shooter(HardwareMap hardwareMap) {
        this(hardwareMap, HardwareNames.FLYWHEEL_MOTOR, HardwareNames.COUNTER_ROLLER_MOTOR,
                HardwareNames.FEEDER_MOTOR);
    }

    public Shooter(HardwareMap hardwareMap, String flywheelName, String rollerName, String feederName) {
        flywheel = hardwareMap.get(DcMotorEx.class, flywheelName);
        roller = hardwareMap.get(DcMotorEx.class, rollerName);
        feeder = hardwareMap.get(DcMotorEx.class, feederName);
        flywheel.setDirection(DcMotorSimple.Direction.FORWARD);
        roller.setDirection(ROLLER_REVERSED
                ? DcMotorSimple.Direction.REVERSE : DcMotorSimple.Direction.FORWARD);
        feeder.setDirection(FEEDER_REVERSED
                ? DcMotorSimple.Direction.REVERSE : DcMotorSimple.Direction.FORWARD);
        // FLOAT: braking a flywheel to a stop is how a belt gets stripped. A disarmed wheel coasts.
        flywheel.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        roller.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        // BRAKE: stopped up-wheels hold the balls where they are instead of letting them roll into the wheels.
        feeder.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        // No setMode: the default run mode takes the power update() computes. The encoders are still
        // wired, so getVelocity() reads them either way.
        writeWheels(0, 0);
        writeFeeder(0);
    }

    /** Begins holding the targets; the power lands on the next {@link #update()}. */
    public void arm() {
        armed = true;
    }

    /**
     * Stops both wheels and the up-wheels now rather than next loop, so a disarm is never one loop of
     * thrown piece. Also stops wheels left running by {@link #setRawPower}.
     */
    public void disarm() {
        feederOff();
        writeFeeder(0);
        armed = false;
        writeWheels(0, 0);   // writes only on change, so calling this every loop costs nothing
    }

    /**
     * Shooter Tuner only: fixed open-loop power on each wheel, no gains. Do not call {@link #update()}
     * while using this, it would overwrite it; {@link #disarm()} stops it.
     */
    public void setRawPower(double flywheelPower, double rollerPower) {
        armed = false;
        writeWheels(flywheelPower, rollerPower);
    }

    /** The power last sent to the flywheel. Can be above 1: the hub clips it to full power. */
    public double getFlywheelPower() {
        return lastFlywheelPower;
    }

    /** The power last sent to the counter-roller. Can be above 1: the hub clips it to full power. */
    public double getRollerPower() {
        return lastRollerPower;
    }

    public boolean isArmed() {
        return armed;
    }

    // ---- Up-wheels: on or off ----

    /** Up-wheels on; the power lands on the next {@link #update()}. */
    public void feederOn() {
        feederOn = true;
    }

    public void feederOff() {
        feederOn = false;
    }

    public boolean isFeederOn() {
        return feederOn;
    }

    // ---- Targets ----
    // A new Shooter holds TARGET_TICKS_PER_SEC / ROLLER_TARGET_TICKS_PER_SEC. Only the tuners change them.

    /** The flywheel's target. */
    public void setTarget(double ticksPerSec) {
        target = ticksPerSec;
    }

    public double getTarget() {
        return target;
    }

    public void setRollerTarget(double ticksPerSec) {
        rollerTarget = ticksPerSec;
    }

    public double getRollerTarget() {
        return rollerTarget;
    }

    /** Measured flywheel speed in ticks per second. */
    public double getVelocity() {
        return flywheel.getVelocity();
    }

    /** Measured counter-roller speed in ticks per second. */
    public double getRollerVelocity() {
        return roller.getVelocity();
    }

    /**
     * Both wheels within {@link #TOLERANCE_TICKS_PER_SEC} of their targets. One comparison each, no
     * dwell and no latch. It says nothing about being armed — a stopped wheel with a target of 0 is
     * at target — so a command that gates a shot on it checks {@link #isArmed()} as well.
     */
    public boolean atTarget() {
        return Math.abs(target - getVelocity()) < TOLERANCE_TICKS_PER_SEC
                && Math.abs(rollerTarget - getRollerVelocity()) < TOLERANCE_TICKS_PER_SEC;
    }

    /** Writes hardware. Called once per loop from {@code Robot.writeActuators()}, after commands run. */
    public void update() {
        if (armed) {
            // A wheel with a target of 0 is off. The Shooter Tuner uses this to spin one wheel alone.
            // Never below zero: a wheel above its target coasts down (FLOAT), it is not driven backwards.
            writeWheels(target <= 0 ? 0 : Math.max(0, kV * target + kP * (target - getVelocity()) + kS),
                    rollerTarget <= 0 ? 0
                            : Math.max(0, ROLLER_kV * rollerTarget + ROLLER_kP * (rollerTarget - getRollerVelocity()) + ROLLER_kS));
        } else {
            writeWheels(0, 0);
        }
        writeFeeder(feederOn ? FEEDER_POWER : 0);
    }

    // An unchanged setPower is still a bus transaction, so each motor writes only on change.

    private void writeWheels(double flywheelPower, double rollerPower) {
        if (flywheelPower != lastFlywheelPower) {
            flywheel.setPower(flywheelPower);
            lastFlywheelPower = flywheelPower;
        }
        if (rollerPower != lastRollerPower) {
            roller.setPower(rollerPower);
            lastRollerPower = rollerPower;
        }
    }

    private void writeFeeder(double power) {
        if (power != lastFeederPower) {
            feeder.setPower(power);
            lastFeederPower = power;
        }
    }

    /** Both wheels' measured/target speeds, whether they are ready, and the up-wheels. One telemetry line. */
    public String getStatusLine() {
        return String.format(Locale.US, "%s fly %.0f/%.0f  roller %.0f/%.0f t/s %s%s",
                armed ? "ARMED" : "off", getVelocity(), target, getRollerVelocity(), rollerTarget,
                atTarget() ? "READY" : "...", feederOn ? "  up ON" : "");
    }

    // ---- Ivy commands ----

    /**
     * Holds the targets for as long as it owns the shooter, and never finishes: whatever runs the shot
     * decides when the wheels are no longer needed, by interrupting this or by ending the group it
     * sits in. Its end deliberately does not disarm — the next owner takes over inside the same
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
     *
     * <p>{@code feederWanted} is Teleop's manual feed: the up-wheels follow it every loop. During a
     * shot macro this command is suspended, so {@code commands/Shoot} drives the up-wheels itself.
     */
    public Command armedControlCommand(BooleanSupplier armedWanted, BooleanSupplier feederWanted) {
        return Command.build()
                .setExecute(() -> {
                    if (armedWanted.getAsBoolean()) arm();
                    else disarm();
                    if (feederWanted.getAsBoolean()) feederOn();
                    else feederOff();
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
