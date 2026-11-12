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
 * The shooter: one flywheel, firing out of the robot's front. There is no feeder motor: the intake
 * carries balls into the wheel, and {@code commands/Shoot} pulses it for each shot. Aiming is the
 * drivetrain's job, through {@link #HEADING_OFFSET_RAD}.
 *
 * <h2>Where the speed comes from</h2>
 * {@code Robot.readSensors()} sets the target every loop: from the Limelight's tag distance through
 * {@link #SPEED_TABLE} ({@link #setSpeedFromDistance}), or the fixed {@link #TARGET_TICKS_PER_SEC}
 * when auto speed is off ({@link #useFixedSpeed}). No tag seen recently = the fixed speed too.
 *
 * <h2>Everything here is ticks per second</h2>
 * {@code DcMotorEx.getVelocity()} reports encoder ticks per second, so that is the unit of the
 * targets and the tolerance. There is no {@code TICKS_PER_REV} and no RPM anywhere: nobody has
 * counted this motor's encoder ticks per revolution, and a guessed conversion would make every number
 * on the card a lie.
 *
 * <h2>Feedforward and a proportional term, not the hub's velocity PID</h2>
 * {@link #update()} writes open-loop power, {@code kV * target + kP * error + kS}, with the motor left
 * in its default run mode. No {@code RUN_USING_ENCODER}, no {@code setVelocity}: the hub's velocity
 * loop integrates, and on a wheel that is unloaded except for the instant a piece crosses it, the
 * integrator is exactly what makes recovery slow and then overshoot. {@code kV} carries the
 * steady-state speed, {@code kP} closes what is left of the gap, {@code kS} pays for belt and bearing
 * drag. Tune them with the {@code Shooter Tuner} OpMode on the Driver Station.
 *
 * <h2>Arming, not spinning up</h2>
 * The wheel is either holding its target or off; there is no idle speed. {@link #armedCommand()}
 * owns the resource for as long as a shot needs and {@link #defaultIdleCommand()} disarms whenever
 * nothing owns it.
 */
public class Shooter {
    /**
     * The flywheel's gains: {@code kV} is power per tick per second, {@code kP} power per tick per
     * second of error, {@code kS} the power that just overcomes drag. Tune them with the
     * {@code Shooter Tuner} OpMode (group Tuning) and paste its line here.
     */
    public static double kS = 0.068, kV = 0.00039, kP = 0.005;

    /**
     * The fixed speed: used when auto speed is off, and the fallback when no tag has been seen for
     * {@link #TAG_HOLD_MS}. Read every loop, so a live edit applies at once; it only lasts until the
     * Robot Controller app restarts, so paste the value you want to keep here.
     */
    public static double TARGET_TICKS_PER_SEC = 1500;
    /** How close counts as at target. */
    public static double TOLERANCE_TICKS_PER_SEC = 50;

    /**
     * Flywheel speed per tag distance: rows of {distance from the camera to the tag in inches, ticks
     * per second}, nearest first. Between rows the speed is interpolated; past either end it is the end
     * row's speed.
     *
     * <p>EMPTY UNTIL MEASURED: with no rows every distance gives {@link #TARGET_TICKS_PER_SEC}, so TAG
     * and FIXED shoot the same. To fill a row: Teleop in FIXED speed, park at a distance facing the
     * CELL, trim the speed with the dpad until the shots land, then copy the tag distance and the speed
     * from the card into a row, e.g. {@code {48, 1450},}. Four or so distances across the range you
     * shoot from.
     */
    public static double[][] SPEED_TABLE = {
    };

    /**
     * After the tag is lost, keep its speed this long before going back to the fixed speed. Covers a
     * frame or two where a ball crosses the camera.
     */
    public static long TAG_HOLD_MS = 1000;

    /**
     * Direction the shot leaves, relative to the robot's front, radians CCW. {@code 0} = out the front,
     * the way the Limelight looks: aiming means turning the robot's front to the CELL. The robot
     * heading that aims at field bearing b is {@code b - HEADING_OFFSET_RAD}.
     */
    public static double HEADING_OFFSET_RAD = 0;

    public static int DEFAULT_IDLE_PRIORITY = -1;

    private final DcMotorEx flywheel;

    private boolean armed = false;
    private double target = TARGET_TICKS_PER_SEC;
    private double lastPower = Double.NaN;
    /** When a tag last gave a distance; -1 = never, or forgotten by {@link #useFixedSpeed()}. */
    private long lastTagMs = -1;

    public Shooter(HardwareMap hardwareMap) {
        this(hardwareMap, HardwareNames.FLYWHEEL_MOTOR);
    }

    public Shooter(HardwareMap hardwareMap, String flywheelName) {
        flywheel = hardwareMap.get(DcMotorEx.class, flywheelName);
        flywheel.setDirection(DcMotorSimple.Direction.FORWARD);
        // FLOAT: braking a flywheel to a stop is how a belt gets stripped. A disarmed wheel coasts.
        flywheel.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        // No setMode: the default run mode takes the power update() computes. The encoder is still
        // wired, so getVelocity() reads it either way.
        write(0);
    }

    /** Begins holding the target; the power lands on the next {@link #update()}. */
    public void arm() {
        armed = true;
    }

    /**
     * Stops the wheel now rather than next loop, so a disarm is never one loop of thrown piece. Also
     * stops a wheel left running by {@link #setRawPower}.
     */
    public void disarm() {
        armed = false;
        write(0);   // writes only on change, so calling this every loop costs nothing
    }

    public boolean isArmed() {
        return armed;
    }

    /**
     * Shooter Tuner only: fixed open-loop power, no gains. Do not call {@link #update()} while using
     * this, it would overwrite it; {@link #disarm()} stops it.
     */
    public void setRawPower(double power) {
        armed = false;
        write(power);
    }

    /** The power last sent to the flywheel. Can be above 1: the hub clips it to full power. */
    public double getPower() {
        return lastPower;
    }

    // ---- Target ----

    public void setTarget(double ticksPerSec) {
        target = ticksPerSec;
    }

    public double getTarget() {
        return target;
    }

    /**
     * The speed for a tag distance, from {@link #SPEED_TABLE}: interpolated between rows, the end row's
     * speed past either end. An empty table (or NaN) gives {@link #TARGET_TICKS_PER_SEC}.
     */
    public static double speedForDistance(double inches) {
        double[][] table = SPEED_TABLE;
        if (table == null || table.length == 0 || Double.isNaN(inches)) return TARGET_TICKS_PER_SEC;
        if (inches <= table[0][0]) return table[0][1];
        for (int i = 1; i < table.length; i++) {
            // Reached only when inches > the previous row, so near[0] < far[0] here: no divide by zero.
            if (inches <= table[i][0]) {
                double[] near = table[i - 1];
                double[] far = table[i];
                double fraction = (inches - near[0]) / (far[0] - near[0]);
                return near[1] + fraction * (far[1] - near[1]);
            }
        }
        return table[table.length - 1][1];
    }

    /**
     * Called every loop with the tag distance in inches, or NaN when no tag is visible. A distance
     * sets the target from the table. NaN keeps the last tag speed for {@link #TAG_HOLD_MS} (a ball
     * crossing the camera should not change the speed mid-shot), then falls back to the fixed speed.
     */
    public void setSpeedFromDistance(double inchesOrNaN) {
        if (!Double.isNaN(inchesOrNaN)) {
            target = speedForDistance(inchesOrNaN);
            lastTagMs = System.currentTimeMillis();
        } else if (!isSpeedFromTag()) {
            target = TARGET_TICKS_PER_SEC;
        }
    }

    /** Target = {@link #TARGET_TICKS_PER_SEC}, and forget any tag. Called every loop in fixed mode. */
    public void useFixedSpeed() {
        target = TARGET_TICKS_PER_SEC;
        lastTagMs = -1;
    }

    /** True while the target comes from a tag (seen now, or within {@link #TAG_HOLD_MS}). */
    public boolean isSpeedFromTag() {
        return lastTagMs >= 0 && System.currentTimeMillis() - lastTagMs < TAG_HOLD_MS;
    }

    /** Measured flywheel speed in ticks per second. */
    public double getVelocity() {
        return flywheel.getVelocity();
    }

    /**
     * Within {@link #TOLERANCE_TICKS_PER_SEC} of the target. One comparison, no dwell and no latch.
     * It says nothing about being armed — a stopped wheel with a target of 0 is at target — so a
     * command that gates a shot on it checks {@link #isArmed()} as well.
     */
    public boolean atTarget() {
        return Math.abs(target - getVelocity()) < TOLERANCE_TICKS_PER_SEC;
    }

    /** Writes hardware. Called once per loop from {@code Robot.writeActuators()}, after commands run. */
    public void update() {
        if (armed) {
            // A target of 0 is off. Never below zero: a wheel above its target coasts down (FLOAT),
            // it is not driven backwards.
            write(target <= 0 ? 0 : Math.max(0, kV * target + kP * (target - getVelocity()) + kS));
        } else {
            write(0);
        }
    }

    /** Writes only on change: an unchanged setPower is still a bus transaction. */
    private void write(double power) {
        if (power != lastPower) {
            flywheel.setPower(power);
            lastPower = power;
        }
    }

    /** One telemetry line, e.g. {@code ARMED 1480/1500 t/s READY  (tag)}. */
    public String getStatusLine() {
        return String.format(Locale.US, "%s %.0f/%.0f t/s %s  (%s)",
                armed ? "ARMED" : "off", getVelocity(), target, atTarget() ? "READY" : "...",
                isSpeedFromTag() ? "tag" : "fixed");
    }

    // ---- Ivy commands ----

    /**
     * Holds the target for as long as it owns the shooter, and never finishes: whatever runs the shot
     * decides when the wheel is no longer needed, by interrupting this or by ending the group it
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
     * The operator's flywheel toggle, as a default command: it re-reads {@code armedWanted} every loop
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
