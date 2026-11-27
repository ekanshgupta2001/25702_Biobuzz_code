package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.util.hardware.HardwareNames;

import java.util.Locale;

/**
 * The shooter, firing out of the robot's front, three motors:
 * <ul>
 *   <li>the <b>flywheel</b> and the <b>counter-roller</b> opposite it, each with its own speed and
 *       its own gains;</li>
 *   <li>the <b>feeder</b> (up-wheels) that carries balls up into them: on ({@link #FEEDER_POWER}),
 *       off, or backwards to clear a stuck ball ({@link #feederOut()}).</li>
 * </ul>
 * The shot leaves the robot's front, the way the Limelight looks: the driver aims by turning the robot.
 *
 * <h2>Where the speeds come from</h2>
 * {@code Robot.readSensors()} sets both targets every loop from the distance to the CELL, through
 * {@link #SPEED_TABLE} ({@link #setSpeedFromDistance}). That distance is the Limelight's tag distance
 * (and the robot's position too, once the Pinpoint is set up: see {@code Robot.updateShotDistance}). No distance for
 * {@link #TAG_HOLD_MS} = the fixed {@link #TARGET_TICKS_PER_SEC} / {@link #ROLLER_TARGET_TICKS_PER_SEC}.
 *
 * <h2>Everything here is ticks per second</h2>
 * {@code DcMotorEx.getVelocity()} reports encoder ticks per second, so that is the unit of the
 * targets and the tolerance. There is no {@code TICKS_PER_REV} and no RPM anywhere: nobody has
 * counted these motors' encoder ticks per revolution, and a guessed conversion would make every number
 * on the card a lie.
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
 * <h2>Two kP values per wheel: small near the target, big away from it</h2>
 * Within {@link #BIG_BAND_TICKS_PER_SEC} of its target a wheel uses its small kP ({@link #kP},
 * {@link #ROLLER_kP}), the one that holds the speed steady. Farther away than that (spinning up, or
 * just after a ball took speed out of it) it uses its big kP ({@link #BIG_kP},
 * {@link #ROLLER_BIG_kP}) to get back sooner. A motor cannot be given more than full power, so once
 * the small kP already asks for full power the big one changes nothing: the Shooter Tuner's VOLLEY
 * test shows when that is the case.
 *
 * <h2>Battery voltage (off unless {@link #TUNED_AT_VOLTS} is set)</h2>
 * The same power turns a wheel slower on a tired battery, so the wheels settle below their targets.
 * With {@link #TUNED_AT_VOLTS} set to the voltage the gains were tuned at, the feedforward is scaled
 * by tuned volts / volts now. At 0 nothing is scaled.
 *
 * <h2>Arming, not spinning up</h2>
 * The wheels are either holding their targets or off; there is no idle speed. The OpMode calls
 * {@link #arm()} / {@link #disarm()} and {@link #feederOn()} / {@link #feederOff()} /
 * {@link #feederOut()}, and {@link #update()} once per loop writes the motors. Disarming also stops
 * the feeder.
 */
public class Shooter {
    /**
     * The flywheel's gains: {@code kV} is power per tick per second, {@code kP} power per tick per
     * second of error, {@code kS} the power that just overcomes drag. Tune them with the
     * {@code Shooter Tuner} OpMode (group Tuning) and paste its two lines here.
     */
    public static double kS = 0.068, kV = 0.00039, kP = 0.005;
    /**
     * The counter-roller's gains, same meaning, tuned the same way. These are the values from before
     * the counter-roller was rebuilt: a starting point only, re-tune them.
     */
    public static double ROLLER_kS = 0.045, ROLLER_kV = 0.00037, ROLLER_kP = 0.0011;

    /**
     * The big kP of each wheel, used while it is {@link #BIG_BAND_TICKS_PER_SEC} or more away from its
     * target. Starting point: 1.15 x the small kP. Raise them with the Shooter Tuner's VOLLEY test.
     */
    public static double BIG_kP = 0.00575, ROLLER_BIG_kP = 0.001265;
    /** Closer to the target than this, a wheel uses its small kP; at this or farther, its big kP. */
    public static double BIG_BAND_TICKS_PER_SEC = 150;

    /**
     * The battery voltage the gains were tuned at (the Shooter Tuner's top line shows it). 0 = off:
     * the power is not changed for the battery. Above 0, the kV and kS part of each wheel's power is
     * multiplied by this / the voltage now, limited to {@link #MIN_VOLTS_SCALE}..{@link #MAX_VOLTS_SCALE}.
     */
    public static double TUNED_AT_VOLTS = 0;
    public static double MIN_VOLTS_SCALE = 0.8, MAX_VOLTS_SCALE = 1.3;

    /**
     * The fixed speeds: the fallback when there has been no distance for {@link #TAG_HOLD_MS}, and
     * what Auto HARDCODED shoots at. Read every loop, so a live edit applies at once; it only lasts until the
     * Robot Controller app restarts, so paste the values you want to keep here.
     */
    public static double TARGET_TICKS_PER_SEC = 1500;
    public static double ROLLER_TARGET_TICKS_PER_SEC = 900;
    /** How close counts as at target, for both wheels. */
    public static double TOLERANCE_TICKS_PER_SEC = 50;

    /**
     * Speeds per distance: rows of {distance from the camera to the tag in inches, flywheel ticks
     * per second, counter-roller ticks per second}, nearest first. The robot's position distance is
     * corrected onto the tag distance, so these inches are the ones both of them use. Between rows both speeds are
     * interpolated; past either end they are the end row's speeds.
     *
     * <p>With no rows every distance gives the fixed speeds. To add or redo rows: {@code Shooter Tuner},
     * Test = TABLE. At each spot it shows the camera distance, you change both speeds until the shots
     * land, Y saves the row, and the bottom of its screen is this whole array, sorted, ready to type
     * in here.
     *
     * <p>Every row needs all three numbers, and the rows must get farther each time: a short row or
     * one out of order stops the OpMode at init with a message.
     */
    public static double[][] SPEED_TABLE = {
            // {inches to the tag, flywheel t/s, counter-roller t/s},  nearest first.
            // Example: a spot 48 in from the tag that shoots well at the fixed speeds above
            // (TARGET_TICKS_PER_SEC 1500, ROLLER_TARGET_TICKS_PER_SEC 900) is this row:
            // {48, 1500, 900},
            {68, 1600, 700},
            {71, 1600, 900},
            {74, 1500, 900},
            {75, 1500, 1000},
            {77, 1500, 1000},
            {78, 1550, 1000},
            {79, 1550, 1050},
    };

    /**
     * After the distance is lost (no tag, and no position), keep the last speeds this long before
     * going back to the fixed speeds.
     */
    public static long TAG_HOLD_MS = 1000;

    /**
     * The counter-roller spins against the flywheel across the piece, so it runs reversed. Read once
     * at construction. Check on {@code Bench: Motors} that the two wheels counter-rotate.
     */
    public static boolean ROLLER_REVERSED = true;

    /** Feeder power when on. Flip {@link #FEEDER_REVERSED} if it carries balls down. */
    public static double FEEDER_POWER = 0.6;
    public static boolean FEEDER_REVERSED = false;

    private static final int FLYWHEEL_COLUMN = 1;
    private static final int ROLLER_COLUMN = 2;

    private final DcMotorEx flywheel;
    private final DcMotorEx roller;
    private final DcMotorEx feeder;

    private boolean armed = false;
    private boolean feederOn = false;
    /** Feeder running backwards (Teleop's LT), to back a stuck ball out. Never true with feederOn. */
    private boolean feederOut = false;
    private double target = TARGET_TICKS_PER_SEC;
    private double rollerTarget = ROLLER_TARGET_TICKS_PER_SEC;
    private double lastFlywheelPower = Double.NaN;
    private double lastRollerPower = Double.NaN;
    private double lastFeederPower = Double.NaN;
    /** When {@link #setSpeedFromDistance} last got a real distance; -1 = never. */
    private long lastDistanceMs = -1;
    /** The last battery reading handed in; 0 = none yet, so no voltage scaling. */
    private double batteryVolts = 0;

    public Shooter(HardwareMap hardwareMap) {
        this(hardwareMap, HardwareNames.FLYWHEEL_MOTOR, HardwareNames.COUNTER_ROLLER_MOTOR,
                HardwareNames.FEEDER_MOTOR);
    }

    public Shooter(HardwareMap hardwareMap, String flywheelName, String rollerName, String feederName) {
        checkSpeedTable();
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
        // BRAKE: a stopped feeder holds the balls where they are instead of letting them roll into the wheels.
        feeder.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        // No setMode: the default run mode takes the power update() computes. The encoders are still
        // wired, so getVelocity() reads them either way.
        writeWheels(0, 0);
        writeFeeder(0);
    }

    /**
     * A short row would only fail the first time a tag is seen, mid-match, and rows out of order would
     * silently give the wrong speeds. Checked here instead, so it stops the OpMode at init and says
     * which row.
     */
    private static void checkSpeedTable() {
        double[][] table = SPEED_TABLE;
        if (table == null) return;
        for (int i = 0; i < table.length; i++) {
            if (table[i] == null || table[i].length < 3) {
                throw new IllegalStateException("Shooter.SPEED_TABLE row " + (i + 1)
                        + " needs 3 numbers: {inches, flywheel, roller}");
            }
            if (i > 0 && table[i][0] <= table[i - 1][0]) {
                throw new IllegalStateException("Shooter.SPEED_TABLE row " + (i + 1) + " (" + table[i][0]
                        + " in) must be farther than row " + i + " (" + table[i - 1][0] + " in): nearest first");
            }
        }
    }

    /** Begins holding the targets; the power lands on the next {@link #update()}. */
    public void arm() {
        armed = true;
    }

    /**
     * Stops both wheels and the feeder now rather than next loop, so a disarm is never one loop of
     * thrown piece. Also stops wheels left running by {@link #setRawPower}.
     */
    public void disarm() {
        feederOff();
        writeFeeder(0);
        armed = false;
        writeWheels(0, 0);   // writes only on change, so calling this every loop costs nothing
    }

    public boolean isArmed() {
        return armed;
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

    // ---- Feeder: on, off, or backwards ----

    /** Feeder on at {@link #FEEDER_POWER}; the power lands on the next {@link #update()}. */
    public void feederOn() {
        feederOn = true;
        feederOut = false;
    }

    public void feederOff() {
        feederOn = false;
        feederOut = false;
    }

    /** Feeder backwards at {@link #FEEDER_POWER}, to back a stuck ball out; lands on the next {@link #update()}. */
    public void feederOut() {
        feederOn = false;
        feederOut = true;
    }

    public boolean isFeederOut() {
        return feederOut;
    }

    public boolean isFeederOn() {
        return feederOn;
    }

    // ---- Targets ----

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

    /**
     * The flywheel speed for a tag distance, from {@link #SPEED_TABLE}: interpolated between rows, the
     * end row's speed past either end. An empty table (or NaN) gives {@link #TARGET_TICKS_PER_SEC}.
     */
    public static double speedForDistance(double inches) {
        return tableSpeed(inches, FLYWHEEL_COLUMN, TARGET_TICKS_PER_SEC);
    }

    /** The counter-roller's speed for a tag distance, the same way; fallback {@link #ROLLER_TARGET_TICKS_PER_SEC}. */
    public static double rollerSpeedForDistance(double inches) {
        return tableSpeed(inches, ROLLER_COLUMN, ROLLER_TARGET_TICKS_PER_SEC);
    }

    private static double tableSpeed(double inches, int column, double fallback) {
        double[][] table = SPEED_TABLE;
        if (table == null || table.length == 0 || Double.isNaN(inches)) return fallback;
        if (inches <= table[0][0]) return table[0][column];
        for (int i = 1; i < table.length; i++) {
            // Reached only when inches > the previous row, so near[0] < far[0] here: no divide by zero.
            if (inches <= table[i][0]) {
                double[] near = table[i - 1];
                double[] far = table[i];
                double fraction = (inches - near[0]) / (far[0] - near[0]);
                return near[column] + fraction * (far[column] - near[column]);
            }
        }
        return table[table.length - 1][column];
    }

    /**
     * Called every loop with the distance to the CELL in inches, or NaN when there is none. A distance
     * sets both targets from the table. NaN keeps the last speeds for {@link #TAG_HOLD_MS}, then falls
     * back to the fixed speeds.
     */
    public void setSpeedFromDistance(double inchesOrNaN) {
        if (!Double.isNaN(inchesOrNaN)) {
            target = speedForDistance(inchesOrNaN);
            rollerTarget = rollerSpeedForDistance(inchesOrNaN);
            lastDistanceMs = System.currentTimeMillis();
        } else if (!isSpeedFromTable()) {
            target = TARGET_TICKS_PER_SEC;
            rollerTarget = ROLLER_TARGET_TICKS_PER_SEC;
        }
    }

    /** True while the targets come from the table (a distance now, or within {@link #TAG_HOLD_MS}). */
    public boolean isSpeedFromTable() {
        return lastDistanceMs >= 0 && System.currentTimeMillis() - lastDistanceMs < TAG_HOLD_MS;
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

    /** True while the flywheel is far enough from its target to be on its big kP. */
    public boolean isBigLoop() {
        return isBig(target, getVelocity());
    }

    /** True while the counter-roller is far enough from its target to be on its big kP. */
    public boolean isRollerBigLoop() {
        return isBig(rollerTarget, getRollerVelocity());
    }

    private static boolean isBig(double target, double velocity) {
        return target > 0 && Math.abs(target - velocity) >= BIG_BAND_TICKS_PER_SEC;
    }

    /**
     * The battery voltage now, for {@link #TUNED_AT_VOLTS}. Whoever owns the shooter hands it in a
     * few times a second; 0 or never called = no scaling.
     */
    public void setBatteryVolts(double volts) {
        batteryVolts = volts;
    }

    /** What the kV and kS part of the power is multiplied by for the battery; 1 = unchanged. */
    public double getVoltsScale() {
        if (TUNED_AT_VOLTS <= 0 || batteryVolts <= 0) return 1;
        return Math.max(MIN_VOLTS_SCALE, Math.min(MAX_VOLTS_SCALE, TUNED_AT_VOLTS / batteryVolts));
    }

    /**
     * One wheel's power. A target of 0 is off (the Shooter Tuner uses this to spin one wheel alone).
     * Never below zero: a wheel above its target coasts down (FLOAT), it is not driven backwards.
     */
    private static double wheelPower(double target, double velocity, double kS, double kV,
                                     double smallKP, double bigKP, double voltsScale) {
        if (target <= 0) return 0;
        double error = target - velocity;
        double kP = isBig(target, velocity) ? bigKP : smallKP;
        return Math.max(0, (kV * target + kS) * voltsScale + kP * error);
    }

    /** Writes the motors. Called once per loop, at the bottom ({@code Robot.writeActuators()}). */
    public void update() {
        if (armed) {
            double voltsScale = getVoltsScale();
            writeWheels(wheelPower(target, getVelocity(), kS, kV, kP, BIG_kP, voltsScale),
                    wheelPower(rollerTarget, getRollerVelocity(), ROLLER_kS, ROLLER_kV,
                            ROLLER_kP, ROLLER_BIG_kP, voltsScale));
        } else {
            writeWheels(0, 0);
        }
        writeFeeder(feederOn ? FEEDER_POWER : feederOut ? -FEEDER_POWER : 0);
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

    /**
     * One telemetry line, e.g. {@code ARMED fly 1480/1500  roller 2090/2100 t/s READY  (table)  feeder ON}.
     */
    public String getStatusLine() {
        return String.format(Locale.US, "%s fly %.0f/%.0f  roller %.0f/%.0f t/s %s  (%s)%s",
                armed ? "ARMED" : "off", getVelocity(), target, getRollerVelocity(), rollerTarget,
                atTarget() ? "READY" : armed && (isBigLoop() || isRollerBigLoop()) ? "... big kP" : "...",
                isSpeedFromTable() ? "table" : "fixed",
                feederOn ? "  feeder ON" : feederOut ? "  feeder OUT" : "");
    }
}
