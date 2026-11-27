package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.util.control.JamDetector;
import org.firstinspires.ftc.teamcode.util.hardware.HardwareNames;

import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;

/**
 * The front roller <b>and</b> the tunnel behind it. One goBILDA 5203-2402-0051 (50.9:1, 117 RPM)
 * drives both through the sprocket chain, so they are one subsystem with one mode: whatever the
 * roller is doing, the tunnel is doing.
 *
 * <p><b>Feeding the shooter is running this forward</b> ({@link #feed()}), together with the shooter's
 * feeder. There is no gate and no sensor anywhere in the path, so a piece reaches the wheels because
 * the tunnel and the feeder ran long enough to carry it there.
 *
 * <p><b>Power, not velocity.</b> The OpMode sets a {@link Mode}; {@link #update()} is the one place
 * that writes the motor, and anti-jam can override it on the way out.
 *
 * <p>Nothing here knows how many pieces are aboard, because nothing on the robot can know. The
 * operator is the only thing that decides when to stop intaking (BIOBUZZ G407, max 4 controlled).
 */
public class Intake {
    /**
     * The motor direction that makes {@link #IN} pull pieces in. {@code Bench: Motors} reads this too.
     * If the intake ever runs backwards again, this is the one line to flip.
     */
    public static final DcMotorSimple.Direction DIRECTION = DcMotorSimple.Direction.REVERSE;

    public static double IN = 1.0;
    public static double OUT = -1.0;

    /**
     * A 117 RPM 5203 stalls near 9 A and the roller pulling a piece in sits at 5-6 A for a moment,
     * so a 5 A threshold spits pieces mid-capture. Read the current on {@code Bench: Intake} — a
     * clean capture, then a deliberate jam — and set this between them.
     */
    public static double STALL_CURRENT_AMPS = 7.0;
    public static long STALL_TIMEOUT_MS = 300;
    public static double UNJAM_POWER = -1.0;
    public static long UNJAM_DURATION_MS = 150;
    public static boolean ANTI_JAM_ENABLED = true;
    /** Consecutive attempts before giving up, so a hard jam cannot cook the motor all match. */
    public static int MAX_UNJAM_ATTEMPTS = 3;
    /**
     * Current must stay under the threshold this long before the attempt count is forgiven. The dip
     * while the motor re-accelerates after a reversal is shorter than this, so a real jam does stop
     * after {@link #MAX_UNJAM_ATTEMPTS}.
     */
    public static long HEALTHY_RESET_MS = 500;

    public enum Mode { OFF, IN, OUT, FEED }

    private final DcMotorEx motor;
    private Mode mode = Mode.OFF;
    /** Sampled once per {@link #update()} while pulling in; NaN otherwise. A current read is its own bus transaction. */
    private double currentAmps = Double.NaN;
    private double lastWritten = Double.NaN;

    private final JamDetector jamDetector = new JamDetector();

    public Intake(HardwareMap hardwareMap) {
        this(hardwareMap, HardwareNames.INTAKE_MOTOR);
    }

    public Intake(HardwareMap hardwareMap, String name) {
        motor = hardwareMap.get(DcMotorEx.class, name);
        motor.setDirection(DIRECTION);
        // BRAKE: the tunnel holds pieces on a slope, and a coasting tunnel lets them drift back.
        motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motor.setPower(0);
    }

    public void in()    { setMode(Mode.IN); }
    public void out()   { setMode(Mode.OUT); }
    /** Runs in like {@link #in()} but with anti-jam off: for feed pulses, when the shooter's feeder runs too. */
    public void feed()  { setMode(Mode.FEED); }
    public void stop()  { setMode(Mode.OFF); }

    private void setMode(Mode next) {
        if (next != mode) {
            // Any deliberate change abandons an unjam in progress. Only running it OUT clears the
            // attempt count, so letting go of the trigger and pulling it again cannot bypass
            // MAX_UNJAM_ATTEMPTS against a hard jam, and the JAMMED warning stays up until then.
            if (next == Mode.OUT) jamDetector.reset();
            else jamDetector.resetTiming();
        }
        mode = next;
    }

    public Mode getMode() {
        return mode;
    }

    /** The power {@link #update()} will write for the current mode, before anti-jam. */
    public double powerFor(Mode m) {
        switch (m) {
            case IN:
            case FEED: return IN;
            case OUT:  return OUT;
            default:   return 0;
        }
    }

    /** This loop's current sample; NaN on a loop the roller was not pulling in, when nothing sampled it. */
    public double getCurrentAmps() {
        return currentAmps;
    }

    public double getVelocity() {
        return motor.getVelocity();
    }

    /** True once anti-jam has exhausted {@link #MAX_UNJAM_ATTEMPTS}. The driver should reverse it by hand. */
    public boolean hasGivenUpUnjamming() {
        return jamDetector.hasGivenUp();
    }

    /** Writes the motor. Called once per loop, at the bottom ({@code Robot.writeActuators()}). */
    public void update() {
        long now = System.currentTimeMillis();
        boolean pulling = mode == Mode.IN;

        // Motor current is not in the hub's bulk read: one ADC transaction per sample, and it only
        // means anything while the roller is loaded. The jam detector and the telemetry share this
        // one sample.
        currentAmps = pulling ? motor.getCurrent(CurrentUnit.AMPS) : Double.NaN;
        if (pulling) {
            // Pushed every pulling loop so edits to the statics above take effect live on a bench.
            jamDetector.configure(STALL_CURRENT_AMPS, STALL_TIMEOUT_MS, UNJAM_DURATION_MS,
                    MAX_UNJAM_ATTEMPTS, HEALTHY_RESET_MS);
        }
        if (jamDetector.update(now, ANTI_JAM_ENABLED && pulling, pulling ? currentAmps : 0)) {
            write(UNJAM_POWER);
            return;
        }
        write(powerFor(mode));
    }

    /** Writes only on change: an unchanged setPower is still a bus transaction. */
    private void write(double power) {
        if (power != lastWritten) {
            motor.setPower(power);
            lastWritten = power;
        }
    }
}
