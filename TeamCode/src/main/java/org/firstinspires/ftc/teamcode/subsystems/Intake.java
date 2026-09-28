package org.firstinspires.ftc.teamcode.subsystems;

import com.pedropathing.ivy.Command;
import com.pedropathing.ivy.behaviors.BlockedBehavior;
import com.pedropathing.ivy.behaviors.InterruptedBehavior;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.util.control.JamDetector;
import org.firstinspires.ftc.teamcode.util.hardware.HardwareNames;

import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;

import java.util.function.Supplier;

/**
 * The front roller <b>and</b> the tunnel behind it. One goBILDA 5203-2402-0051 (50.9:1, 117 RPM)
 * drives both through the sprocket chain, so they are one subsystem with one mode: whatever the
 * roller is doing, the tunnel is doing.
 *
 * <p><b>Feeding the shooter is running this forward.</b> There is no gate and no sensor anywhere in
 * the path, so a piece reaches the flywheel because the tunnel ran long enough to carry it there.
 * That is why {@code commands/Shoot} owns this subsystem for the length of a shot and pulses it.
 *
 * <p><b>Power, not velocity.</b> A 50.9:1 roller has no use for a closed velocity loop; the SDK's
 * velocity PID would only add a tuning surface and a bus transaction. Commands set a {@link Mode};
 * {@link #update()} is the one place that writes hardware, and anti-jam can override it on the way
 * out.
 *
 * <p>Nothing here knows how many pieces are aboard, because nothing on the robot can know. The
 * operator is the only thing that decides when to stop intaking (BIOBUZZ G407, max 4 controlled).
 */
public class Intake {
    public static double IN = 1.0;
    public static double OUT = -1.0;
    /**
     * Enough to hold pieces against the tunnel without grinding them, for carrying a load between
     * scoring positions. Borrowed from the reference robot's {@code idle = 0.5}; measure it.
     */
    public static double IDLE = 0.35;

    /**
     * A 117 RPM 5203 stalls near 9 A and the roller pulling a piece in sits at 5-6 A for a moment,
     * so a 5 A threshold spits pieces mid-capture. Measure both with {@code Bench: Intake} — the
     * peak of a clean capture, then of a deliberate jam — and set this between them.
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

    public static int DEFAULT_IDLE_PRIORITY = -1;

    public enum Mode { OFF, IN, OUT, IDLE }

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
        motor.setDirection(DcMotorSimple.Direction.FORWARD);
        // BRAKE: the tunnel holds pieces on a slope, and a coasting tunnel lets them drift back.
        motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motor.setPower(0);
    }

    public void in()    { setMode(Mode.IN); }
    public void out()   { setMode(Mode.OUT); }
    public void idle()  { setMode(Mode.IDLE); }
    public void stop()  { setMode(Mode.OFF); }

    private void setMode(Mode next) {
        if (next != mode) {
            // Any deliberate change abandons an unjam in progress. Re-entering IN keeps the attempt
            // count, so mashing the button cannot bypass MAX_UNJAM_ATTEMPTS against a hard jam.
            if (next == Mode.IN) jamDetector.resetTiming();
            else jamDetector.reset();
        }
        mode = next;
    }

    public Mode getMode() {
        return mode;
    }

    /** The power {@link #update()} will write for the current mode, before anti-jam. */
    public double powerFor(Mode m) {
        switch (m) {
            case IN:   return IN;
            case OUT:  return OUT;
            case IDLE: return IDLE;
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

    public boolean isStallSuspected() {
        return jamDetector.isStallSuspected();
    }

    public boolean isUnjamming() {
        return jamDetector.isUnjamming(System.currentTimeMillis());
    }

    public int getUnjamAttempts() {
        return jamDetector.getAttempts();
    }

    /** True once anti-jam has exhausted {@link #MAX_UNJAM_ATTEMPTS}. The driver should reverse it by hand. */
    public boolean hasGivenUpUnjamming() {
        return jamDetector.hasGivenUp();
    }

    /** Writes hardware. Called once per loop from {@code Robot.writeActuators()}, after commands run. */
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

    // ---- Ivy commands ----

    /**
     * The operator's standing wish, as a default command: it re-reads {@code mode} every loop rather
     * than being re-scheduled on every press.
     *
     * <p>This matters because Ivy has <em>no duplicate guard</em> — scheduling the same intent every
     * loop would re-run {@code start()} every loop. At priority -1 with {@code SUSPEND} a shooting
     * cycle (which owns the intake to pulse the tunnel) preempts this and it resumes by itself
     * afterwards, still holding whatever the operator last asked for.
     */
    public Command operatorControlCommand(Supplier<Mode> mode) {
        return Command.build()
                .setExecute(() -> setMode(mode.get()))
                .setDone(() -> false)
                .setEnd(ec -> stop())
                .setPriority(DEFAULT_IDLE_PRIORITY)
                .setInterruptedBehavior(InterruptedBehavior.SUSPEND)
                .setBlockedBehavior(BlockedBehavior.QUEUE)
                .requiring(this);
    }

    public Command inCommand()   { return holdMode(this::in); }
    public Command outCommand()  { return holdMode(this::out); }
    public Command idleCommand() { return holdMode(this::idle); }

    /**
     * Schedule once at OpMode init. Suspends when a real command takes the resource and resumes when
     * that command ends. Logic in {@code setExecute} because the Scheduler's resume path does not
     * re-call {@code start()}; {@link BlockedBehavior#QUEUE} so it is not silently dropped if
     * something already holds the intake at init.
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

    private Command holdMode(Runnable start) {
        return Command.build()
                .setStart(start)
                .setDone(() -> false)
                .setEnd(ec -> stop())
                .requiring(this);
    }
}
