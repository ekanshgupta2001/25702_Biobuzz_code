package org.firstinspires.ftc.teamcode.pedro.procedures;

import com.pedropathing.tuning.autotune.Inputs;
import com.pedropathing.tuning.autotune.Procedure;
import com.pedropathing.tuning.autotune.TuningOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.subsystems.Shooter;

import java.math.BigDecimal;
import java.util.Locale;

/**
 * Shooter Tuner, on the Pedro tuning site (http://192.168.43.1:10158). Not a Quickstart procedure:
 * this one is ours.
 *
 * <p>Type a target speed and kS / kV / kP, the flywheel spins up from rest, and the next form shows
 * how long it took to reach the target, how far it overshot, how long it ran at full power, and the
 * average error once settled. Change a value and run again; tick Done for the line to paste into
 * {@code Shooter.java}. Each run waits for the wheel to stop first. Live numbers are on the Driver
 * Station while it runs.
 *
 * <p>What the gains change: the spin-up is mostly full power whatever they are, so kV and kS show up
 * in the settled error (kV sets the speed it holds on its own) and kP in the overshoot.
 *
 * <p>The values go straight into {@code Shooter}'s statics and stay until the Robot Controller app
 * restarts, so Teleop straight after a session uses them. They are only permanent once pasted.
 */
public class ShooterTuner extends Procedure {
    public ShooterTuner() {
        super("Shooter Tuner", "Spin the flywheel up to a target speed and time it, to tune kS, kV and kP.");
    }

    @Override
    public void run() throws InterruptedException {
        String lastRun = "No run yet.";
        while (true) {
            Inputs form = inputs("Shooter Tuner", lastRun + "  Change a value and run again, or tick Done.");
            Inputs.NumberField<Double> target = form.d("Target velocity (ticks/sec)")
                    .withDefault(Shooter.TARGET_TICKS_PER_SEC);
            Inputs.NumberField<Double> kS = form.d("kS").withDefault(Shooter.kS);
            Inputs.NumberField<Double> kV = form.d("kV").withDefault(Shooter.kV);
            Inputs.NumberField<Double> kP = form.d("kP").withDefault(Shooter.kP);
            Inputs.NumberField<Double> seconds = form.d("Run time (seconds)").withDefault(3.0).min(0.5).max(15.0);
            Inputs.Field<Boolean> done = form.b("Done: show the values to paste").withDefault(false);
            awaitInputs(form);

            Shooter.TARGET_TICKS_PER_SEC = target.get();
            Shooter.kS = kS.get();
            Shooter.kV = kV.get();
            Shooter.kP = kP.get();
            if (done.get()) break;

            lastRun = runOpMode(new SpinUp(Shooter.TARGET_TICKS_PER_SEC, seconds.get()));
        }

        result("Last run", lastRun);
        code(Language.JAVA, "public static double kS = " + plain(Shooter.kS)
                + ", kV = " + plain(Shooter.kV) + ", kP = " + plain(Shooter.kP) + ";");
    }

    /** 0.00039 rather than 3.9E-4. */
    private static String plain(double value) {
        return BigDecimal.valueOf(value).stripTrailingZeros().toPlainString();
    }
}

/** One spin-up from rest. Returns a one-line summary for the next form. */
class SpinUp extends TuningOpMode<String> {
    /** Slower than this counts as stopped. */
    private static final double AT_REST_TICKS_PER_SEC = 20;
    private static final double MAX_WAIT_FOR_REST_S = 5;
    /** The settled error is averaged over this last part of the run. */
    private static final double SETTLED_WINDOW_S = 0.5;

    private final double target;
    private final double seconds;

    SpinUp(double target, double seconds) {
        super("Spin up", String.format(Locale.US,
                "The flywheel spins up to %.0f ticks/sec for %.1f s, then stops. Keep hands clear.",
                target, seconds), true);
        this.target = target;
        this.seconds = seconds;
    }

    @Override
    protected String runTuningOpMode() {
        Shooter shooter = new Shooter(hardwareMap);
        shooter.setTarget(target);
        waitForStart();

        // A floating flywheel coasts for a while after the last run; time from rest, not from a roll.
        ElapsedTime clock = new ElapsedTime();
        while (opModeIsActive() && Math.abs(shooter.getVelocity()) > AT_REST_TICKS_PER_SEC
                && clock.seconds() < MAX_WAIT_FOR_REST_S) {
            telemetry.addData("waiting for the wheel to stop", "%.0f t/s", shooter.getVelocity());
            telemetry.update();
        }
        double startVelocity = shooter.getVelocity();

        clock.reset();
        shooter.arm();
        double reachedMs = -1;
        double peak = 0;
        double velocity = 0;
        double fullPowerMs = 0;
        double settledErrorSum = 0;
        int settledSamples = 0;
        double lastMs = 0;
        while (opModeIsActive() && clock.seconds() < seconds) {
            shooter.update();
            double nowMs = clock.milliseconds();
            velocity = shooter.getVelocity();
            peak = Math.max(peak, velocity);
            if (reachedMs < 0 && shooter.atTarget()) reachedMs = nowMs;
            // The same sum Shooter.update() writes; at 1 or more the motor is simply at full power.
            double asked = Shooter.kV * target + Shooter.kP * (target - velocity) + Shooter.kS;
            if (asked >= 1) fullPowerMs += nowMs - lastMs;
            if (clock.seconds() > seconds - SETTLED_WINDOW_S) {
                settledErrorSum += target - velocity;
                settledSamples++;
            }
            lastMs = nowMs;

            telemetry.addData("target", "%.0f t/s", target);
            telemetry.addData("velocity", "%.0f t/s", velocity);
            telemetry.addData("error", "%.0f t/s", target - velocity);
            telemetry.addData("reached target", reachedMs < 0 ? "not yet" : String.format(Locale.US, "%.0f ms", reachedMs));
            telemetry.update();
        }
        shooter.disarm();

        String start = Math.abs(startVelocity) > AT_REST_TICKS_PER_SEC
                ? String.format(Locale.US, " (started at %.0f t/s, not from rest)", startVelocity) : "";
        String settled = settledSamples == 0 ? "-"
                : String.format(Locale.US, "%+.0f", settledErrorSum / settledSamples);
        if (reachedMs < 0) {
            return String.format(Locale.US,
                    "Last run%s: never reached %.0f t/s (peak %.0f, full power %.0f ms). If it was at full "
                            + "power the target is above the wheel's top speed or a motor is reversed; "
                            + "otherwise raise kV.",
                    start, target, peak, fullPowerMs);
        }
        return String.format(Locale.US,
                "Last run%s: reached %.0f t/s in %.0f ms, peak %.0f (%+.0f over), full power %.0f ms, "
                        + "settled error %s t/s.",
                start, target, reachedMs, peak, peak - target, fullPowerMs, settled);
    }
}
