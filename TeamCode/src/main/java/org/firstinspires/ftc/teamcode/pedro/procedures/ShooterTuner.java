package org.firstinspires.ftc.teamcode.pedro.procedures;

import com.pedropathing.tuning.autotune.Inputs;
import com.pedropathing.tuning.autotune.Procedure;
import com.pedropathing.tuning.autotune.TuningOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.commands.Shoot;
import org.firstinspires.ftc.teamcode.subsystems.Intake;
import org.firstinspires.ftc.teamcode.subsystems.Shooter;

import java.math.BigDecimal;
import java.util.Locale;

/**
 * Shooter Tuner, on the Pedro tuning site (http://192.168.43.1:10158). Not a Quickstart procedure:
 * this one is ours.
 *
 * <p>Type each wheel's target speed and kS / kV / kP and run. Then pick the test on gamepad 1:
 * <ul>
 *   <li><b>dpad left</b>: the flywheel spins up alone;</li>
 *   <li><b>dpad right</b>: the counter-roller spins up alone;</li>
 *   <li><b>dpad up</b>: both spin up, then <b>A</b> fires one ball (the intake and the up-wheels for
 *       one {@code Shoot.FEED_PULSE_MS}, exactly a match shot) and it measures how far each wheel
 *       dips and how long it takes to come back. <b>B</b> skips the shot.</li>
 * </ul>
 * One test per run. The next form shows, for each wheel, how long it took to reach its target, how
 * far it overshot, how long it ran at full power and the average error once settled. Change a value
 * and run again; tick Done for the lines to paste into {@code Shooter.java}. Each run waits for the
 * wheels to stop first. Live numbers are on the Driver Station while it runs.
 *
 * <p>What the gains change: the spin-up is mostly full power whatever they are, so kV and kS show up
 * in the settled error (kV sets the speed it holds on its own) and kP in the overshoot and in how
 * fast a wheel comes back after a shot.
 *
 * <p>The values go straight into {@code Shooter}'s statics and stay until the Robot Controller app
 * restarts, so Teleop straight after a session uses them. They are only permanent once pasted.
 */
public class ShooterTuner extends Procedure {
    public ShooterTuner() {
        super("Shooter Tuner", "Spin the flywheel, the counter-roller or both up to a target and time it, "
                + "to tune kS, kV and kP. With both, fire a ball and see how far each wheel dips.");
    }

    @Override
    public void run() throws InterruptedException {
        String lastRun = "No run yet.";
        while (true) {
            Inputs form = inputs("Shooter Tuner", lastRun + "\nChange a value and run again, or tick Done.");
            Inputs.NumberField<Double> target = form.d("Flywheel target (ticks/sec)")
                    .withDefault(Shooter.TARGET_TICKS_PER_SEC);
            Inputs.NumberField<Double> kS = form.d("Flywheel kS").withDefault(Shooter.kS);
            Inputs.NumberField<Double> kV = form.d("Flywheel kV").withDefault(Shooter.kV);
            Inputs.NumberField<Double> kP = form.d("Flywheel kP").withDefault(Shooter.kP);
            Inputs.NumberField<Double> rollerTarget = form.d("Counter-roller target (ticks/sec)")
                    .withDefault(Shooter.ROLLER_TARGET_TICKS_PER_SEC);
            Inputs.NumberField<Double> rollerKS = form.d("Counter-roller kS").withDefault(Shooter.ROLLER_kS);
            Inputs.NumberField<Double> rollerKV = form.d("Counter-roller kV").withDefault(Shooter.ROLLER_kV);
            Inputs.NumberField<Double> rollerKP = form.d("Counter-roller kP").withDefault(Shooter.ROLLER_kP);
            Inputs.NumberField<Double> seconds = form.d("Run time (seconds)").withDefault(3.0).min(0.5).max(15.0);
            Inputs.Field<Boolean> done = form.b("Done: show the values to paste").withDefault(false);
            awaitInputs(form);

            Shooter.TARGET_TICKS_PER_SEC = target.get();
            Shooter.kS = kS.get();
            Shooter.kV = kV.get();
            Shooter.kP = kP.get();
            Shooter.ROLLER_TARGET_TICKS_PER_SEC = rollerTarget.get();
            Shooter.ROLLER_kS = rollerKS.get();
            Shooter.ROLLER_kV = rollerKV.get();
            Shooter.ROLLER_kP = rollerKP.get();
            if (done.get()) break;

            lastRun = runOpMode(new ShooterTest(
                    Shooter.TARGET_TICKS_PER_SEC, Shooter.ROLLER_TARGET_TICKS_PER_SEC, seconds.get()));
        }

        result("Last run", lastRun);
        code(Language.JAVA, "public static double kS = " + plain(Shooter.kS)
                + ", kV = " + plain(Shooter.kV) + ", kP = " + plain(Shooter.kP) + ";\n"
                + "public static double ROLLER_kS = " + plain(Shooter.ROLLER_kS)
                + ", ROLLER_kV = " + plain(Shooter.ROLLER_kV) + ", ROLLER_kP = " + plain(Shooter.ROLLER_kP) + ";");
    }

    /** 0.00039 rather than 3.9E-4. */
    private static String plain(double value) {
        return BigDecimal.valueOf(value).stripTrailingZeros().toPlainString();
    }
}

/** One test, picked on gamepad 1's dpad. Returns a summary for the next form. */
class ShooterTest extends TuningOpMode<String> {
    /** Slower than this counts as stopped. */
    static final double AT_REST_TICKS_PER_SEC = 20;
    private static final double MAX_WAIT_FOR_REST_S = 5;
    /** The settled error is averaged over this last part of the spin-up. */
    private static final double SETTLED_WINDOW_S = 0.5;
    /** How long the wheels hold speed waiting for A (fire) or B (skip). */
    private static final double MAX_WAIT_TO_FIRE_S = 10;
    /** How long to keep watching the wheels after the feed pulse ends. */
    private static final double WATCH_AFTER_PULSE_MS = 1000;

    private final double flywheelTarget;
    private final double rollerTarget;
    private final double seconds;

    ShooterTest(double flywheelTarget, double rollerTarget, double seconds) {
        super("Shooter test", String.format(Locale.US,
                "On gamepad 1: dpad left = flywheel alone (%.0f t/s), dpad right = counter-roller alone "
                        + "(%.0f t/s), dpad up = both, then A fires one ball. Spins for %.1f s. Keep hands clear.",
                flywheelTarget, rollerTarget, seconds), true);
        this.flywheelTarget = flywheelTarget;
        this.rollerTarget = rollerTarget;
        this.seconds = seconds;
    }

    @Override
    protected String runTuningOpMode() {
        Shooter shooter = new Shooter(hardwareMap);
        Intake intake = new Intake(hardwareMap);
        waitForStart();

        // Pick the test.
        boolean testFlywheel = false;
        boolean testRoller = false;
        while (opModeIsActive() && !testFlywheel && !testRoller) {
            if (gamepad1.dpad_left) {
                testFlywheel = true;
            } else if (gamepad1.dpad_right) {
                testRoller = true;
            } else if (gamepad1.dpad_up) {
                testFlywheel = true;
                testRoller = true;
            }
            telemetry.addLine("Pick a test on gamepad 1:");
            telemetry.addLine("  dpad left  = flywheel alone");
            telemetry.addLine("  dpad right = counter-roller alone");
            telemetry.addLine("  dpad up    = both, then A fires one ball");
            telemetry.update();
        }
        if (!testFlywheel && !testRoller) return "No test run: no dpad was pressed.";
        boolean both = testFlywheel && testRoller;

        // The wheel not under test gets a target of 0, which Shooter treats as off.
        WheelStats flywheel = new WheelStats("Flywheel", testFlywheel ? flywheelTarget : 0,
                Shooter.kS, Shooter.kV, Shooter.kP);
        WheelStats roller = new WheelStats("Counter-roller", testRoller ? rollerTarget : 0,
                Shooter.ROLLER_kS, Shooter.ROLLER_kV, Shooter.ROLLER_kP);
        shooter.setTarget(flywheel.target);
        shooter.setRollerTarget(roller.target);

        // A floating wheel coasts for a while after the last run; time from rest, not from a roll.
        ElapsedTime clock = new ElapsedTime();
        while (opModeIsActive() && clock.seconds() < MAX_WAIT_FOR_REST_S
                && ((testFlywheel && Math.abs(shooter.getVelocity()) > AT_REST_TICKS_PER_SEC)
                || (testRoller && Math.abs(shooter.getRollerVelocity()) > AT_REST_TICKS_PER_SEC))) {
            telemetry.addData("waiting for the wheels to stop", "flywheel %.0f, counter-roller %.0f t/s",
                    shooter.getVelocity(), shooter.getRollerVelocity());
            telemetry.update();
        }
        flywheel.startVelocity = shooter.getVelocity();
        roller.startVelocity = shooter.getRollerVelocity();

        // Spin up for the run time.
        clock.reset();
        shooter.arm();
        double bothReadyMs = -1;
        double lastMs = 0;
        while (opModeIsActive() && clock.seconds() < seconds) {
            shooter.update();
            double nowMs = clock.milliseconds();
            boolean settling = clock.seconds() > seconds - SETTLED_WINDOW_S;
            flywheel.spinUpSample(nowMs, nowMs - lastMs, shooter.getVelocity(), settling);
            roller.spinUpSample(nowMs, nowMs - lastMs, shooter.getRollerVelocity(), settling);
            if (both && bothReadyMs < 0 && shooter.atTarget()) bothReadyMs = nowMs;
            lastMs = nowMs;

            if (testFlywheel) flywheel.addTelemetry(telemetry);
            if (testRoller) roller.addTelemetry(telemetry);
            telemetry.update();
        }

        String summary;
        if (both) {
            String ready = bothReadyMs < 0 ? "Both never at target together."
                    : String.format(Locale.US, "Both at target at %.0f ms.", bothReadyMs);
            summary = flywheel.spinUpSummary() + "\n" + roller.spinUpSummary() + "\n" + ready
                    + "\n" + fire(shooter, intake, flywheel, roller);
        } else {
            summary = testFlywheel ? flywheel.spinUpSummary() : roller.spinUpSummary();
        }
        shooter.disarm();
        intake.stop();
        intake.update();
        return summary;
    }

    /**
     * Holds both wheels at speed until A or B, then on A runs one feed pulse (intake and up-wheels,
     * like {@code commands/Shoot}) and watches each wheel dip and come back.
     */
    private String fire(Shooter shooter, Intake intake, WheelStats flywheel, WheelStats roller) {
        ElapsedTime clock = new ElapsedTime();
        boolean fire = false;
        while (opModeIsActive() && clock.seconds() < MAX_WAIT_TO_FIRE_S) {
            if (gamepad1.a) {
                fire = true;
                break;
            }
            if (gamepad1.b) break;
            shooter.update();
            telemetry.addLine("A = fire one ball, B = skip");
            telemetry.addData("both", shooter.atTarget() ? "READY" : "not at target");
            flywheel.addTelemetry(telemetry);
            roller.addTelemetry(telemetry);
            telemetry.update();
        }
        if (!fire) return "Shot: skipped.";

        double firedFlywheel = shooter.getVelocity();
        double firedRoller = shooter.getRollerVelocity();
        clock.reset();
        intake.in();
        shooter.feederOn();
        while (opModeIsActive() && clock.milliseconds() < Shoot.FEED_PULSE_MS + WATCH_AFTER_PULSE_MS) {
            if (shooter.isFeederOn() && clock.milliseconds() >= Shoot.FEED_PULSE_MS) {
                intake.stop();
                shooter.feederOff();
            }
            intake.update();
            shooter.update();
            double nowMs = clock.milliseconds();
            flywheel.shotSample(nowMs, shooter.getVelocity());
            roller.shotSample(nowMs, shooter.getRollerVelocity());

            telemetry.addData("feed pulse", shooter.isFeederOn() ? "ON" : "done");
            flywheel.addTelemetry(telemetry);
            roller.addTelemetry(telemetry);
            telemetry.update();
        }
        intake.stop();
        shooter.feederOff();
        intake.update();
        shooter.update();

        return String.format(Locale.US,
                "Shot fired at flywheel %.0f, counter-roller %.0f t/s (%d ms pulse):\n%s\n%s\n"
                        + "A match feeds the next ball %d ms after the pulse whether or not the wheels are back.",
                firedFlywheel, firedRoller, Shoot.FEED_PULSE_MS, flywheel.shotSummary(), roller.shotSummary(),
                Shoot.RECOVER_TIMEOUT_MS);
    }
}

/** One wheel's numbers over a test. */
class WheelStats {
    final String name;
    final double target;
    private final double kS, kV, kP;

    double startVelocity;

    // The spin-up.
    private double reachedMs = -1;
    private double peak = 0;
    private double fullPowerMs = 0;
    private double settledErrorSum = 0;
    private int settledSamples = 0;
    private double velocity = 0;

    // After a shot.
    private double lowest = Double.MAX_VALUE;
    private double leftBandMs = -1;
    private double backInBandMs = -1;

    WheelStats(String name, double target, double kS, double kV, double kP) {
        this.name = name;
        this.target = target;
        this.kS = kS;
        this.kV = kV;
        this.kP = kP;
    }

    private boolean inBand(double v) {
        return Math.abs(target - v) < Shooter.TOLERANCE_TICKS_PER_SEC;
    }

    void spinUpSample(double nowMs, double sinceLastMs, double v, boolean settling) {
        velocity = v;
        peak = Math.max(peak, v);
        if (reachedMs < 0 && inBand(v)) reachedMs = nowMs;
        // The same sum Shooter.update() writes; at 1 or more the motor is simply at full power.
        if (kV * target + kP * (target - v) + kS >= 1) fullPowerMs += sinceLastMs;
        if (settling) {
            settledErrorSum += target - v;
            settledSamples++;
        }
    }

    void shotSample(double msSincePulse, double v) {
        velocity = v;
        lowest = Math.min(lowest, v);
        if (leftBandMs < 0) {
            if (!inBand(v)) leftBandMs = msSincePulse;
        } else if (backInBandMs < 0 && inBand(v)) {
            backInBandMs = msSincePulse;
        }
    }

    void addTelemetry(Telemetry telemetry) {
        telemetry.addData(name, "%.0f / %.0f t/s (error %.0f), reached %s", velocity, target, target - velocity,
                reachedMs < 0 ? "not yet" : String.format(Locale.US, "%.0f ms", reachedMs));
    }

    String spinUpSummary() {
        String start = Math.abs(startVelocity) > ShooterTest.AT_REST_TICKS_PER_SEC
                ? String.format(Locale.US, " (started at %.0f t/s, not from rest)", startVelocity) : "";
        if (reachedMs < 0) {
            return String.format(Locale.US,
                    "%s%s: never reached %.0f t/s (peak %.0f, full power %.0f ms). If it was at full power the "
                            + "target is above the wheel's top speed or the motor is reversed; otherwise raise kV.",
                    name, start, target, peak, fullPowerMs);
        }
        String settled = settledSamples == 0 ? "-"
                : String.format(Locale.US, "%+.0f", settledErrorSum / settledSamples);
        return String.format(Locale.US,
                "%s%s: reached %.0f t/s in %.0f ms, peak %.0f (%+.0f over), full power %.0f ms, settled error %s t/s.",
                name, start, target, reachedMs, peak, peak - target, fullPowerMs, settled);
    }

    String shotSummary() {
        if (leftBandMs < 0) {
            return String.format(Locale.US, "%s: no dip past tolerance (lowest %.0f t/s).", name, lowest);
        }
        String back = backInBandMs < 0 ? "not back within the watch window"
                : String.format(Locale.US, "back in %.0f ms", backInBandMs - leftBandMs);
        return String.format(Locale.US, "%s: dipped %.0f t/s (to %.0f), %s.", name, target - lowest, lowest, back);
    }
}
