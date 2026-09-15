package org.firstinspires.ftc.teamcode.opmodes.test;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.commands.Macros;
import org.firstinspires.ftc.teamcode.subsystems.Shooter;

/**
 * The flywheel. Feeds {@code Shooter.TICKS_PER_REV} (compare the measured ticks per second with the
 * RPM a tachometer or the motor's spec says), {@code SHOOT_RPM} (the speed that lands a shot in the
 * up-CELL from the wall), {@code AT_SPEED_TOLERANCE_RPM} (how much the reading wanders once
 * settled), the spin-up time, and {@code SHOT_RECOVERY_MIN_MS} / {@code SHOT_RECOVERY_TIMEOUT_MS}
 * (Y fires one piece through with the real storage+transfer pulse and records how far the speed
 * dipped and how long it took to come back). X toggles the custom velocity PIDF live.
 */
@TeleOp(name = "Bench: Shooter", group = "Bench")
public class ShooterBench extends BenchOpMode {
    private static final String[] CONTROLS = {
            "A: flywheel on/off      dpad up/down: target +/- 100 rpm",
            "Y: fire one (storage + transfer pulse) while spinning; records the dip and recovery",
            "X: custom PIDF on/off (re-applied at once)      B: reset peaks",
    };

    private boolean spinning = false;
    private long spinStartedMs = -1;
    private long spinUpMs = -1;
    private long pulseUntilMs = -1;
    private long shotAtMs = -1;
    private double dipRpm = Double.NaN;
    private long recoveryMs = -1;
    private final Peak rpmWhileReady = new Peak();
    private final Peak amps = new Peak();

    @Override
    protected String title() {
        return "BENCH: SHOOTER  (Shooter.java constants)";
    }

    @Override
    protected String[] controls() {
        return CONTROLS;
    }

    @Override
    protected void onBench() {
        Shooter shooter = robot.shooter;
        Shooter.SHOOT_RPM = adjust(Shooter.SHOOT_RPM, gamepad1.dpadUpWasPressed(), gamepad1.dpadDownWasPressed(),
                100, 0, 6500);
        if (gamepad1.aWasPressed()) {
            spinning = !spinning;
            spinStartedMs = spinning ? nowMs : -1;
            spinUpMs = -1;
        }
        if (gamepad1.xWasPressed()) {
            Shooter.CUSTOM_PIDF = !Shooter.CUSTOM_PIDF;
            shooter.applyPidf();
        }
        if (gamepad1.bWasPressed()) {
            rpmWhileReady.reset();
            amps.reset();
            dipRpm = Double.NaN;
            recoveryMs = -1;
        }
        if (gamepad1.yWasPressed() && spinning && shooter.atSpeed()) {
            pulseUntilMs = nowMs + Macros.SENSORLESS_FEED_PULSE_MS;
            shotAtMs = nowMs;
            dipRpm = shooter.getRpm();
            recoveryMs = -1;
        }

        if (spinning) shooter.setTargetRpm(Shooter.SHOOT_RPM);
        else shooter.idle();

        boolean pulsing = pulseUntilMs >= 0 && nowMs < pulseUntilMs;
        if (!pulsing) pulseUntilMs = -1;
        if (pulsing) {
            robot.storage.advance();
            robot.transfer.feed();
        } else {
            robot.storage.stop();
            robot.transfer.stop();
        }

        double rpm = shooter.getRpm();
        if (spinning && spinUpMs < 0 && shooter.atSpeed()) spinUpMs = nowMs - spinStartedMs;
        if (shotAtMs >= 0) {
            if (rpm < dipRpm) dipRpm = rpm;
            if (recoveryMs < 0 && nowMs - shotAtMs > Shooter.SHOT_RECOVERY_MIN_MS && shooter.atSpeed()) {
                recoveryMs = nowMs - shotAtMs;
            }
        }
        if (spinning && shooter.atSpeed() && !pulsing && shotAtMs < 0) rpmWhileReady.add(rpm);
        if (spinning) amps.add(shooter.getCurrentAmps());

        telemetry.addData("Flywheel", "%s  target %.0f rpm (dpad)  measured %.0f rpm = %.0f t/s",
                spinning ? "ON" : "off", Shooter.SHOOT_RPM, rpm, Shooter.rpmToTicksPerSec(rpm));
        telemetry.addData("Ticks/rev", "%.0f assumed: check measured t/s x 60 / true rpm", Shooter.TICKS_PER_REV);
        telemetry.addData("Ready", "in band now %s   atSpeed (latched %d loops) %s   tolerance +/-%.0f rpm",
                shooter.inBandNow(), Shooter.AT_SPEED_LOOPS, shooter.atSpeed(), Shooter.AT_SPEED_TOLERANCE_RPM);
        telemetry.addData("Spin-up", spinUpMs >= 0 ? fmt("%d ms", spinUpMs) : (spinning ? "..." : "n/a"));
        telemetry.addData("Settled rpm", rpmWhileReady.status("%.0f") + "  (spread -> AT_SPEED_TOLERANCE_RPM)");
        telemetry.addData("Last shot", "dip to %s rpm   recovery %s   (-> SHOT_RECOVERY_MIN_MS / TIMEOUT_MS)",
                num(dipRpm, "%.0f"), recoveryMs >= 0 ? fmt("%d ms", recoveryMs) : (shotAtMs >= 0 ? "..." : "n/a"));
        telemetry.addData("Current", "%.2f A  peak %s", shooter.getCurrentAmps(), amps.status("%.2f"));
        telemetry.addData("PIDF", "%s  P %.3f I %.4f D %.1f F %.2f  (X toggles)",
                Shooter.CUSTOM_PIDF ? "CUSTOM" : "SDK default", Shooter.PIDF_P, Shooter.PIDF_I, Shooter.PIDF_D, Shooter.PIDF_F);
    }

    /** Milliseconds from the last Y shot until the wheel read at speed again, or -1 if not yet. */
    public long getLastRecoveryMs() {
        return recoveryMs;
    }

    /** Lowest rpm seen since the last Y shot, or NaN before the first. */
    public double getLastDipRpm() {
        return dipRpm;
    }
}
