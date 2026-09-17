package org.firstinspires.ftc.teamcode.opmodes.test;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.game.PieceType;
import org.firstinspires.ftc.teamcode.subsystems.Intake;

/**
 * Feeds {@code Intake.MOTOR_FREE_SPEED_TICKS_PER_SEC} (the peak measured velocity with nothing in
 * the roller), {@code INTAKE_TICKS_PER_SEC} (a speed the loop can actually hold), and
 * {@code STALL_CURRENT_AMPS} / {@code STALL_TIMEOUT_MS} (peak amps of a clean capture versus the amps
 * of a deliberate jam: hold a piece against the roller and watch). Y toggles anti-jam so the jam can
 * be observed without the reversal getting in the way.
 */
@TeleOp(name = "Bench: Intake", group = "Bench")
public class IntakeBench extends BenchOpMode {
    private static final String[] CONTROLS = {
            "hold RB intake / LB outtake / B eject; release = stop",
            "hold A: run at the custom speed; dpad up/down: custom speed +/- 100 t/s",
            "Y: anti-jam on/off    X: reset peaks",
    };

    private double customTicksPerSec;
    private final Peak velocity = new Peak();
    private final Peak amps = new Peak();

    @Override
    protected String title() {
        return "BENCH: INTAKE  (Intake.java constants)";
    }

    @Override
    protected String[] controls() {
        return CONTROLS;
    }

    @Override
    protected void onBenchInit() {
        customTicksPerSec = Intake.INTAKE_TICKS_PER_SEC;
    }

    @Override
    protected void onBench() {
        customTicksPerSec = adjust(customTicksPerSec, gamepad1.dpadUpWasPressed(), gamepad1.dpadDownWasPressed(),
                100, 0, 6000);
        if (gamepad1.yWasPressed()) Intake.ANTI_JAM_ENABLED = !Intake.ANTI_JAM_ENABLED;
        if (gamepad1.xWasPressed()) {
            velocity.reset();
            amps.reset();
        }

        Intake intake = robot.intake;
        if (gamepad1.right_bumper) intake.intake();
        else if (gamepad1.left_bumper) intake.outtake();
        else if (gamepad1.b) intake.eject();
        else if (gamepad1.a) intake.setVelocity(customTicksPerSec);
        else intake.stop();

        double measured = intake.getVelocityTicksPerSec();
        double current = intake.getCurrentAmps();
        if (intake.getMode() != Intake.Mode.IDLE) {
            velocity.add(Math.abs(measured));
            amps.add(current);
        }

        telemetry.addData("Mode", intake.getMode());
        telemetry.addData("Velocity", "target %.0f  measured %.0f t/s", intake.getTargetVelocity(), measured);
        telemetry.addData("Peak velocity", velocity.status("%.0f") + "  (free speed -> MOTOR_FREE_SPEED_TICKS_PER_SEC)");
        telemetry.addData("Current", "%.2f A   peak %s  (stall threshold now %.1f A / %d ms)",
                current, amps.status("%.2f"), Intake.STALL_CURRENT_AMPS, Intake.STALL_TIMEOUT_MS);
        telemetry.addData("Anti-jam", "%s  stall suspected %s  unjamming %s  attempts %d%s",
                Intake.ANTI_JAM_ENABLED ? "ON" : "off", intake.isStallSuspected(), intake.isUnjamming(),
                intake.getUnjamAttempts(), intake.hasGivenUpUnjamming() ? "  GAVE UP" : "");
        telemetry.addData("Piece", "blocked by full %s  rejecting %s (%d)",
                intake.isBlockedByFullStorage(), intake.isRejecting(), intake.getRejections());
        telemetry.addData("Entrance sees", robot.storageEntranceSensor.isAvailable()
                ? String.valueOf(PieceType.classify(robot.storageEntranceSensor)) : "no sensor");
        telemetry.addData("Custom speed", "%.0f t/s (hold A)", customTicksPerSec);
    }
}
