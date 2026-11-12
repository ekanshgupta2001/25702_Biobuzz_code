package org.firstinspires.ftc.teamcode.opmodes.test;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.subsystems.Intake;

/**
 * Runs the intake (roller and tunnel, one motor), with Teleop's controls. Gamepad 1: hold RT = in,
 * hold LT = out, neither or both = stop.
 *
 * <p>The current only reads while running in. The peak stays on the card after RT is let go and
 * starts over on the next RT press. Use it to set {@code Intake.STALL_CURRENT_AMPS}: the peak of a
 * clean pick-up, then of a deliberate jam, and put the threshold between them.
 */
@TeleOp(name = "Bench: Intake", group = "Bench")
public class IntakeBench extends OpMode {
    private Intake intake;
    private double peakAmps = 0;

    @Override
    public void init() {
        intake = new Intake(hardwareMap);
    }

    @Override
    public void loop() {
        boolean in = gamepad1.right_trigger > 0.5;
        boolean out = gamepad1.left_trigger > 0.5;
        if (in && !out) {
            if (intake.getMode() != Intake.Mode.IN) peakAmps = 0;   // a new pick-up: fresh peak
            intake.in();
        } else if (out && !in) {
            intake.out();
        } else {
            intake.stop();   // neither, or both. There is no default command on a bench to do it.
        }
        intake.update();

        telemetry.addLine("hold RT = in   hold LT = out");
        telemetry.addData("Mode", intake.getMode());
        telemetry.addData("Power", "%.2f", intake.powerFor(intake.getMode()));
        telemetry.addData("Velocity", "%.0f ticks/sec", intake.getVelocity());
        double amps = intake.getCurrentAmps();
        if (!Double.isNaN(amps)) peakAmps = Math.max(peakAmps, amps);
        telemetry.addData("Current", "%s   peak %.2f A (resets on the next RT)",
                Double.isNaN(amps) ? "- (only read while in)" : String.format("%.2f A", amps), peakAmps);
        if (intake.hasGivenUpUnjamming()) telemetry.addLine("Anti-jam gave up: run it out (LT)");
    }
}
