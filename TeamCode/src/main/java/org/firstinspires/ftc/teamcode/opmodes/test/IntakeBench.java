package org.firstinspires.ftc.teamcode.opmodes.test;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.subsystems.Intake;

/**
 * Runs the intake (roller and tunnel, one motor). Gamepad 1: RB = in, LB = out, both = stop.
 * It keeps doing the last thing until told otherwise.
 *
 * <p>The current only reads while running in, and the peak is held until the next stop. Use it to set
 * {@code Intake.STALL_CURRENT_AMPS}: the peak of a clean pick-up, then of a deliberate jam, and put the
 * threshold between them.
 */
@TeleOp(name = "Bench: Intake", group = "Bench")
public class IntakeBench extends OpMode {
    private Intake intake;
    /** Both bumpers were pressed: ignore a single bumper until both are let go. */
    private boolean waitForBumperRelease = false;
    private double peakAmps = 0;

    @Override
    public void init() {
        intake = new Intake(hardwareMap);
    }

    @Override
    public void loop() {
        boolean in = gamepad1.right_bumper;
        boolean out = gamepad1.left_bumper;
        if (in && out) {
            intake.stop();
            peakAmps = 0;
            waitForBumperRelease = true;
        } else if (!in && !out) {
            waitForBumperRelease = false;
        } else if (!waitForBumperRelease) {
            if (in) intake.in();
            else intake.out();
        }
        intake.update();

        telemetry.addLine("RB = in   LB = out   both = stop");
        telemetry.addData("Mode", intake.getMode());
        telemetry.addData("Power", "%.2f", intake.powerFor(intake.getMode()));
        telemetry.addData("Velocity", "%.0f ticks/sec", intake.getVelocity());
        double amps = intake.getCurrentAmps();
        if (!Double.isNaN(amps)) peakAmps = Math.max(peakAmps, amps);
        telemetry.addData("Current", "%s   peak %.2f A",
                Double.isNaN(amps) ? "- (only read while in)" : String.format("%.2f A", amps), peakAmps);
        if (intake.hasGivenUpUnjamming()) telemetry.addLine("Anti-jam gave up: run it out (LB)");
    }
}
