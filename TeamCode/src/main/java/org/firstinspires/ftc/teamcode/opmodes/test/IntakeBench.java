package org.firstinspires.ftc.teamcode.opmodes.test;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.opmodes.MatchOpMode;
import org.firstinspires.ftc.teamcode.subsystems.Intake;
import org.firstinspires.ftc.teamcode.util.field.Alliance;
import org.firstinspires.ftc.teamcode.util.time.MatchClock;

/**
 * Finds {@link Intake#STALL_CURRENT_AMPS} and {@link Intake#STALL_TIMEOUT_MS}, and confirms the CAD
 * claim that one motor drives both the roller and the tunnel.
 *
 * <h2>Two separate runs, and the threshold goes between them</h2>
 * A clean capture and a real jam both draw a lot of current; the whole difficulty is that they
 * overlap. So: run one clean capture and read the peak, then switch anti-jam off, deliberately stall
 * the roller, and read that peak. {@code STALL_CURRENT_AMPS} belongs between the two, and
 * {@code STALL_TIMEOUT_MS} must outlast the longest over-threshold run a clean capture produces.
 *
 * <p>Both are <b>peak-held</b>, because a capture's current spike passes faster than anyone can read
 * a number off the Driver Station.
 */
@TeleOp(name = "Bench: Intake", group = "Bench")
public class IntakeBench extends MatchOpMode {
    private double peakAmps = 0;
    private double peakVelocity = 0;
    private double peakIdleVelocity = 0;
    /** Longest unbroken run of over-threshold current, milliseconds. Feeds STALL_TIMEOUT_MS. */
    private long longestOverRunMs = 0;
    private long overSinceMs = 0;

    @Override
    protected Alliance alliance() {
        return Alliance.BLUE;
    }

    @Override
    protected MatchClock.Period matchPeriod() {
        return MatchClock.Period.TELEOP;
    }

    @Override
    protected boolean usesScheduler() {
        return false;   // a bench drives the subsystem directly; nothing else re-asserts idle
    }

    @Override
    protected void onDecide() {
        // Latched, not held: freeing a jam by hand takes both of them.
        if (gamepad1.aWasPressed()) robot.intake.in();
        if (gamepad1.bWasPressed()) robot.intake.out();
        if (gamepad1.xWasPressed()) robot.intake.idle();
        if (gamepad1.yWasPressed()) robot.intake.stop();

        if (gamepad1.leftBumperWasPressed()) Intake.ANTI_JAM_ENABLED = !Intake.ANTI_JAM_ENABLED;
        if (gamepad1.rightBumperWasPressed()) clearPeaks();

        if (gamepad1.dpadUpWasPressed()) Intake.STALL_CURRENT_AMPS += 0.5;
        if (gamepad1.dpadDownWasPressed()) Intake.STALL_CURRENT_AMPS -= 0.5;
        if (gamepad1.dpadRightWasPressed()) Intake.STALL_TIMEOUT_MS += 50;
        if (gamepad1.dpadLeftWasPressed()) Intake.STALL_TIMEOUT_MS -= 50;
    }

    private void clearPeaks() {
        peakAmps = 0;
        peakVelocity = 0;
        peakIdleVelocity = 0;
        longestOverRunMs = 0;
        overSinceMs = 0;
    }

    @Override
    protected void onAfterAct() {
        double amps = robot.intake.getCurrentAmps();
        double velocity = Math.abs(robot.intake.getVelocity());

        // NaN is "not sampled": the current read only happens while the roller pulls in.
        if (!Double.isNaN(amps)) {
            if (amps > peakAmps) peakAmps = amps;

            long now = System.currentTimeMillis();
            if (amps >= Intake.STALL_CURRENT_AMPS) {
                if (overSinceMs == 0) overSinceMs = now;
                longestOverRunMs = Math.max(longestOverRunMs, now - overSinceMs);
            } else {
                overSinceMs = 0;
            }
        } else {
            overSinceMs = 0;
        }

        if (robot.intake.getMode() == Intake.Mode.IN && velocity > peakVelocity) peakVelocity = velocity;
        if (robot.intake.getMode() == Intake.Mode.IDLE && velocity > peakIdleVelocity) {
            peakIdleVelocity = velocity;
        }
    }

    @Override
    protected void onTelemetry() {
        telemetry.addData("Loop", "%.0f Hz", robot.getLoopHz());
        telemetry.addLine("A=in  B=out  X=idle  Y=off   LB=anti-jam   RB=clear peaks");
        telemetry.addLine("dpad up/down = STALL_CURRENT_AMPS    left/right = STALL_TIMEOUT_MS");
        telemetry.addLine();

        telemetry.addData("Mode", "%s  (power %.2f)", robot.intake.getMode(),
                robot.intake.powerFor(robot.intake.getMode()));
        telemetry.addData("Velocity", "%.0f t/s   peak in %.0f   peak idle %.0f",
                robot.intake.getVelocity(), peakVelocity, peakIdleVelocity);

        double amps = robot.intake.getCurrentAmps();
        telemetry.addData("Current", "%s A   PEAK %.2f A",
                Double.isNaN(amps) ? "not sampled" : String.format("%.2f", amps), peakAmps);
        telemetry.addData("Over threshold", "longest unbroken run %d ms", longestOverRunMs);
        telemetry.addLine();

        telemetry.addData("Anti-jam", "%s   suspect=%s unjamming=%s attempts=%d givenUp=%s",
                Intake.ANTI_JAM_ENABLED ? "ON" : "OFF", robot.intake.isStallSuspected(),
                robot.intake.isUnjamming(), robot.intake.getUnjamAttempts(),
                robot.intake.hasGivenUpUnjamming());
        telemetry.addLine();

        telemetry.addData("PASTE Intake.STALL_CURRENT_AMPS", "%.2f  (currently %.2f)",
                peakAmps, Intake.STALL_CURRENT_AMPS);
        telemetry.addData("PASTE Intake.STALL_TIMEOUT_MS", "> %d  (currently %d)",
                longestOverRunMs, Intake.STALL_TIMEOUT_MS);
        telemetry.addLine();
        telemetry.addLine("1. clean capture, anti-jam ON  -> note PEAK");
        telemetry.addLine("2. RB, anti-jam OFF, stall it  -> note PEAK; threshold goes between");
        telemetry.addLine("CHECK: the roller AND the tunnel must both turn. One motor drives both.");
    }

    @Override
    protected void onStop() {
        robot.stopMechanisms();
    }
}
