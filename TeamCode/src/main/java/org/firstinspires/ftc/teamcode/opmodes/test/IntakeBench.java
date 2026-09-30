package org.firstinspires.ftc.teamcode.opmodes.test;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.opmodes.MatchOpMode;
import org.firstinspires.ftc.teamcode.subsystems.Intake;
import org.firstinspires.ftc.teamcode.util.field.Alliance;
import org.firstinspires.ftc.teamcode.util.time.MatchClock;

import java.util.EnumSet;
import java.util.Set;

/**
 * Proves the {@code Intake} subsystem works: every mode turns the motor the right way, the current
 * is sampled while pulling, and the jam detector reacts. One motor drives the roller <em>and</em> the
 * tunnel, so the check a human makes by eye is that both move.
 *
 * <p>Each mode earns a PASS on the card once it has been seen turning in the direction of its power,
 * faster than {@link #MIN_TICKS_PER_SEC}. Direction is judged against the commanded sign only: the
 * SDK applies {@code Direction} to power and velocity together, so a motor wired backwards still
 * passes here. Whether the roller pulls <em>in</em> is for the eye.
 *
 * <p>The peak current and the longest over-threshold run are held so HANDOFF §8 step 5 still works:
 * one clean capture, then a deliberate stall with anti-jam off. {@link Intake#STALL_CURRENT_AMPS} goes
 * between the two peaks, and {@link Intake#STALL_TIMEOUT_MS} must outlast the clean capture's run.
 */
@TeleOp(name = "Bench: Intake", group = "Bench")
public class IntakeBench extends MatchOpMode {
    /** Slower than this does not count as the mode working. */
    public static double MIN_TICKS_PER_SEC = 50;

    private final Set<Intake.Mode> passed = EnumSet.noneOf(Intake.Mode.class);
    private double peakAmps = 0;
    /** Longest unbroken run of current at or over the threshold, ms. Feeds STALL_TIMEOUT_MS. */
    private long longestOverRunMs = 0;
    private long overSinceMs = 0;
    private boolean antiJamAtStart;

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
    protected void onInit() {
        antiJamAtStart = Intake.ANTI_JAM_ENABLED;
    }

    @Override
    protected void onDecide() {
        // Latched, not held: freeing a jam by hand takes both hands.
        if (gamepad1.aWasPressed()) robot.intake.in();
        if (gamepad1.bWasPressed()) robot.intake.out();
        if (gamepad1.xWasPressed()) robot.intake.idle();
        if (gamepad1.yWasPressed()) robot.intake.stop();

        if (gamepad1.leftBumperWasPressed()) Intake.ANTI_JAM_ENABLED = !Intake.ANTI_JAM_ENABLED;
        if (gamepad1.rightBumperWasPressed()) {
            peakAmps = 0;
            longestOverRunMs = 0;
            overSinceMs = 0;
        }
    }

    @Override
    protected void onAfterAct() {
        Intake.Mode mode = robot.intake.getMode();
        double power = robot.intake.powerFor(mode);
        double velocity = robot.intake.getVelocity();
        if (power != 0 && !robot.intake.isUnjamming()
                && Math.signum(velocity) == Math.signum(power)
                && Math.abs(velocity) >= MIN_TICKS_PER_SEC) {
            passed.add(mode);
        }

        // NaN is "not sampled": the current read only happens while the roller pulls in.
        double amps = robot.intake.getCurrentAmps();
        if (!Double.isNaN(amps) && amps > peakAmps) peakAmps = amps;
        if (!Double.isNaN(amps) && amps >= Intake.STALL_CURRENT_AMPS) {
            long now = System.currentTimeMillis();
            if (overSinceMs == 0) overSinceMs = now;
            longestOverRunMs = Math.max(longestOverRunMs, now - overSinceMs);
        } else {
            overSinceMs = 0;
        }
    }

    @Override
    protected void onTelemetry() {
        telemetry.addData("Loop", "%.0f Hz", robot.getLoopHz());
        telemetry.addLine("A=in  B=out  X=idle  Y=off   LB=anti-jam on/off   RB=clear peaks");
        telemetry.addLine();

        Intake.Mode mode = robot.intake.getMode();
        telemetry.addData("Mode", "%s  (power %.2f)", mode, robot.intake.powerFor(mode));
        telemetry.addData("Velocity", "%.0f t/s", robot.intake.getVelocity());
        double amps = robot.intake.getCurrentAmps();
        telemetry.addData("Current", "%s   PEAK %.2f A",
                Double.isNaN(amps) ? "not sampled" : String.format("%.2f A", amps), peakAmps);
        telemetry.addData("Anti-jam", "%s   suspect=%s unjamming=%s attempts=%d givenUp=%s",
                Intake.ANTI_JAM_ENABLED ? "ON" : "OFF", robot.intake.isStallSuspected(),
                robot.intake.isUnjamming(), robot.intake.getUnjamAttempts(),
                robot.intake.hasGivenUpUnjamming());
        telemetry.addLine();

        telemetry.addLine("Checks");
        telemetry.addData("  IN", verdict(Intake.Mode.IN));
        telemetry.addData("  OUT", verdict(Intake.Mode.OUT));
        telemetry.addData("  IDLE", verdict(Intake.Mode.IDLE));
        telemetry.addData("  current sampled", peakAmps > 0 ? "PASS" : "-  (run IN)");
        telemetry.addLine("By eye: the roller AND the tunnel both turn; IN pulls a piece inward.");
        telemetry.addLine();
        telemetry.addData("STALL_CURRENT_AMPS", "%.2f  (between a clean capture's PEAK and a stall's)",
                Intake.STALL_CURRENT_AMPS);
        telemetry.addData("STALL_TIMEOUT_MS", "%d  (must beat a clean capture's run over threshold: %d ms)",
                Intake.STALL_TIMEOUT_MS, longestOverRunMs);
    }

    private String verdict(Intake.Mode mode) {
        return passed.contains(mode) ? "PASS" : "-";
    }

    @Override
    protected void onStop() {
        // No motor writes here: the SDK rejects them from stop() and zeroes the motors itself.
        Intake.ANTI_JAM_ENABLED = antiJamAtStart;
    }
}
