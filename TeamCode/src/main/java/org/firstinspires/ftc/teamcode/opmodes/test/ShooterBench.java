package org.firstinspires.ftc.teamcode.opmodes.test;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.commands.Shoot;
import org.firstinspires.ftc.teamcode.opmodes.MatchOpMode;
import org.firstinspires.ftc.teamcode.subsystems.Shooter;
import org.firstinspires.ftc.teamcode.util.field.Alliance;
import org.firstinspires.ftc.teamcode.util.time.MatchClock;

/**
 * Finds the three flywheel gains and the distance table. The most important bench on the robot:
 * nothing else produces a number the shooter cannot work without.
 *
 * <h2>The workflow, in order</h2>
 * <ol>
 *   <li><b>kS</b> — raise a raw power until the wheels just turn. That power is {@link Shooter#kS}.
 *       Note the <em>breakaway</em> power (the one that starts it from rest) is higher than the one
 *       that keeps it turning; the card shows both and kS is the lower.</li>
 *   <li><b>kV</b> — hold a raw power, let the velocity settle, read {@code power / velocity}.</li>
 *   <li><b>kP</b> — close the loop at a target, fire a piece through with RB, and watch the dip and
 *       the milliseconds back to tolerance. Trim {@link Shooter#kP} live until recovery is quick
 *       without the wheel overshooting.</li>
 *   <li><b>Table</b> — set a speed that scores from a measured distance, and paste the pair.</li>
 * </ol>
 *
 * <h2>No motor handles here</h2>
 * Everything goes through {@code Shooter}: the open-loop steps use
 * {@link Shooter#setOpenLoopPower(double)}, which sets a mode so {@code update()} stays the only
 * thing that writes hardware. A bench holding its own {@code DcMotorEx} would give the port a second
 * power cache, and {@code Robot.stopMechanisms()} could no longer stop the wheel.
 */
@TeleOp(name = "Bench: Shooter", group = "Bench")
public class ShooterBench extends MatchOpMode {
    private enum Step { KS, KV, KP, TABLE }

    /** Velocity band and dwell that count as "settled" for the kV reading. */
    public static double SETTLED_BAND_TICKS = 25;
    public static long SETTLED_MS = 500;

    private Step step = Step.KS;
    private boolean running = false;
    private double rawPower = 0;
    private double distanceInches = 48;

    private double turningPower = Double.NaN;   // lowest power seen with the wheel actually turning
    private double breakawayPower = Double.NaN; // power at which it started from rest
    private double settledVelocity = Double.NaN;
    private long inBandSinceMs = 0;

    private double dipFloor = Double.NaN;       // lowest velocity seen after a pulse
    private long recoveredMs = -1;
    private long pulseAtMs = -1;
    private long pulseEndsAtMs = -1;

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
        return false;
    }

    @Override
    protected void onDecide() {
        if (gamepad1.bWasPressed()) changeStep(1);
        if (gamepad1.leftBumperWasPressed()) changeStep(-1);
        if (gamepad1.aWasPressed()) running = !running;
        if (gamepad1.yWasPressed()) resetReadings();

        if (gamepad1.xWasPressed()) {
            // Never reverse a spinning flywheel: stop first, then flip.
            stopWheel();
            Shooter.SECOND_MOTOR_REVERSED = !Shooter.SECOND_MOTOR_REVERSED;
            robot.shooter.applySecondMotorDirection();
        }

        if (gamepad1.rightBumperWasPressed()) startPulse();

        boolean openLoop = step == Step.KS || step == Step.KV;
        if (openLoop) {
            if (gamepad1.dpadUpWasPressed()) rawPower += 0.01;
            if (gamepad1.dpadDownWasPressed()) rawPower -= 0.01;
            if (gamepad1.dpadRightWasPressed()) rawPower += 0.002;   // fine, for kS
            if (gamepad1.dpadLeftWasPressed()) rawPower -= 0.002;
            rawPower = Math.max(0, Math.min(1, rawPower));
        } else {
            if (gamepad1.dpadUpWasPressed()) robot.shooter.setTarget(robot.shooter.getTarget() + 25);
            if (gamepad1.dpadDownWasPressed()) robot.shooter.setTarget(robot.shooter.getTarget() - 25);
            if (step == Step.KP) {
                if (gamepad1.dpadRightWasPressed()) Shooter.kP += 0.002;
                if (gamepad1.dpadLeftWasPressed()) Shooter.kP = Math.max(0, Shooter.kP - 0.002);
            } else {
                if (gamepad1.dpadRightWasPressed()) distanceInches += 6;
                if (gamepad1.dpadLeftWasPressed()) distanceInches = Math.max(0, distanceInches - 6);
            }
        }

        // Apply the step's intent. A bench re-asks for motion every loop, so releasing a control or
        // hitting a guard always comes to rest rather than latching.
        if (!running) {
            stopWheel();
        } else if (openLoop) {
            robot.shooter.setOpenLoopPower(rawPower);
        } else {
            robot.shooter.endOpenLoop();
            robot.shooter.arm();
        }

        // The tunnel pulse that pushes a piece through the wheel.
        long now = System.currentTimeMillis();
        if (pulseEndsAtMs > 0 && now < pulseEndsAtMs) robot.intake.in();
        else {
            robot.intake.stop();
            pulseEndsAtMs = -1;
        }
    }

    private void changeStep(int delta) {
        Step[] all = Step.values();
        int i = Math.max(0, Math.min(all.length - 1, step.ordinal() + delta));
        if (all[i] == step) return;
        stopWheel();            // a step change always stops the wheel
        step = all[i];
        resetReadings();
    }

    private void stopWheel() {
        running = false;
        robot.shooter.endOpenLoop();
        robot.shooter.disarm();
    }

    private void resetReadings() {
        turningPower = Double.NaN;
        breakawayPower = Double.NaN;
        settledVelocity = Double.NaN;
        inBandSinceMs = 0;
        dipFloor = Double.NaN;
        recoveredMs = -1;
        pulseAtMs = -1;
    }

    private void startPulse() {
        pulseAtMs = System.currentTimeMillis();
        pulseEndsAtMs = pulseAtMs + Shoot.FEED_PULSE_MS;
        dipFloor = Double.NaN;
        recoveredMs = -1;
    }

    @Override
    protected void onAfterAct() {
        long now = System.currentTimeMillis();
        double v = robot.shooter.getVelocity();
        boolean turning = Math.abs(v) > SETTLED_BAND_TICKS;

        if (robot.shooter.isOpenLoop() && running) {
            if (turning) {
                if (Double.isNaN(breakawayPower)) breakawayPower = rawPower;
                if (Double.isNaN(turningPower) || rawPower < turningPower) turningPower = rawPower;
            }
            // Settled = inside a band for a dwell, so kV is not read off a still-accelerating wheel.
            if (!Double.isNaN(settledVelocity) && Math.abs(v - settledVelocity) > SETTLED_BAND_TICKS) {
                inBandSinceMs = 0;
            }
            if (inBandSinceMs == 0) {
                inBandSinceMs = now;
                settledVelocity = v;
            } else if (now - inBandSinceMs >= SETTLED_MS) {
                settledVelocity = v;
            }
        }

        if (pulseAtMs > 0) {
            if (Double.isNaN(dipFloor) || v < dipFloor) dipFloor = v;
            if (recoveredMs < 0 && robot.shooter.atTarget() && now - pulseAtMs > Shoot.SETTLE_AFTER_PULSE_MS) {
                recoveredMs = now - pulseAtMs;
            }
        }
    }

    @Override
    protected void onTelemetry() {
        telemetry.addData("Loop", "%.0f Hz", robot.getLoopHz());
        telemetry.addData("STEP", "%d/4  %s   %s", step.ordinal() + 1, step, running ? "RUNNING" : "stopped");
        telemetry.addLine("A=run/stop  B=next  LB=back  Y=reset  X=flip 2nd motor  RB=feed a piece");
        telemetry.addLine();

        switch (step) {
            case KS:
                telemetry.addLine("Raise power until the wheels JUST keep turning. dpad up/down=0.01, left/right=0.002");
                telemetry.addData("power", "%.3f", rawPower);
                telemetry.addData("breakaway (starts from rest)", fmt(breakawayPower));
                telemetry.addData("PASTE Shooter.kS", "%s  (currently %.4f)", fmt(turningPower), Shooter.kS);
                break;
            case KV:
                telemetry.addLine("Hold a power near full, let it settle, read power/velocity.");
                telemetry.addData("power", "%.3f", rawPower);
                telemetry.addData("settled velocity", fmt(settledVelocity));
                telemetry.addData("PASTE Shooter.kV", "%s  (currently %.5f)",
                        Double.isNaN(settledVelocity) || settledVelocity == 0
                                ? "-" : String.format("%.5f", rawPower / settledVelocity),
                        Shooter.kV);
                break;
            case KP:
                telemetry.addLine("Closed loop. RB to fire a piece; watch the dip and the recovery.");
                telemetry.addData("dip floor", fmt(dipFloor));
                telemetry.addData("recovered in", recoveredMs < 0 ? "-" : recoveredMs + " ms");
                telemetry.addData("PASTE Shooter.kP", "%.4f", Shooter.kP);
                break;
            default:
                telemetry.addLine("Set a speed that scores from a measured distance, then paste the pair.");
                telemetry.addData("distance", "%.0f in   (table says %.0f t/s)",
                        distanceInches, robot.shooter.tableTargetFor(distanceInches));
                telemetry.addData("PASTE table pair", "{%.0f in, %.0f t/s}",
                        distanceInches, robot.shooter.getTarget());
                telemetry.addData("PASTE MANUAL_TICKS_PER_SEC", "%.0f", robot.shooter.getTarget());
                break;
        }

        telemetry.addLine();
        telemetry.addData("target / actual", "%.0f / %.0f  err %.0f  atTarget=%s",
                robot.shooter.getTarget(), robot.shooter.getVelocity(),
                robot.shooter.getTarget() - robot.shooter.getVelocity(), robot.shooter.atTarget());
        telemetry.addData("power written", fmt(robot.shooter.getLastWritten()));
        telemetry.addData("pair", "left %.0f   right %.0f   %s", robot.shooter.getVelocity(),
                robot.shooter.getSecondVelocity(), pairVerdict());
        telemetry.addData("SECOND_MOTOR_REVERSED", Shooter.SECOND_MOTOR_REVERSED);
        // A tired battery looks exactly like a bad kV: the same power produces less speed.
        telemetry.addData("battery", "%.1f V", robot.getBatteryVolts());
    }

    private String pairVerdict() {
        double a = robot.shooter.getVelocity();
        double b = robot.shooter.getSecondVelocity();
        if (Math.abs(a) < SETTLED_BAND_TICKS && Math.abs(b) < SETTLED_BAND_TICKS) return "";
        if (Math.abs(a) < SETTLED_BAND_TICKS || Math.abs(b) < SETTLED_BAND_TICKS) return "!! ONE IS DEAD";
        if (Math.signum(a) != Math.signum(b)) return "!! FIGHTING - flip with X";
        return Math.abs(Math.abs(a) - Math.abs(b)) > 4 * SETTLED_BAND_TICKS ? "!! MISMATCHED" : "matched";
    }

    private static String fmt(double v) {
        return Double.isNaN(v) ? "-" : String.format("%.3f", v);
    }

    @Override
    protected void onStop() {
        robot.stopMechanisms();
    }
}
