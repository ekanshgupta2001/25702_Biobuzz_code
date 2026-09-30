//package org.firstinspires.ftc.teamcode.opmodes.test;
//
//import com.acmerobotics.dashboard.FtcDashboard;
//import com.acmerobotics.dashboard.config.reflection.ReflectionConfig;
//import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
//import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
//
//import org.firstinspires.ftc.teamcode.commands.Shoot;
//import org.firstinspires.ftc.teamcode.opmodes.MatchOpMode;
//import org.firstinspires.ftc.teamcode.subsystems.Shooter;
//import org.firstinspires.ftc.teamcode.util.field.Alliance;
//import org.firstinspires.ftc.teamcode.util.time.MatchClock;
//
///**
// * Proves the shooter works and tunes it live from FTC Dashboard, without a redeploy per number.
// *
// * <h2>Not match legal</h2>
// * FTC Dashboard runs a web server whenever it is on the classpath, and R704 bans that in a match.
// * Before an event this file and the {@code dashboard} dependency are removed by hand (HANDOFF §4).
// * Open {@code http://192.168.43.1:8080/dash} on the robot's Wi-Fi.
// *
// * <h2>What is live</h2>
// * Every {@code public static} in {@link Shooter} appears in Dashboard's config panel under "Shooter",
// * and this bench's own under "ShooterBench". {@code Shooter.update()} reads {@code kS}, {@code kV} and
// * {@code kP} every loop, and this bench pushes {@link Shooter#TARGET_TICKS_PER_SEC} into
// * {@code setTarget} every loop, so an edit in the browser lands on the next loop. Flipping
// * {@link Shooter#SECOND_MOTOR_REVERSED} there (or with X) stops the wheel first, then re-applies it.
// *
// * <p>Edits are <b>not</b> saved to source. They live until the Robot Controller app restarts, so a
// * Teleop run straight after this bench shoots with the tuned values, which is what you want while
// * practising. An event APK is a fresh deploy with this file stripped, so they cannot reach a match. The
// * numbers must be pasted into {@code Shooter.java} to be real; the card prints them paste-ready.
// *
// * <h2>Graphs</h2>
// * Each loop sends a {@link TelemetryPacket} with the numbers worth plotting: target, velocity (both
// * wheels), error, power written, and the running kV estimate. Dashboard's Graph view plots any of
// * them.
// *
// * <h2>Workflow</h2>
// * <ol>
// *   <li><b>kS</b>: B for open loop, raise {@link #OPEN_LOOP_POWER} in Dashboard until the wheels
// *       just keep turning.</li>
// *   <li><b>kV</b>: open loop near full power, let it settle, read {@code kV estimate}.</li>
// *   <li><b>kP</b>: A to arm, RB to push a piece through, watch the dip and the recovery on the graph,
// *       and trim kP until recovery is quick without overshoot.</li>
// *   <li><b>Table</b>: set a target that scores from a measured distance and paste the pair.</li>
// * </ol>
// *
// * <p>No motor handles here: everything goes through {@code Shooter}, whose {@code update()} stays the
// * only writer (docs/03 §17).
// */
//@TeleOp(name = "Bench: Shooter", group = "Bench")
//public class ShooterBench extends MatchOpMode {
//    /** Raw power for the open-loop (kS / kV) runs. Edit it in Dashboard. */
//    public static double OPEN_LOOP_POWER = 0.3;
//    /** dpad up/down step for the closed-loop target, ticks/sec. */
//    public static double TARGET_STEP_TICKS_PER_SEC = 25;
//    /** Below this a wheel counts as stopped, for the pair check. */
//    public static double STOPPED_TICKS_PER_SEC = 25;
//
//    private enum Run { STOPPED, OPEN_LOOP, CLOSED_LOOP }
//
//    private FtcDashboard dashboard;
//    private Run run = Run.STOPPED;
//    private boolean appliedSecondReversed;
//
//    private long pulseAtMs = -1;
//    private long pulseEndsAtMs = -1;
//    private double dipFloor = Double.NaN;
//    private long recoveredMs = -1;
//
//    @Override
//    protected Alliance alliance() {
//        return Alliance.BLUE;
//    }
//
//    @Override
//    protected MatchClock.Period matchPeriod() {
//        return MatchClock.Period.TELEOP;
//    }
//
//    @Override
//    protected boolean usesScheduler() {
//        return false;
//    }
//
//    @Override
//    protected void onInit() {
//        appliedSecondReversed = Shooter.SECOND_MOTOR_REVERSED;
//        dashboard = FtcDashboard.getInstance();   // null if Dashboard is disabled on the RC
//        if (dashboard != null) {
//            dashboard.withConfigRoot(root -> {
//                root.putVariable("Shooter", ReflectionConfig.createVariableFromClass(Shooter.class));
//                root.putVariable("ShooterBench", ReflectionConfig.createVariableFromClass(ShooterBench.class));
//            });
//            dashboard.updateConfig();
//        }
//    }
//
//    @Override
//    protected void onInitLoop() {
//        telemetry.addLine(dashboard == null
//                ? "!! FTC Dashboard not running: enable it on the RC, or tune by redeploying"
//                : "Dashboard: http://192.168.43.1:8080/dash  (config: Shooter, ShooterBench)");
//    }
//
//    @Override
//    protected void onDecide() {
//        if (gamepad1.aWasPressed()) run = run == Run.CLOSED_LOOP ? Run.STOPPED : Run.CLOSED_LOOP;
//        if (gamepad1.bWasPressed()) run = run == Run.OPEN_LOOP ? Run.STOPPED : Run.OPEN_LOOP;
//        if (gamepad1.yWasPressed()) resetReadings();
//        if (gamepad1.xWasPressed()) Shooter.SECOND_MOTOR_REVERSED = !Shooter.SECOND_MOTOR_REVERSED;
//        if (gamepad1.dpadUpWasPressed()) Shooter.TARGET_TICKS_PER_SEC += TARGET_STEP_TICKS_PER_SEC;
//        if (gamepad1.dpadDownWasPressed()) {
//            Shooter.TARGET_TICKS_PER_SEC = Math.max(0, Shooter.TARGET_TICKS_PER_SEC - TARGET_STEP_TICKS_PER_SEC);
//        }
//        if (gamepad1.rightBumperWasPressed()) startPulse();
//
//        // Never reverse a spinning flywheel: stop first, then flip. Catches a flip from X or Dashboard.
//        if (Shooter.SECOND_MOTOR_REVERSED != appliedSecondReversed) {
//            run = Run.STOPPED;
//            robot.shooter.endOpenLoop();
//            robot.shooter.disarm();
//            robot.shooter.applySecondMotorDirection();
//            appliedSecondReversed = Shooter.SECOND_MOTOR_REVERSED;
//        }
//
//        // Re-asked every loop, so a Dashboard edit lands on the next loop and stopping always rests.
//        robot.shooter.setTarget(Shooter.TARGET_TICKS_PER_SEC);
//        switch (run) {
//            case OPEN_LOOP:
//                robot.shooter.setOpenLoopPower(Math.max(0, Math.min(1, OPEN_LOOP_POWER)));
//                break;
//            case CLOSED_LOOP:
//                robot.shooter.endOpenLoop();
//                robot.shooter.arm();
//                break;
//            default:
//                robot.shooter.endOpenLoop();
//                robot.shooter.disarm();
//                break;
//        }
//
//        // The tunnel pulse that pushes a piece through the wheel.
//        long now = System.currentTimeMillis();
//        if (pulseEndsAtMs > 0 && now < pulseEndsAtMs) {
//            robot.intake.in();
//        } else {
//            robot.intake.stop();
//            pulseEndsAtMs = -1;
//        }
//    }
//
//    private void startPulse() {
//        pulseAtMs = System.currentTimeMillis();
//        pulseEndsAtMs = pulseAtMs + Shoot.FEED_PULSE_MS;
//        dipFloor = Double.NaN;
//        recoveredMs = -1;
//    }
//
//    private void resetReadings() {
//        pulseAtMs = -1;
//        dipFloor = Double.NaN;
//        recoveredMs = -1;
//    }
//
//    @Override
//    protected void onAfterAct() {
//        if (pulseAtMs < 0) return;
//        long now = System.currentTimeMillis();
//        double v = robot.shooter.getVelocity();
//        if (Double.isNaN(dipFloor) || v < dipFloor) dipFloor = v;
//        if (recoveredMs < 0 && robot.shooter.isArmed() && robot.shooter.atTarget()
//                && now - pulseAtMs > Shoot.SETTLE_AFTER_PULSE_MS) {
//            recoveredMs = now - pulseAtMs;
//        }
//    }
//
//    @Override
//    protected void onTelemetry() {
//        double target = robot.shooter.getTarget();
//        double v = robot.shooter.getVelocity();
//        double v2 = robot.shooter.getSecondVelocity();
//        double written = robot.shooter.getLastWritten();
//        double kvEstimate = run == Run.OPEN_LOOP && Math.abs(v) > STOPPED_TICKS_PER_SEC
//                ? written / v : Double.NaN;
//
//        telemetry.addData("Loop", "%.0f Hz", robot.getLoopHz());
//        telemetry.addData("Run", "%s", run);
//        telemetry.addLine("A=closed loop  B=open loop  RB=feed a piece  Y=reset  X=flip 2nd motor");
//        telemetry.addLine("dpad up/down = target +/-" + (int) TARGET_STEP_TICKS_PER_SEC
//                + "   everything else: Dashboard");
//        telemetry.addLine();
//        telemetry.addData("target / actual", "%.0f / %.0f  err %.0f  atTarget=%s",
//                target, v, target - v, robot.shooter.atTarget());
//        telemetry.addData("power written", fmt(written));
//        telemetry.addData("pair", "left %.0f   right %.0f   %s", v, v2, pairVerdict(v, v2));
//        telemetry.addData("open-loop power", "%.3f   kV estimate %s", OPEN_LOOP_POWER, fmt(kvEstimate));
//        telemetry.addData("last shot", "dip floor %s   recovered %s", fmt(dipFloor),
//                recoveredMs < 0 ? "-" : recoveredMs + " ms");
//        // A tired battery looks exactly like a bad kV: the same power produces less speed.
//        telemetry.addData("battery", "%.1f V", robot.getBatteryVolts());
//        telemetry.addLine();
//        telemetry.addLine("PASTE into Shooter.java:");
//        telemetry.addLine(String.format("  kS = %.4f, kV = %.6f, kP = %.4f", Shooter.kS, Shooter.kV, Shooter.kP));
//        telemetry.addLine(String.format("  TOLERANCE_TICKS_PER_SEC = %.0f   SECOND_MOTOR_REVERSED = %s",
//                Shooter.TOLERANCE_TICKS_PER_SEC, Shooter.SECOND_MOTOR_REVERSED));
//        telemetry.addLine(String.format("  table pair / MANUAL_TICKS_PER_SEC: %.0f t/s", target));
//
//        if (dashboard != null) {
//            TelemetryPacket packet = new TelemetryPacket();
//            packet.put("target", target);
//            packet.put("velocity", v);
//            packet.put("velocity 2nd", v2);
//            packet.put("error", target - v);
//            packet.put("power", Double.isNaN(written) ? 0 : written);
//            packet.put("feeding", pulseEndsAtMs > 0 ? 1 : 0);
//            if (!Double.isNaN(kvEstimate)) packet.put("kV estimate", kvEstimate);
//            packet.put("battery", robot.getBatteryVolts());
//            dashboard.sendTelemetryPacket(packet);
//        }
//    }
//
//    private static String pairVerdict(double a, double b) {
//        boolean aStopped = Math.abs(a) < STOPPED_TICKS_PER_SEC;
//        boolean bStopped = Math.abs(b) < STOPPED_TICKS_PER_SEC;
//        if (aStopped && bStopped) return "";
//        if (aStopped || bStopped) return "!! ONE IS DEAD";
//        if (Math.signum(a) != Math.signum(b)) return "!! FIGHTING - flip with X";
//        return Math.abs(Math.abs(a) - Math.abs(b)) > 4 * STOPPED_TICKS_PER_SEC ? "!! MISMATCHED" : "matched";
//    }
//
//    private static String fmt(double v) {
//        return Double.isNaN(v) ? "-" : String.format("%.5f", v);
//    }
//
//    // No onStop: the SDK rejects motor writes from stop() and zeroes the motors itself. The edited
//    // statics are left as they are on purpose, see the class comment.
//}
