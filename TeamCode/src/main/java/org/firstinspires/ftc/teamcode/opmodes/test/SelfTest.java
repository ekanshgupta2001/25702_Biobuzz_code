package org.firstinspires.ftc.teamcode.opmodes.test;

import com.pedropathing.math.Pose;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.opmodes.MatchOpMode;
import org.firstinspires.ftc.teamcode.util.field.Alliance;
import org.firstinspires.ftc.teamcode.util.time.MatchClock;

import java.util.ArrayList;
import java.util.List;

/**
 * Run this first at every event. A clock-stepped state machine that exercises each mechanism in turn
 * and leaves a PASS / WARN / FAIL table on the screen.
 *
 * <h2>There is no SKIP row any more</h2>
 * Nothing on this robot is optional: every config name is resolved with a bare
 * {@code hardwareMap.get}, so a missing device has already thrown at OpMode init and named itself.
 * If this OpMode runs at all, everything is present, and a row can only be PASS, WARN or FAIL.
 *
 * <h2>What it cannot tell you</h2>
 * It catches a dead motor, lead, gearbox or encoder cable. It <b>cannot</b> catch a reversed one: the
 * SDK applies direction to the commanded power and the reported velocity together, so a backwards
 * wheel still reads positive. Wheel directions are the SDK's <b>Utility → TestHardware</b> job, or the
 * Mecanum Tuner's — do that before this. The drive row here only proves the motors turn.
 */
@TeleOp(name = "SelfTest", group = "Bench")
public class SelfTest extends MatchOpMode {
    public static double MIN_BATTERY_VOLTS = 12.0;
    public static double DRIVE_TEST_POWER = 0.25;
    public static long DRIVE_TEST_MS = 600;
    public static double INTAKE_MIN_TICKS_PER_SEC = 50;
    public static long INTAKE_TEST_MS = 800;
    public static double FLYWHEEL_TEST_POWER = 0.3;
    public static double FLYWHEEL_MIN_TICKS_PER_SEC = 100;
    /** Two flywheels further apart than this at the same power are mismatched or fighting. */
    public static double FLYWHEEL_MATCH_TICKS_PER_SEC = 150;
    public static long FLYWHEEL_TEST_MS = 1200;

    private enum Step { BATTERY, DRIVE, INTAKE, FLYWHEEL, CAMERA, LOCALIZER, DONE }

    private Step step = Step.BATTERY;
    private long stepSinceMs = 0;
    private final List<String> results = new ArrayList<>();

    /** Peak magnitudes seen during the step that is running. */
    private double peakA = 0;
    private double peakB = 0;
    private Pose poseAtStepStart = null;

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
    protected void onStart() {
        enter(Step.BATTERY);
    }

    private void enter(Step next) {
        step = next;
        stepSinceMs = System.currentTimeMillis();
        peakA = 0;
        peakB = 0;
        poseAtStepStart = robot.drivetrain.getPose();
    }

    private long elapsed() {
        return System.currentTimeMillis() - stepSinceMs;
    }

    @Override
    protected void onDecide() {
        // Rest first, every loop: a step has to re-ask for motion, so nothing can latch on if this
        // OpMode is stopped or a step falls through.
        robot.intake.stop();
        robot.shooter.endOpenLoop();
        if (step != Step.DRIVE) robot.drivetrain.drive(0, 0, 0);

        switch (step) {
            case BATTERY: {
                double v = robot.getBatteryVolts();
                record("Battery", v >= MIN_BATTERY_VOLTS ? "PASS" : "FAIL",
                        String.format("%.1f V (min %.1f)", v, MIN_BATTERY_VOLTS));
                enter(Step.DRIVE);
                break;
            }
            case DRIVE: {
                robot.drivetrain.drive(DRIVE_TEST_POWER, 0, 0);
                if (elapsed() >= DRIVE_TEST_MS) {
                    robot.drivetrain.drive(0, 0, 0);
                    Pose now = robot.drivetrain.getPose();
                    if (poseAtStepStart == null || now == null) {
                        record("Drive", "WARN",
                                "ran 4 motors forward; no localizer, so movement NOT measured - watch it");
                    } else {
                        double moved = now.distance(poseAtStepStart);
                        record("Drive", moved > 1.0 ? "PASS" : "FAIL",
                                String.format("moved %.1f in (direction NOT checked)", moved));
                    }
                    enter(Step.INTAKE);
                }
                break;
            }
            case INTAKE: {
                robot.intake.in();
                peakA = Math.max(peakA, Math.abs(robot.intake.getVelocity()));
                if (elapsed() >= INTAKE_TEST_MS) {
                    robot.intake.stop();
                    record("Intake", peakA >= INTAKE_MIN_TICKS_PER_SEC ? "PASS" : "FAIL",
                            String.format("peak %.0f t/s (min %.0f) - roller AND tunnel should turn",
                                    peakA, INTAKE_MIN_TICKS_PER_SEC));
                    enter(Step.FLYWHEEL);
                }
                break;
            }
            case FLYWHEEL: {
                robot.shooter.setOpenLoopPower(FLYWHEEL_TEST_POWER);
                peakA = Math.max(peakA, Math.abs(robot.shooter.getVelocity()));
                peakB = Math.max(peakB, Math.abs(robot.shooter.getSecondVelocity()));
                if (elapsed() >= FLYWHEEL_TEST_MS) {
                    robot.shooter.endOpenLoop();
                    boolean alive = peakA >= FLYWHEEL_MIN_TICKS_PER_SEC
                            && peakB >= FLYWHEEL_MIN_TICKS_PER_SEC;
                    boolean matched = Math.abs(peakA - peakB) <= FLYWHEEL_MATCH_TICKS_PER_SEC;
                    record("Flywheel pair", alive && matched ? "PASS" : "FAIL",
                            String.format("left %.0f, right %.0f t/s%s", peakA, peakB,
                                    alive && !matched ? " - MISMATCHED, check SECOND_MOTOR_REVERSED"
                                            : alive ? "" : " - one is dead"));
                    enter(Step.CAMERA);
                }
                break;
            }
            case CAMERA: {
                record("Limelight", robot.limelight.isConnected() ? "PASS" : "FAIL",
                        robot.limelight.getStatusLine());
                enter(Step.LOCALIZER);
                break;
            }
            case LOCALIZER: {
                if (!robot.drivetrain.hasFollower()) {
                    // Expected before AutoTune has been run, so a warning and not a failure - but a
                    // match robot in this state has no pose, no paths, no aim lock and no distance
                    // table, which is why it is not a quiet pass either.
                    record("Pinpoint / follower", "WARN",
                            "not tuned: paste localizerConfig + foresightConfig from AutoTune");
                } else {
                    Pose p = robot.drivetrain.getPose();
                    record("Pinpoint / follower", p != null && robot.drivetrain.isLocalizerSettled()
                                    ? "PASS" : "FAIL",
                            p == null ? "follower built but no pose" : "pose available, IMU settled");
                }
                enter(Step.DONE);
                break;
            }
            default:
                break;
        }
    }

    private void record(String name, String verdict, String detail) {
        results.add(String.format("%-20s %-5s %s", name, verdict, detail));
    }

    @Override
    protected void onTelemetry() {
        telemetry.addData("SelfTest", step == Step.DONE ? "COMPLETE" : "running: " + step);
        telemetry.addLine();
        for (String row : results) telemetry.addLine(row);
        if (step != Step.DONE) telemetry.addLine("...");
        telemetry.addLine();
        telemetry.addLine("Wheel DIRECTIONS are not checked here: use Utility > TestHardware.");
    }

    // No onStop: the SDK rejects motor writes from stop() and zeroes the motors itself. Inside the
    // loop, onDecide() already returns every mechanism to rest before each step.
}
