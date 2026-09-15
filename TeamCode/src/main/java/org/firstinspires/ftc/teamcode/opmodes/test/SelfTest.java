package org.firstinspires.ftc.teamcode.opmodes.test;

import com.qualcomm.hardware.limelightvision.LLStatus;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.subsystems.ColorSensor;

import java.util.LinkedHashMap;
import java.util.Map;

/**
 * Run first at every event: builds the real {@code Robot}, then steps through every subsystem and
 * leaves a PASS / FAIL / SKIP table on the screen. Each velocity mechanism is commanded at a low
 * speed for {@link #SETTLE_MS}, then its measured velocity is read back: the motor is there, wired
 * the right way round, and its encoder counts. Nothing here needs a game piece; SKIP means "not
 * fitted", not "broken".
 *
 * <p><b>MECHANISMS WILL SPIN.</b> Put the robot on blocks and clear the intake and shooter.
 */
@TeleOp(name = "SelfTest", group = "Bench")
public class SelfTest extends BenchOpMode {
    public static long SETTLE_MS = 600;
    /** A mechanism commanded at the test speed must read at least this back to pass. */
    public static double MIN_TICKS_PER_SEC = 100;
    public static double TEST_TICKS_PER_SEC = 600;
    public static double TEST_RPM = 600;
    public static double MIN_BATTERY_VOLTS = 12.0;

    private enum Step { CONFIG, BATTERY, INTAKE, STORAGE, TRANSFER, SHOOTER, SENSORS, LIMELIGHT, DRIVE, LOCALIZER, DONE }

    private Step step = Step.CONFIG;
    private long stepStartedMs = 0;
    private final Map<String, String> results = new LinkedHashMap<>();

    @Override
    protected String title() {
        return "SELF TEST   MECHANISMS WILL SPIN: robot on blocks, clear the intake and shooter";
    }

    @Override
    protected String[] controls() {
        return new String[] {"START runs every check in turn; the table stays on screen when done."};
    }

    @Override
    protected void onBench() {
        if (stepStartedMs == 0) stepStartedMs = nowMs;
        long inStep = nowMs - stepStartedMs;

        switch (step) {
            case CONFIG:
                results.put("config names", robot.getMissingHardware().isEmpty() ? "PASS"
                        : "FAIL missing " + robot.getMissingHardware());
                next();
                break;
            case BATTERY: {
                double volts = robot.getBatteryVolts();
                results.put("battery", volts <= 0 ? "SKIP no voltage sensor"
                        : volts >= MIN_BATTERY_VOLTS ? fmt("PASS %.2f V", volts) : fmt("FAIL %.2f V < %.1f", volts, MIN_BATTERY_VOLTS));
                next();
                break;
            }
            case INTAKE:
                if (!robot.intake.isAvailable()) { results.put("intake", "SKIP not fitted"); next(); break; }
                robot.intake.setVelocity(TEST_TICKS_PER_SEC);
                if (inStep >= SETTLE_MS) {
                    results.put("intake", velocityVerdict(robot.intake.getVelocityTicksPerSec()));
                    robot.intake.stop();
                    next();
                }
                break;
            case STORAGE:
                if (!robot.storage.isAvailable()) { results.put("storage", "SKIP not fitted"); next(); break; }
                robot.storage.advance();
                if (inStep >= SETTLE_MS) {
                    results.put("storage", velocityVerdict(robot.storage.getVelocityTicksPerSec()));
                    robot.storage.stop();
                    next();
                }
                break;
            case TRANSFER:
                if (!robot.transfer.isAvailable()) { results.put("transfer", "SKIP not fitted"); next(); break; }
                robot.transfer.liftPiece();
                if (inStep >= SETTLE_MS) {
                    results.put("transfer", velocityVerdict(robot.transfer.getVelocityTicksPerSec()));
                    robot.transfer.stop();
                    next();
                }
                break;
            case SHOOTER:
                if (!robot.shooter.isAvailable()) { results.put("shooter", "SKIP not fitted"); next(); break; }
                robot.shooter.setTargetRpm(TEST_RPM);
                if (inStep >= SETTLE_MS) {
                    double rpm = robot.shooter.getRpm();
                    results.put("shooter", rpm > 0.3 * TEST_RPM ? fmt("PASS %.0f rpm at %.0f", rpm, TEST_RPM)
                            : fmt("FAIL %.0f rpm at %.0f (wiring? TICKS_PER_REV?)", rpm, TEST_RPM));
                    robot.shooter.stop();
                    next();
                }
                break;
            case SENSORS:
                sensorVerdict("sensor entrance", robot.storageEntranceSensor);
                sensorVerdict("sensor full", robot.storageFullSensor);
                sensorVerdict("sensor transfer", robot.transferSensor);
                sensorVerdict("sensor feed", robot.shooterFeedSensor);
                next();
                break;
            case LIMELIGHT: {
                if (!robot.limelight.isAvailable()) { results.put("limelight", "SKIP not fitted"); next(); break; }
                LLStatus status = robot.limelight.getStatus();
                if (status != null && status.getFps() > 0) {
                    results.put("limelight", fmt("PASS %.0f fps", status.getFps()));
                    next();
                } else if (inStep >= 3000) {
                    results.put("limelight", "FAIL no frames in 3 s");
                    next();
                }
                break;
            }
            case DRIVE:
                results.put("drive", robot.drivetrain.isAvailable() ? "PASS Pedro follower"
                        : robot.openLoopDrive.isAvailable() ? "PASS open loop (Pedro not tuned)" : "FAIL no drive motors");
                next();
                break;
            case LOCALIZER:
                if (!robot.drivetrain.isAvailable()) { results.put("localizer", "SKIP no follower"); next(); break; }
                if (robot.drivetrain.isLocalizerSettled()) {
                    results.put("localizer", "PASS settled, pose " + robot.drivetrain.getPose());
                    next();
                } else if (inStep >= 3000) {
                    results.put("localizer", "FAIL not settled after 3 s");
                    next();
                }
                break;
            case DONE:
            default:
                robot.stopMechanisms();
                break;
        }

        telemetry.addData("Step", step);
        for (Map.Entry<String, String> row : results.entrySet()) telemetry.addData(row.getKey(), row.getValue());
    }

    private void next() {
        step = Step.values()[Math.min(step.ordinal() + 1, Step.DONE.ordinal())];
        stepStartedMs = nowMs;
    }

    private static String velocityVerdict(double measured) {
        if (measured >= MIN_TICKS_PER_SEC) return fmt("PASS %.0f t/s", measured);
        if (measured <= -MIN_TICKS_PER_SEC) return fmt("FAIL runs BACKWARD (%.0f t/s): flip the direction", measured);
        return fmt("FAIL %.0f t/s: no motion or no encoder", measured);
    }

    private void sensorVerdict(String name, ColorSensor sensor) {
        if (!sensor.isAvailable()) { results.put(name, "SKIP not fitted"); return; }
        boolean alive = sensor.getValue() > 0.02 || sensor.getAlpha() > 0.02 || !Double.isNaN(sensor.getDistanceInches());
        results.put(name, alive ? fmt("PASS val %.2f dist %s", sensor.getValue(), num(sensor.getDistanceInches(), "%.1f"))
                : "FAIL reads nothing (cable? I2C address?)");
    }

    public boolean isDone() {
        return step == Step.DONE;
    }

    /** The verdict table, for tests and for anyone logging it. */
    public Map<String, String> getResults() {
        return results;
    }
}
