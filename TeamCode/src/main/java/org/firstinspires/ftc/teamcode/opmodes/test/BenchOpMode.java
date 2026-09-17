package org.firstinspires.ftc.teamcode.opmodes.test;

import com.pedropathing.ivy.Scheduler;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.opmodes.RobotTunables;
import org.firstinspires.ftc.teamcode.util.diagnostics.LoopTimer;
import org.firstinspires.ftc.teamcode.util.diagnostics.Tunables;

import java.util.List;
import java.util.Locale;

/**
 * Base for the bench OpModes: one per mechanism, run in the pit with the robot on blocks, to see
 * a subsystem move under the real motor, direction, velocity and sensor code the match OpModes use,
 * and to measure the numbers HANDOFF section 9 asks for. The code is otherwise proven only against
 * fakes, so this is where "it compiles" becomes "it works".
 *
 * <p>Same {@code Robot}, same config names, no Ivy: each loop is {@code readSensors()}, the bench's
 * own button handling that sets intents directly on the subsystems, then {@code writeActuators()}.
 * Because no default commands are scheduled nothing re-asserts idle; a bench that stops calling a
 * mechanism must call its {@code stop()}. Edges are read straight off the gamepads, once per loop.
 *
 * <p>Registered under the "Bench" group so they sort together on the Driver Station, away from the
 * match OpModes. {@link #buildRobot()} is the same JVM seam {@code MatchOpMode} has.
 *
 * <p>A value a bench changes is a {@code public static} and lives until the app restarts, so it is
 * what the next Teleop runs with. That is the pit workflow (tune, then try it on the practice
 * field), so nothing here restores anything on stop; instead every card lists what differs from
 * the compiled defaults, and BACK on gamepad 1 restores them all (fixthese R2-A5).
 */
public abstract class BenchOpMode extends OpMode {
    public static int TELEMETRY_INTERVAL_MS = 50;

    protected Robot robot;
    /** One timestamp per loop from the robot's clock. */
    protected long nowMs = 0;
    protected final LoopTimer loopStats = new LoopTimer();
    private final ElapsedTime loopTimer = new ElapsedTime();

    /** One line naming the bench, shown at the top of every card. */
    protected abstract String title();

    /** Read the pads, set intents, add telemetry. Runs between readSensors() and writeActuators(). */
    protected abstract void onBench();

    /** Runs once after the robot is built. */
    protected void onBenchInit() {}

    /** Runs after readSensors() on every init loop; the default shows the controls. */
    protected void onBenchInitLoop() {
        for (String line : controls()) telemetry.addLine(line);
        telemetry.addLine("BACK: restore every tunable to its compiled default");
    }

    /** The bench's button card. */
    protected abstract String[] controls();

    /** The robot this bench drives. Tests return one assembled from fakes. */
    protected Robot buildRobot() {
        return new Robot(hardwareMap);
    }

    @Override
    public final void init() {
        RobotTunables.snapshot();     // first: the defaults must be on record before this bench edits any
        Scheduler.reset();
        robot = buildRobot();
        telemetry.setMsTransmissionInterval(TELEMETRY_INTERVAL_MS);
        onBenchInit();
        telemetry.addLine(title());
        missingHardware();
    }

    @Override
    public final void init_loop() {
        robot.readSensors();
        telemetry.addLine(title());
        onBenchInitLoop();
        missingHardware();
    }

    @Override
    public final void start() {
        gamepad1.resetEdgeDetection();
        gamepad2.resetEdgeDetection();
        robot.drivetrain.onStart();
        loopTimer.reset();
    }

    @Override
    public final void loop() {
        double loopMs = loopTimer.milliseconds();
        loopTimer.reset();
        loopStats.record(loopMs);
        nowMs = robot.getClock().nowMs();

        robot.readSensors();
        telemetry.addLine(title());
        if (gamepad1.backWasPressed()) Tunables.restoreDefaults();
        onBench();
        robot.writeActuators();
        footer();
    }

    @Override
    public final void stop() {
        // The SDK rejects motor writes from stop() and zeroes every motor itself when an OpMode
        // ends; only non-actuator hardware is released here.
        if (robot != null) robot.stop();
    }

    // ---- Shared telemetry ----

    private void footer() {
        telemetry.addLine();
        double volts = robot.getBatteryVolts();
        telemetry.addData("Battery", volts > 0 ? String.format(Locale.US, "%.2f V", volts) : "n/a");
        telemetry.addData("Loop", loopStats.getStatus());
        telemetry.addData("Sensors", robot.sensingSummary());
        missingHardware();
        List<String> tuned = Tunables.changed();
        if (!tuned.isEmpty()) {
            telemetry.addLine("TUNED THIS SESSION (Teleop will run with these; BACK restores):");
            for (String line : tuned) telemetry.addLine("  " + line);
        }
    }

    private void missingHardware() {
        for (String name : robot.getMissingHardware()) telemetry.addData("!! MISSING", name);
    }

    protected static String fmt(String format, Object... args) {
        return String.format(Locale.US, format, args);
    }

    /** "value" or "n/a" for a reading that is NaN. */
    protected static String num(double value, String format) {
        return Double.isNaN(value) ? "n/a" : String.format(Locale.US, format, value);
    }

    // ---- Shared input helpers ----

    /** {@code value} nudged by {@code step} on the dpad, clamped. */
    protected static double adjust(double value, boolean up, boolean down, double step, double min, double max) {
        if (up) value += step;
        if (down) value -= step;
        return Math.max(min, Math.min(max, value));
    }

    /** Running minimum and maximum of a signal, for "what did it peak at" readouts. */
    protected static final class Peak {
        private double min = Double.NaN;
        private double max = Double.NaN;

        void add(double value) {
            if (Double.isNaN(value)) return;
            if (Double.isNaN(min) || value < min) min = value;
            if (Double.isNaN(max) || value > max) max = value;
        }

        void reset() {
            min = Double.NaN;
            max = Double.NaN;
        }

        String status(String format) {
            return Double.isNaN(max) ? "n/a" : fmt(format + " .. " + format, min, max);
        }
    }
}
