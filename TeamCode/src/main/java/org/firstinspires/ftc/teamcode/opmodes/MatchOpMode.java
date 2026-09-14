package org.firstinspires.ftc.teamcode.opmodes;

import com.pedropathing.ivy.Scheduler;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.util.diagnostics.LoopTimer;
import org.firstinspires.ftc.teamcode.util.diagnostics.MatchLogger;
import org.firstinspires.ftc.teamcode.util.time.MatchClock;

import java.io.IOException;
import java.util.List;

/**
 * Everything a match OpMode has to do, done once: building the robot, resetting the scheduler,
 * opening a log, setting the telemetry rate, resetting gamepad edge detection, starting the match
 * clock, timing the loop, and closing it all down again. Two copies of a lifecycle would be two
 * places for it to drift.
 *
 * <h2>The loop order is enforced here, not remembered</h2>
 * {@link #loop()} is {@code final}. Subclasses fill in the gaps between the fixed steps:
 *
 * <pre>
 *   readSensors()        1. observe   (fixed)
 *   updateLocalization()
 *   onDecide()                        (yours: read buttons, schedule commands)
 *   Scheduler.execute()  2. decide    (fixed)
 *   writeActuators()     3. act       (fixed)
 *   onAfterAct()                      (yours: haptics, pose hand-off)
 *   log / telemetry                   (fixed)
 * </pre>
 *
 * Observe, decide, act, in that order. A subclass cannot get it wrong because there is no ordering
 * left for it to choose (docs/03 section 1). One {@link #nowMs} timestamp is taken per loop and
 * shared, rather than each caller reading the clock again.
 *
 * <p>Two protected seams, {@link #buildRobot()} and {@link #openLogger()}, exist so a JVM test can
 * run this whole lifecycle on a robot made of fakes; production never overrides them.
 *
 * <p>The SDK transmits telemetry itself after every {@code init_loop()} and {@code loop()}, so
 * nothing here calls {@code telemetry.update()}.
 */
public abstract class MatchOpMode extends OpMode {
    /**
     * How often telemetry is transmitted to the Driver Station, in milliseconds. The SDK default is
     * 250 ms; set explicitly so the rate is a decision, and so it can be lowered while debugging.
     */
    public static int TELEMETRY_INTERVAL_MS = 100;

    protected Robot robot;
    protected MatchLogger logger;
    /** Non-null when the log file could not be opened; subclasses surface it in telemetry. */
    protected String loggerError = null;

    /** Duration of the previous loop, in milliseconds. */
    protected double loopMs = 0;
    /** Loop-time statistics for the whole run: p95, max, spike count. */
    protected final LoopTimer loopStats = new LoopTimer();
    /** One timestamp per loop, from the robot's {@code Clock}, so nothing re-reads it mid-cycle. */
    protected long nowMs = 0;

    private final ElapsedTime loopTimer = new ElapsedTime();

    // ---- Subclass contract ----

    /** Filename prefix for this OpMode's match log, e.g. {@code "teleop"}. */
    protected abstract String logTag();

    /** Which match period this OpMode runs in, for the match clock. */
    protected abstract MatchClock.Period matchPeriod();

    /** Built subsystems are available; schedule default commands here. */
    protected void onInit() {}

    /** Runs after {@code readSensors()} on every init loop. Menus, localisation, warnings. */
    protected void onInitLoop() {}

    /** Runs once on START, after edge detection is reset, the clock has started and the drivetrain is in manual. */
    protected void onStart() {}

    /** Read inputs and schedule commands. Runs before the scheduler, on fresh sensor data. */
    protected void onDecide() {}

    /** Runs after actuators are written. Haptics, pose hand-off: anything that observes the result. */
    protected void onAfterAct() {}

    /** Add this OpMode's telemetry. Called every loop. */
    protected void onTelemetry() {}

    /** Runs first in {@code stop()}, while the robot is still live. */
    protected void onStop() {}

    // ---- Seams for tests ----

    /** The robot this OpMode drives. Tests return one assembled from fakes. */
    protected Robot buildRobot() {
        return new Robot(hardwareMap);
    }

    /** The match log, in {@link MatchLogger#BIOBUZZ_COLUMNS} order. Tests return {@code null}. */
    protected MatchLogger openLogger() throws IOException {
        return new MatchLogger(logTag(), MatchLogger.BIOBUZZ_COLUMNS);
    }

    // ---- Fixed lifecycle ----

    @Override
    public final void init() {
        // The scheduler is static and survives OpMode restarts: clear it before anything schedules.
        Scheduler.reset();
        robot = buildRobot();
        telemetry.setMsTransmissionInterval(TELEMETRY_INTERVAL_MS);

        try {
            logger = openLogger();
            loggerError = null;
        } catch (Exception e) {
            // A missing SD card must not cost a match. Record it and carry on unlogged.
            logger = null;
            loggerError = String.valueOf(e.getMessage());
        }

        onInit();
        reportMissingHardware();
    }

    @Override
    public final void init_loop() {
        robot.readSensors();
        onInitLoop();
    }

    @Override
    public final void start() {
        // *WasPressed() latches until read, and init_loop() reads few of them. Without this, every
        // button bumped during init fires at once on the first loop tick, including the ones that
        // launch a macro.
        gamepad1.resetEdgeDetection();
        gamepad2.resetEdgeDetection();
        robot.startMatch(matchPeriod());
        robot.drivetrain.onStart();      // follower to the sticks; init pose repeats after calibration
        loopTimer.reset();
        onStart();
    }

    @Override
    public final void loop() {
        loopMs = loopTimer.milliseconds();
        loopTimer.reset();
        loopStats.record(loopMs);
        nowMs = robot.getClock().nowMs();

        robot.readSensors();          // 1. observe
        robot.updateLocalization();   //    blend any absolute fix into the pose estimate
        onDecide();                   // 2. decide
        Scheduler.execute();          //    driver control and macros both run here
        robot.writeActuators();       // 3. act

        onAfterAct();
        if (logger != null) logger.logRow(robot.logCells(loopMs));
        onTelemetry();
    }

    @Override
    public final void stop() {
        onStop();
        Scheduler.reset();
        if (robot != null) robot.stop();
        if (logger != null) logger.close();
    }

    /** Lists any configuration name that could not be resolved. Empty output means all present. */
    protected void reportMissingHardware() {
        List<String> missing = robot.getMissingHardware();
        if (missing.isEmpty()) {
            telemetry.addLine("All hardware present.");
            return;
        }
        telemetry.addLine("MISSING HARDWARE (the robot will still run):");
        for (String name : missing) telemetry.addLine("  - " + name);
    }
}
