package org.firstinspires.ftc.teamcode.opmodes;

import com.pedropathing.ivy.Scheduler;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.util.time.MatchClock;

/**
 * The OpMode base for autonomous: a fixed lifecycle with hooks. Teleop and the benches are plain
 * {@code OpMode}s that write the same loop out by hand.
 *
 * <h2>The loop order is structural</h2>
 * {@link #loop()} is {@code final} and always runs observe, decide, execute, act, in that order:
 * <pre>
 * robot.readSensors();      // 1. observe  — one consistent snapshot for the whole loop
 * onDecide();               // 2. decide   — schedule commands, safety checks
 * Scheduler.execute();      //    commands run here
 * robot.writeActuators();   // 3. act      — every mechanism writes hardware
 * </pre>
 * A subclass cannot reorder it: a decide step that ran after the act step would see last loop's
 * data, and the bug would look like a tuning problem.
 *
 * <p>The SDK transmits telemetry itself after every {@code init_loop()} and {@code loop()}, so
 * nothing here calls {@code telemetry.update()}.
 */
public abstract class MatchOpMode extends OpMode {
    /**
     * Telemetry transmission interval. The SDK default is 250 ms; set explicitly so the rate is a
     * decision, and so it can be lowered while debugging.
     */
    public static int TELEMETRY_INTERVAL_MS = 100;

    protected Robot robot;

    // ---- Subclass contract ----

    /** Which match period this OpMode runs in, for the match clock. */
    protected abstract MatchClock.Period matchPeriod();

    /** Subsystems are built; schedule default commands here. */
    protected void onInit() {}

    /** Runs after {@code readSensors()} on every init loop. Placement cards, warnings. */
    protected void onInitLoop() {}

    /** Runs once on START, after edge detection is reset, the clock has started and the drivetrain is in manual. */
    protected void onStart() {}

    /** Schedule commands and run safety checks. Runs on fresh sensor data. */
    protected void onDecide() {}

    /** Runs after actuators are written. Haptics, pose hand-off: anything that observes the result. */
    protected void onAfterAct() {}

    /** Add this OpMode's telemetry. Called every loop. */
    protected void onTelemetry() {}

    /** Runs first in {@code stop()}, while the robot is still live. */
    protected void onStop() {}

    // ---- Fixed lifecycle ----

    @Override
    public final void init() {
        // The scheduler is static and survives OpMode restarts: clear it before anything schedules.
        Scheduler.reset();
        robot = new Robot(hardwareMap);
        telemetry.setMsTransmissionInterval(TELEMETRY_INTERVAL_MS);
        onInit();
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
        onStart();
    }

    @Override
    public final void loop() {
        robot.readSensors();
        onDecide();
        Scheduler.execute();
        robot.writeActuators();
        onAfterAct();
        onTelemetry();
    }

    @Override
    public final void stop() {
        onStop();
        Scheduler.reset();
        if (robot != null) robot.stop();
    }
}
