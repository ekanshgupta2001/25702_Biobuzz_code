package org.firstinspires.ftc.teamcode.opmodes;

import com.pedropathing.ivy.Scheduler;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.util.field.Alliance;
import org.firstinspires.ftc.teamcode.util.time.MatchClock;

/**
 * The one OpMode base: a fixed lifecycle with hooks, used by the match OpModes and the pit benches
 * alike.
 *
 * <h2>The loop order is structural</h2>
 * {@link #loop()} is {@code final} and always runs observe, decide, execute, act, in that order:
 * <pre>
 * robot.readSensors();      // 1. observe  — one consistent snapshot for the whole loop
 * onDecide();               // 2. decide   — read gamepads, schedule commands
 * Scheduler.execute();      //    driver control and macros both run here
 * robot.writeActuators();   // 3. act      — every mechanism writes hardware
 * </pre>
 * A subclass cannot reorder it, which is the point: a decide step that ran after the act step would
 * see last loop's data, and the bug would look like a tuning problem.
 *
 * <h2>Benches share this, they do not duplicate it</h2>
 * A bench wants the same robot, the same config names and the same write path, but no command
 * scheduler — nothing else is running, so nothing re-asserts idle, and a bench that stops calling a
 * mechanism calls its {@code stop()}. It overrides {@link #usesScheduler()} to false and drives the
 * subsystems directly from {@link #onDecide()}. This used to be a second near-identical base class.
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

    /**
     * Which side this OpMode plays. Fixed per OpMode class ({@code BlueTeleop} / {@code RedTeleop}),
     * so it is chosen when the driver picks the OpMode and can never be stale or unconfirmed. Benches
     * return either.
     */
    protected abstract Alliance alliance();

    /** Which match period this OpMode runs in, for the match clock. */
    protected abstract MatchClock.Period matchPeriod();

    /** False for a bench: the loop then never calls {@code Scheduler.execute()}. */
    protected boolean usesScheduler() {
        return true;
    }

    /** Subsystems are built; schedule default commands here. */
    protected void onInit() {}

    /** Runs after {@code readSensors()} on every init loop. Placement cards, warnings. */
    protected void onInitLoop() {}

    /** Runs once on START, after edge detection is reset, the clock has started and the drivetrain is in manual. */
    protected void onStart() {}

    /** Read inputs and schedule commands, or drive a bench's mechanisms. Runs on fresh sensor data. */
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
        robot = new Robot(hardwareMap, alliance());
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
        if (usesScheduler()) Scheduler.execute();
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
