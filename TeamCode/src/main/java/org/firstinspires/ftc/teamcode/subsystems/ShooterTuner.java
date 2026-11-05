package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.VoltageSensor;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.internal.system.AppUtil;
import org.firstinspires.ftc.teamcode.commands.Shoot;

import java.io.BufferedWriter;
import java.io.File;
import java.io.FileWriter;
import java.io.IOException;
import java.math.BigDecimal;
import java.math.MathContext;
import java.util.ArrayList;
import java.util.List;
import java.util.Locale;
import java.util.function.DoubleConsumer;
import java.util.function.DoubleSupplier;

/**
 * Shooter Tuner: a tuning dashboard for both shooter wheels on the Driver Station. Keep hands clear.
 *
 * <pre>
 * Gamepad 1
 *   dpad up/down     pick a line            A  HOLD on/off: the wheels hold their targets and
 *   dpad left/right  change it                 every change you make applies at once
 *   LB / RB          smaller / bigger step  X  run the Test picked in the menu
 *                                           Y  apply the kS / kV the last FF FIT found
 *                                           B  STOP everything
 * </pre>
 *
 * Tuning order, one wheel at a time (the Wheels line), then both:
 * <ol>
 *   <li><b>Test = FF FIT, X.</b> Runs the wheel at five fixed powers and fits kS and kV to what it
 *       does. <b>Y</b> puts them in.</li>
 *   <li><b>A (HOLD).</b> The average error should already be small. Raise kP until "reach" stops
 *       getting shorter without the ripple growing. Bump the target +-50 to time it again.</li>
 *   <li><b>Test = STEP, X.</b> From rest: time to reach, time to settle, overshoot, settled error,
 *       and what to change.</li>
 *   <li><b>Test = SHOT, X.</b> Both wheels. Once it says READY, drop balls in by hand, one at a
 *       time: each dip and how long it took to recover is listed.</li>
 *   <li>Type the two lines at the bottom into {@code Shooter.java}. Values set here only last until
 *       the Robot Controller app restarts.</li>
 * </ol>
 *
 * Every test writes a CSV to /sdcard/FIRST/data (over USB: {@code adb pull /sdcard/FIRST/data}).
 * Builds only the {@link Shooter}: no intake, no up-wheels.
 */
@TeleOp(name = "Shooter Tuner", group = "Tuning")
public class ShooterTuner extends OpMode {
    /** Slower than this counts as stopped. */
    private static final double AT_REST_TICKS_PER_SEC = 20;
    private static final double MAX_WAIT_FOR_REST_MS = 5000;
    /** STEP's settled error, ripple and slope are measured over its last half second. */
    private static final double SETTLED_WINDOW_MS = 500;
    /** HOLD's average error and ripple are over this window. */
    private static final double LIVE_WINDOW_MS = 500;
    /** At or above this commanded power the motor is flat out (the hub clips at 1). */
    private static final double SATURATED = 0.98;

    private static final double[] FIT_POWERS = {0.2, 0.4, 0.6, 0.8, 1.0};
    private static final double FIT_STEP_MS = 1500;
    /** A fit step slower than this means the wheel was stuck, not turning, and is left out. */
    private static final double FIT_MIN_TICKS_PER_SEC = 50;

    private static final int MAX_SHOTS = 4;
    private static final double SHOT_IDLE_TIMEOUT_MS = 15000;
    /** A dip lasts at least this long, so a wheel the ball reaches a moment later is not missed. */
    private static final double MIN_DIP_MS = 100;
    private static final double DIP_TIMEOUT_MS = 2000;

    private static final double VOLTAGE_PERIOD_MS = 250;   // a voltage read is its own bus transaction
    private static final double SLOW_LOOP_MS = 25;

    private enum Test {
        STEP("STEP"), FF_FIT("FF FIT"), SHOT("SHOT (hand)");
        final String label;
        Test(String label) { this.label = label; }
    }

    private enum Wheels { BOTH, FLYWHEEL, ROLLER }

    private enum Mode { IDLE, HOLD, WAIT_FOR_REST, STEP, FF_FIT, SHOT }

    private Shooter shooter;
    private List<LynxModule> hubs;
    private VoltageSensor battery;
    private double volts;
    private double lastVoltageMs = -VOLTAGE_PERIOD_MS;

    private long startNs;
    private double nowMs;
    private final LoopTimer loopTimer = new LoopTimer();

    private final Gamepad cur = new Gamepad();
    private final Gamepad prev = new Gamepad();

    // The menu: two choice lines, then the number rows.
    private Test test = Test.FF_FIT;
    private Wheels wheels = Wheels.FLYWHEEL;
    private double runTimeSec = 3.0;
    private final List<Row> rows = new ArrayList<>();
    private int cursor = 0;

    private Mode mode = Mode.IDLE;
    private double modeStartMs;
    private String note = "";
    private final List<String> result = new ArrayList<>();
    private CsvLog log;

    private final Wheel fw = new Wheel("FW");
    private final Wheel rl = new Wheel("RL");

    private StepStats fwStep, rlStep;
    private FitStats fwFit, rlFit;
    private int fitIndex;
    private double fitStepStartMs;
    private boolean shotReady;
    private double shotIdleSinceMs;
    private double lowSinceMs = -1;
    private Dip dip;
    private final List<Dip> dips = new ArrayList<>();

    @Override
    public void init() {
        shooter = new Shooter(hardwareMap);
        // MANUAL: one bulk read per loop, cleared at the top of loop(), so every velocity read in a loop
        // (here and inside Shooter.update()) is the same snapshot, and costs one bus transaction.
        hubs = hardwareMap.getAll(LynxModule.class);
        for (LynxModule hub : hubs) hub.setBulkCachingMode(LynxModule.BulkCachingMode.MANUAL);
        battery = hardwareMap.voltageSensor.iterator().next();
        startNs = System.nanoTime();

        telemetry.setDisplayFormat(Telemetry.DisplayFormat.MONOSPACE);
        telemetry.setMsTransmissionInterval(100);

        rows.add(new Row("FW target", () -> Shooter.TARGET_TICKS_PER_SEC,
                v -> Shooter.TARGET_TICKS_PER_SEC = v, 1, 10, 50, 100, 250));
        rows.add(new Row("FW kS", () -> Shooter.kS, v -> Shooter.kS = v, 1, 0.0001, 0.001, 0.01, 0.1));
        rows.add(new Row("FW kV", () -> Shooter.kV, v -> Shooter.kV = v, 1, 0.000001, 0.00001, 0.0001));
        rows.add(new Row("FW kP", () -> Shooter.kP, v -> Shooter.kP = v, 1, 0.0001, 0.001, 0.01, 0.1));
        rows.add(new Row("RL target", () -> Shooter.ROLLER_TARGET_TICKS_PER_SEC,
                v -> Shooter.ROLLER_TARGET_TICKS_PER_SEC = v, 1, 10, 50, 100, 250));
        rows.add(new Row("RL kS", () -> Shooter.ROLLER_kS, v -> Shooter.ROLLER_kS = v, 1, 0.0001, 0.001, 0.01, 0.1));
        rows.add(new Row("RL kV", () -> Shooter.ROLLER_kV, v -> Shooter.ROLLER_kV = v, 1, 0.000001, 0.00001, 0.0001));
        rows.add(new Row("RL kP", () -> Shooter.ROLLER_kP, v -> Shooter.ROLLER_kP = v, 1, 0.0001, 0.001, 0.01, 0.1));
        rows.add(new Row("Run time s", () -> runTimeSec, v -> runTimeSec = Math.max(0.5, v), 0, 0.5, 1));
    }

    /** Before start: the menu works, nothing moves. */
    @Override
    public void init_loop() {
        beginLoop();
        handleMenu();
        telemetry.addLine("SHOOTER TUNER   press START to begin");
        telemetry.addLine("You can change values now.");
        addSettings();
        addPasteLines();
    }

    @Override
    public void start() {
        cur.copy(gamepad1);   // a button held through START is not a press
    }

    @Override
    public void loop() {
        // 1. One snapshot of the encoders, the clock and the battery.
        double dtMs = beginLoop();

        // 2. Buttons. B first, so STOP wins over anything else pressed in the same loop.
        if (pressed(cur.b, prev.b)) stopAll("Stopped (B).");
        handleMenu();
        if (pressed(cur.a, prev.a)) toggleHold();
        if (pressed(cur.x, prev.x)) startOrEndTest();
        if (pressed(cur.y, prev.y)) applyFit();

        // 3. The mode decides the targets (or the raw power) and moves its test along.
        switch (mode) {
            case HOLD:          setTargets(flywheelSelected(), rollerSelected()); break;
            case WAIT_FOR_REST: runWaitForRest(); break;
            case STEP:          runStep(dtMs); break;
            case FF_FIT:        runFit(); break;
            case SHOT:          runShot(); break;
            default:            setTargets(false, false); break;
        }

        // 4. Write. FF FIT writes raw power itself and update() would overwrite it.
        if (mode != Mode.FF_FIT) shooter.update();
        fw.power = shooter.getFlywheelPower();
        rl.power = shooter.getRollerPower();
        fw.measure(nowMs);
        rl.measure(nowMs);

        // 5. Record and show.
        if (log != null) log.row(nowMs - modeStartMs, dtMs, volts, fw, rl);
        addScreen();
    }

    @Override
    public void stop() {
        // The SDK zeroes the motors itself; an iterative OpMode may not write them here.
        closeLog();
    }

    // ------------------------------------------------------------------ every loop

    /** Clears the bulk cache, ticks the clock, samples the battery and both wheels. Returns the loop time. */
    private double beginLoop() {
        for (LynxModule hub : hubs) hub.clearBulkCache();
        nowMs = (System.nanoTime() - startNs) / 1e6;
        double dtMs = loopTimer.tick(nowMs);
        if (nowMs - lastVoltageMs >= VOLTAGE_PERIOD_MS) {
            volts = battery.getVoltage();
            lastVoltageMs = nowMs;
        }
        fw.velocity = shooter.getVelocity();
        rl.velocity = shooter.getRollerVelocity();
        prev.copy(cur);
        cur.copy(gamepad1);
        return dtMs;
    }

    private static boolean pressed(boolean now, boolean before) {
        return now && !before;
    }

    private boolean flywheelSelected() {
        return wheels != Wheels.ROLLER;
    }

    private boolean rollerSelected() {
        return wheels != Wheels.FLYWHEEL;
    }

    /** Closed loop at the targets in the menu; a wheel that is off gets target 0, which Shooter treats as off. */
    private void setTargets(boolean flywheelOn, boolean rollerOn) {
        fw.target = flywheelOn ? Shooter.TARGET_TICKS_PER_SEC : 0;
        rl.target = rollerOn ? Shooter.ROLLER_TARGET_TICKS_PER_SEC : 0;
        shooter.setTarget(fw.target);
        shooter.setRollerTarget(rl.target);
        if (flywheelOn || rollerOn) shooter.arm();
        else shooter.disarm();
    }

    private boolean testRunning() {
        return mode == Mode.WAIT_FOR_REST || mode == Mode.STEP || mode == Mode.FF_FIT;
    }

    // ------------------------------------------------------------------ buttons

    private void handleMenu() {
        int lines = 2 + rows.size();
        if (pressed(cur.dpad_up, prev.dpad_up)) cursor = (cursor + lines - 1) % lines;
        if (pressed(cur.dpad_down, prev.dpad_down)) cursor = (cursor + 1) % lines;
        if (testRunning()) return;   // the numbers a test is measuring stay put; B still stops it

        int change = pressed(cur.dpad_right, prev.dpad_right) ? 1
                : pressed(cur.dpad_left, prev.dpad_left) ? -1 : 0;
        if (change != 0) {
            if (cursor == 0) {
                test = Test.values()[(test.ordinal() + change + Test.values().length) % Test.values().length];
            } else if (cursor == 1) {
                wheels = Wheels.values()[(wheels.ordinal() + change + Wheels.values().length) % Wheels.values().length];
            } else {
                rows.get(cursor - 2).nudge(change);
            }
        }
        if (cursor >= 2) {
            if (pressed(cur.right_bumper, prev.right_bumper)) rows.get(cursor - 2).changeStep(1);
            if (pressed(cur.left_bumper, prev.left_bumper)) rows.get(cursor - 2).changeStep(-1);
        }
    }

    private void toggleHold() {
        if (mode == Mode.HOLD) {
            endMode();
            note = "";
        } else if (mode == Mode.IDLE) {
            mode = Mode.HOLD;
            note = "";
        } else {
            note = "A works when no test is running (B stops it).";
        }
    }

    private void startOrEndTest() {
        if (mode == Mode.SHOT) {
            finishShot("Ended (X).");
            return;
        }
        if (testRunning()) return;
        note = "";
        result.clear();
        switch (test) {
            case STEP:
                setTargets(false, false);
                mode = Mode.WAIT_FOR_REST;
                modeStartMs = nowMs;
                break;
            case FF_FIT:
                startFit();
                break;
            case SHOT:
                startShot();
                break;
        }
    }

    /** B: whatever is running ends now. The last result stays on screen. */
    private void stopAll(String why) {
        if (mode == Mode.SHOT) {
            finishShot(why);
            return;
        }
        if (testRunning()) result.add(why);
        else note = why;
        endMode();
    }

    /** The end of every test and of HOLD: wheels off (they coast), CSV closed, back to IDLE. */
    private void endMode() {
        shooter.disarm();
        fw.target = 0;
        rl.target = 0;
        mode = Mode.IDLE;
        closeLog();
    }

    private void applyFit() {
        if (testRunning() || mode == Mode.SHOT) {
            note = "Y works once the test is over.";
            return;
        }
        StringBuilder applied = new StringBuilder();
        if (fwFit != null && fwFit.ok()) {
            Shooter.kS = fwFit.kS;
            Shooter.kV = fwFit.kV;
            applied.append(String.format(Locale.US, "FW kS %s kV %s  ", plain(fwFit.kS), plain(fwFit.kV)));
        }
        if (rlFit != null && rlFit.ok()) {
            Shooter.ROLLER_kS = rlFit.kS;
            Shooter.ROLLER_kV = rlFit.kV;
            applied.append(String.format(Locale.US, "RL kS %s kV %s", plain(rlFit.kS), plain(rlFit.kV)));
        }
        note = applied.length() == 0 ? "No fit to apply: run Test = FF FIT first." : "Applied " + applied;
    }

    // ------------------------------------------------------------------ STEP

    /** A floating wheel coasts for a while; time the spin-up from rest, not from a roll. */
    private void runWaitForRest() {
        setTargets(false, false);
        boolean fwMoving = flywheelSelected() && Math.abs(fw.velocity) > AT_REST_TICKS_PER_SEC;
        boolean rlMoving = rollerSelected() && Math.abs(rl.velocity) > AT_REST_TICKS_PER_SEC;
        if ((fwMoving || rlMoving) && nowMs - modeStartMs < MAX_WAIT_FOR_REST_MS) return;

        fwStep = flywheelSelected() ? new StepStats("FW", Shooter.TARGET_TICKS_PER_SEC, Shooter.kS, Shooter.kV, fw.velocity) : null;
        rlStep = rollerSelected() ? new StepStats("RL", Shooter.ROLLER_TARGET_TICKS_PER_SEC, Shooter.ROLLER_kS, Shooter.ROLLER_kV, rl.velocity) : null;
        mode = Mode.STEP;
        modeStartMs = nowMs;
        openLog("step");
        setTargets(flywheelSelected(), rollerSelected());
    }

    private void runStep(double dtMs) {
        setTargets(flywheelSelected(), rollerSelected());
        double t = nowMs - modeStartMs;
        double runMs = runTimeSec * 1000;
        boolean settling = t > runMs - SETTLED_WINDOW_MS;
        // fw.power is what was sent last loop, which is the power behind this loop's velocity.
        if (fwStep != null) fwStep.add(t, dtMs, fw.velocity, fw.power, settling);
        if (rlStep != null) rlStep.add(t, dtMs, rl.velocity, rl.power, settling);
        if (t < runMs) return;

        if (fwStep != null) result.addAll(fwStep.summary());
        if (rlStep != null) result.addAll(rlStep.summary());
        if (fwStep != null && rlStep != null) {
            double both = Math.max(fwStep.reachMs, rlStep.reachMs);
            result.add(fwStep.reachMs < 0 || rlStep.reachMs < 0 ? "Both never at target."
                    : String.format(Locale.US, "Both at target by %.0f ms.", both));
        }
        addLogPath();
        endMode();
    }

    // ------------------------------------------------------------------ FF FIT

    private void startFit() {
        fwFit = flywheelSelected() ? new FitStats("FW", Shooter.TARGET_TICKS_PER_SEC) : null;
        rlFit = rollerSelected() ? new FitStats("RL", Shooter.ROLLER_TARGET_TICKS_PER_SEC) : null;
        fitIndex = 0;
        fitStepStartMs = nowMs;
        mode = Mode.FF_FIT;
        modeStartMs = nowMs;
        fw.target = 0;
        rl.target = 0;
        openLog("fit");
    }

    private void runFit() {
        double power = FIT_POWERS[fitIndex];
        shooter.setRawPower(fwFit != null ? power : 0, rlFit != null ? power : 0);
        double t = nowMs - fitStepStartMs;
        if (t >= FIT_STEP_MS - SETTLED_WINDOW_MS) {   // average only the end of each step, once it has levelled off
            if (fwFit != null) fwFit.sample(fw.velocity);
            if (rlFit != null) rlFit.sample(rl.velocity);
        }
        if (t < FIT_STEP_MS) return;

        if (fwFit != null) fwFit.endStep(fitIndex);
        if (rlFit != null) rlFit.endStep(fitIndex);
        fitIndex++;
        fitStepStartMs = nowMs;
        if (fitIndex < FIT_POWERS.length) return;

        if (fwFit != null) result.addAll(fwFit.finish(volts));
        if (rlFit != null) result.addAll(rlFit.finish(volts));
        if ((fwFit != null && fwFit.ok()) || (rlFit != null && rlFit.ok())) result.add("Y = apply kS and kV.");
        addLogPath();
        endMode();
    }

    // ------------------------------------------------------------------ SHOT (hand feed)

    private void startShot() {
        if (Shooter.TARGET_TICKS_PER_SEC <= 0 || Shooter.ROLLER_TARGET_TICKS_PER_SEC <= 0) {
            note = "SHOT needs both targets above 0.";
            return;
        }
        dips.clear();
        dip = null;
        shotReady = false;
        lowSinceMs = -1;
        shotIdleSinceMs = nowMs;
        mode = Mode.SHOT;
        modeStartMs = nowMs;
        openLog("shot");
        setTargets(true, true);
    }

    private void runShot() {
        setTargets(true, true);   // a shot needs both wheels, whatever the Wheels line says
        if (dip == null) {
            if (!shotReady) {
                shotReady = fw.inBand() && rl.inBand();
                if (shotReady) shotIdleSinceMs = nowMs;
            } else if (fw.isLow() || rl.isLow()) {
                // Low two loops running is a ball; one stray sample is ripple.
                if (lowSinceMs < 0) lowSinceMs = nowMs;
                else dip = new Dip(lowSinceMs);
            } else {
                lowSinceMs = -1;
            }
            if (dip == null) {
                if (nowMs - shotIdleSinceMs > SHOT_IDLE_TIMEOUT_MS) {
                    finishShot(shotReady ? "No ball for 15 s."
                            : "Never READY in 15 s: get HOLD on target first.");
                }
                return;
            }
        }

        double t = nowMs - dip.startMs;
        dip.fw.add(t, fw);
        dip.rl.add(t, rl);
        boolean over = t >= MIN_DIP_MS && dip.fw.recovered() && dip.rl.recovered();
        if (!over && t < DIP_TIMEOUT_MS) return;

        dips.add(dip);
        result.add(dip.line(dips.size(), Shooter.TARGET_TICKS_PER_SEC, Shooter.ROLLER_TARGET_TICKS_PER_SEC));
        shotReady = over;   // a wheel that never came back must be READY again before the next ball counts
        dip = null;
        lowSinceMs = -1;
        shotIdleSinceMs = nowMs;
        if (dips.size() >= MAX_SHOTS) finishShot("");
    }

    private void finishShot(String why) {
        if (!why.isEmpty()) result.add(why);
        double worst = -1;
        boolean saturated = false;
        boolean notBack = false;
        for (Dip d : dips) {
            worst = Math.max(worst, Math.max(d.fw.recoveryMs(), d.rl.recoveryMs()));
            saturated |= d.fw.saturated || d.rl.saturated;
            notBack |= !d.fw.recovered() || !d.rl.recovered();
        }
        if (dips.isEmpty()) {
            result.add("No dips seen. Drop the ball into the wheels once it says READY.");
        } else if (notBack) {
            result.add("-> A wheel was not back in 2 s: it can't hold this target. Lower it or check FF FIT.");
        } else if (worst > Shoot.RECOVER_TIMEOUT_MS) {
            result.add(String.format(Locale.US, "-> Slowest %.0f ms > match wait %d ms. %s", worst,
                    Shoot.RECOVER_TIMEOUT_MS, saturated ? "It hit full power: lower the target." : "Raise kP."));
        } else {
            result.add(String.format(Locale.US, "-> Slowest recovery %.0f ms, inside the match's %d ms.",
                    worst, Shoot.RECOVER_TIMEOUT_MS));
        }
        addLogPath();
        dip = null;
        endMode();
    }

    // ------------------------------------------------------------------ CSV

    private void openLog(String tag) {
        closeLog();
        log = CsvLog.open(tag, String.format(Locale.US,
                "# kS=%s kV=%s kP=%s ROLLER_kS=%s ROLLER_kV=%s ROLLER_kP=%s fw_target=%.0f rl_target=%.0f tolerance=%.0f battery=%.2fV",
                plain(Shooter.kS), plain(Shooter.kV), plain(Shooter.kP),
                plain(Shooter.ROLLER_kS), plain(Shooter.ROLLER_kV), plain(Shooter.ROLLER_kP),
                Shooter.TARGET_TICKS_PER_SEC, Shooter.ROLLER_TARGET_TICKS_PER_SEC,
                Shooter.TOLERANCE_TICKS_PER_SEC, volts));
    }

    private void addLogPath() {
        result.add(log == null ? "(CSV could not be opened)" : "CSV " + log.path);
    }

    private void closeLog() {
        if (log != null) log.close();
        log = null;
    }

    // ------------------------------------------------------------------ the screen

    private void addScreen() {
        telemetry.addLine(String.format(Locale.US, "SHOOTER TUNER %-7s %5.2fV loop %.0f/%.0fms",
                mode == Mode.WAIT_FOR_REST ? "STEP" : mode.name().replace('_', ' '),
                volts, loopTimer.avgMs, loopTimer.maxMs));
        addWarnings();
        addLive(fw);
        addLive(rl);
        telemetry.addLine(status());
        addSettings();
        if (!result.isEmpty()) {
            telemetry.addLine("-- result -------------------------------");
            for (String line : result) telemetry.addLine(line);
        }
        String next = nextStep();
        if (!next.isEmpty()) telemetry.addLine(next);
        addPasteLines();
        telemetry.addLine("A hold  X run test  Y apply fit  B STOP");
    }

    private void addWarnings() {
        if (loopTimer.avgMs > SLOW_LOOP_MS) {
            telemetry.addLine(String.format(Locale.US, "!! loop %.0f ms: the control is lagging", loopTimer.avgMs));
        }
        addWheelWarnings(fw, rl, "sl", "sr");
        addWheelWarnings(rl, fw, "sr", "sl");
    }

    private void addWheelWarnings(Wheel w, Wheel other, String port, String otherPort) {
        if (w.noEncoder) {
            boolean swapped = other.power == 0 && Math.abs(other.velocity) > 100;
            telemetry.addLine(swapped
                    ? String.format(Locale.US, "!! %s powered, reads 0; %s reads %.0f unpowered: encoder cables swapped (%s/%s)?",
                            w.name, other.name, other.velocity, port, otherPort)
                    : String.format(Locale.US, "!! %s powered but reads 0: encoder cable unplugged or in the wrong port", w.name));
        }
        if (w.backwards) {
            telemetry.addLine(String.format(Locale.US, "!! %s turns backwards: motor or encoder direction reversed", w.name));
        }
    }

    private void addLive(Wheel w) {
        if (w.target > 0) {
            telemetry.addLine(String.format(Locale.US, "%s %5.0f / %-5.0f err %+5.0f  %s",
                    w.name, w.velocity, w.target, w.target - w.velocity, w.inBand() ? "READY" : ""));
            telemetry.addLine(String.format(Locale.US, "   pwr %s  avg %s  rip %s  reach %s",
                    powerText(w.power), dash("%+.0f", w.window.avg), dash("%.0f", w.window.ripple),
                    w.reachMs < 0 ? "--" : String.format(Locale.US, "%.0fms", w.reachMs)));
        } else {
            telemetry.addLine(String.format(Locale.US, "%s %5.0f t/s   %s", w.name, w.velocity,
                    w.power == 0 ? "off" : "raw power"));
            telemetry.addLine("   pwr " + powerText(w.power));
        }
    }

    private String status() {
        double t = (nowMs - modeStartMs) / 1000;
        switch (mode) {
            case HOLD:
                return "HOLD: change values live. " + note;
            case WAIT_FOR_REST:
                return String.format(Locale.US, "STEP: waiting for the wheels to stop (%.1f s)", t);
            case STEP:
                return String.format(Locale.US, "STEP: %.1f / %.1f s", t, runTimeSec);
            case FF_FIT:
                return String.format(Locale.US, "FF FIT: step %d/%d, power %.1f, %.1f s", fitIndex + 1,
                        FIT_POWERS.length, FIT_POWERS[Math.min(fitIndex, FIT_POWERS.length - 1)],
                        (nowMs - fitStepStartMs) / 1000);
            case SHOT:
                if (dip != null) return String.format(Locale.US, "SHOT: ball %d, recovering %.0f ms",
                        dips.size() + 1, nowMs - dip.startMs);
                return shotReady
                        ? String.format(Locale.US, "SHOT: READY, drop ball %d of %d in (X = end)", dips.size() + 1, MAX_SHOTS)
                        : "SHOT: spinning up, wait for READY";
            default:
                return note.isEmpty() ? "IDLE" : note;
        }
    }

    private void addSettings() {
        telemetry.addLine(testRunning() ? "-- settings (locked while testing) ------"
                : "-- settings (dpad, LB/RB = step) -------");
        telemetry.addLine(menuLine(0, "Test", test.label));
        telemetry.addLine(menuLine(1, "Wheels", wheels.name()));
        for (int i = 0; i < rows.size(); i++) {
            Row row = rows.get(i);
            telemetry.addLine(menuLine(i + 2, row.label, String.format(Locale.US, "%-10s +-%s",
                    plain(row.get.getAsDouble()), plain(row.step()))));
        }
        telemetry.addLine("  FF power at target: FW " + feedforwardCheck(Shooter.kS, Shooter.kV, Shooter.TARGET_TICKS_PER_SEC)
                + "  RL " + feedforwardCheck(Shooter.ROLLER_kS, Shooter.ROLLER_kV, Shooter.ROLLER_TARGET_TICKS_PER_SEC));
    }

    private String menuLine(int index, String label, String value) {
        return String.format(Locale.US, "%s %-10s %s", index == cursor ? ">" : " ", label, value);
    }

    /** What kV and kS alone would send at the target. At 1 or more the wheel can't get there. */
    private static String feedforwardCheck(double kS, double kV, double target) {
        if (kV == 0) return "kV=0!";
        double ff = kS + kV * target;
        return String.format(Locale.US, "%.2f%s", ff, ff >= 1 ? " >1!" : "");
    }

    private String nextStep() {
        if (mode != Mode.IDLE && mode != Mode.HOLD) return "";
        boolean fwOn = flywheelSelected(), rlOn = rollerSelected();
        if ((fwOn && Shooter.kV == 0) || (rlOn && Shooter.ROLLER_kV == 0)) {
            return "Next: Test = FF FIT, X. It finds kS and kV.";
        }
        if (mode == Mode.HOLD) return "Next: raise kP until reach stops improving.";
        if ((fwOn && Shooter.kP == 0) || (rlOn && Shooter.ROLLER_kP == 0)) return "Next: A (HOLD) and raise kP.";
        return "Next: STEP, then SHOT, then copy the lines below.";
    }

    private void addPasteLines() {
        telemetry.addLine("-- Shooter.java (public static double) --");
        telemetry.addLine(String.format(Locale.US, "kS = %s, kV = %s, kP = %s;",
                plain(Shooter.kS), plain(Shooter.kV), plain(Shooter.kP)));
        telemetry.addLine(String.format(Locale.US, "ROLLER_kS = %s, ROLLER_kV = %s, ROLLER_kP = %s;",
                plain(Shooter.ROLLER_kS), plain(Shooter.ROLLER_kV), plain(Shooter.ROLLER_kP)));
    }

    // ------------------------------------------------------------------ formatting

    /** 0.00039 rather than 3.9E-4, and 1300 rather than 1300.0. */
    private static String plain(double value) {
        return BigDecimal.valueOf(value).stripTrailingZeros().toPlainString();
    }

    private static double threeFigures(double value) {
        return new BigDecimal(value).round(new MathContext(3)).doubleValue();
    }

    private static String dash(String format, double value) {
        return Double.isNaN(value) ? "--" : String.format(Locale.US, format, value);
    }

    /** The power the motor actually gets (the hub clips at 1), and SAT when it is flat out. */
    private static String powerText(double commanded) {
        double clipped = Math.max(-1, Math.min(1, commanded));
        return String.format(Locale.US, "%.2f%s", clipped, Math.abs(commanded) >= SATURATED ? " SAT" : "");
    }

    // ------------------------------------------------------------------ helpers

    /** One number in the menu: left/right nudges it by the step, LB/RB pick the step. Never below 0. */
    private static final class Row {
        final String label;
        final DoubleSupplier get;
        final DoubleConsumer set;
        private final double[] steps;
        private int stepIndex;

        Row(String label, DoubleSupplier get, DoubleConsumer set, int stepIndex, double... steps) {
            this.label = label;
            this.get = get;
            this.set = set;
            this.steps = steps;
            this.stepIndex = stepIndex;
        }

        double step() {
            return steps[stepIndex];
        }

        /** In decimal, so 0.00039 + 0.00001 is 0.0004 and not 0.00039999999. */
        void nudge(int direction) {
            BigDecimal next = BigDecimal.valueOf(get.getAsDouble())
                    .add(BigDecimal.valueOf(step()).multiply(BigDecimal.valueOf(direction)));
            set.accept(Math.max(0, next.doubleValue()));
        }

        void changeStep(int direction) {
            stepIndex = Math.max(0, Math.min(steps.length - 1, stepIndex + direction));
        }
    }

    /** One wheel, every loop: what it is doing, the live average/ripple, the reach timer and the wiring checks. */
    private static final class Wheel {
        final String name;
        double target;     // closed-loop target this loop; 0 = off (or raw power in FF FIT)
        double velocity;   // ticks/sec, from this loop's bulk read
        double power;      // last power sent; can be above 1
        final Window window = new Window();
        double reachMs = -1;
        boolean noEncoder, backwards;

        private double lastTarget = Double.NaN;
        private double reachFromMs;
        private double noEncoderSinceMs = -1, backwardsSinceMs = -1;

        Wheel(String name) {
            this.name = name;
        }

        boolean inBand() {
            return target > 0 && Math.abs(target - velocity) < Shooter.TOLERANCE_TICKS_PER_SEC;
        }

        /** Below the band: a ball is slowing it down. */
        boolean isLow() {
            return target > 0 && target - velocity >= Shooter.TOLERANCE_TICKS_PER_SEC;
        }

        void measure(double nowMs) {
            // A new target (HOLD starting, or a +-50 bump) restarts the reach timer: a live step response.
            if (target != lastTarget) {
                lastTarget = target;
                reachFromMs = nowMs;
                reachMs = -1;
                window.clear();
            }
            if (target > 0) {
                window.add(nowMs, target - velocity);
                if (reachMs < 0 && inBand()) reachMs = nowMs - reachFromMs;
            }

            double p = Math.max(-1, Math.min(1, power));
            // Pushed hard but not turning: the encoder is not reaching the hub.
            boolean dead = Math.abs(p) > 0.3 && Math.abs(velocity) < AT_REST_TICKS_PER_SEC;
            noEncoderSinceMs = dead ? (noEncoderSinceMs < 0 ? nowMs : noEncoderSinceMs) : -1;
            noEncoder = noEncoderSinceMs >= 0 && nowMs - noEncoderSinceMs > 500;
            // Forward power, backward speed. (Negative power on a forward wheel is just kP braking.)
            boolean reversed = p > 0.3 && velocity < -FIT_MIN_TICKS_PER_SEC;
            backwardsSinceMs = reversed ? (backwardsSinceMs < 0 ? nowMs : backwardsSinceMs) : -1;
            backwards = backwardsSinceMs >= 0 && nowMs - backwardsSinceMs > 300;
        }
    }

    /** Average and max - min over consecutive half-second windows. NaN until the first one ends. */
    private static final class Window {
        double avg = Double.NaN, ripple = Double.NaN;
        private double startMs = -1, sum, min, max;
        private int n;

        void add(double nowMs, double value) {
            if (startMs < 0) restart(nowMs);
            sum += value;
            n++;
            min = Math.min(min, value);
            max = Math.max(max, value);
            if (nowMs - startMs >= LIVE_WINDOW_MS) {
                avg = sum / n;
                ripple = max - min;
                restart(nowMs);
            }
        }

        void clear() {
            startMs = -1;
            avg = Double.NaN;
            ripple = Double.NaN;
        }

        private void restart(double nowMs) {
            startMs = nowMs;
            sum = 0;
            n = 0;
            min = Double.MAX_VALUE;
            max = -Double.MAX_VALUE;
        }
    }

    /** One wheel's numbers over a STEP run, from rest. */
    private static final class StepStats {
        final String name;
        final double target;
        final double kS, kV;
        final double startVelocity;
        double reachMs = -1;

        private double lastOutOfBandMs = 0;
        private boolean inBandAtEnd;
        private double peak = -Double.MAX_VALUE;
        private double saturatedMs = 0;
        // The settled window: error mean and spread, power, and a least-squares slope of velocity (t in seconds).
        private int n;
        private double eSum, eeSum, pSum, vSum, tSum, ttSum, tvSum;

        StepStats(String name, double target, double kS, double kV, double startVelocity) {
            this.name = name;
            this.target = target;
            this.kS = kS;
            this.kV = kV;
            this.startVelocity = startVelocity;
        }

        void add(double tMs, double dtMs, double v, double power, boolean settling) {
            peak = Math.max(peak, v);
            boolean in = Math.abs(target - v) < Shooter.TOLERANCE_TICKS_PER_SEC;
            if (in && reachMs < 0) reachMs = tMs;
            if (!in) lastOutOfBandMs = tMs;
            inBandAtEnd = in;
            if (Math.abs(power) >= SATURATED) saturatedMs += dtMs;
            if (settling) {
                double t = tMs / 1000, e = target - v;
                n++;
                eSum += e;
                eeSum += e * e;
                pSum += Math.max(-1, Math.min(1, power));
                vSum += v;
                tSum += t;
                ttSum += t * t;
                tvSum += t * v;
            }
        }

        private double slope() {
            double denominator = n * ttSum - tSum * tSum;
            return n < 3 || denominator == 0 ? 0 : (n * tvSum - tSum * vSum) / denominator;
        }

        List<String> summary() {
            List<String> lines = new ArrayList<>();
            String fromRest = Math.abs(startVelocity) > AT_REST_TICKS_PER_SEC
                    ? String.format(Locale.US, " (started at %.0f, not rest)", startVelocity) : "";
            lines.add(String.format(Locale.US, "%s STEP to %.0f%s", name, target, fromRest));
            lines.add(String.format(Locale.US, "   reach %s  settle %s",
                    reachMs < 0 ? "never" : String.format(Locale.US, "%.0fms", reachMs),
                    reachMs >= 0 && inBandAtEnd ? String.format(Locale.US, "%.0fms", lastOutOfBandMs) : "never"));
            lines.add(String.format(Locale.US, "   peak %.0f (%+.0f)  full power %.0fms", peak, peak - target, saturatedMs));
            if (n > 0) {
                double mean = eSum / n;
                double spread = Math.sqrt(Math.max(0, eeSum / n - mean * mean));
                lines.add(String.format(Locale.US, "   settled err %+.0f +-%.0f  pwr %.2f", mean, spread, pSum / n));
            }
            lines.add("   -> " + diagnosis());
            return lines;
        }

        private String diagnosis() {
            if (n == 0) return "No settled samples: the run was cut short or the loop is too slow.";
            double vSettled = vSum / n, pSettled = pSum / n, error = target - vSettled;
            double tolerance = Shooter.TOLERANCE_TICKS_PER_SEC, slope = slope();
            if (vSettled * target < 0) return "Turns backwards: motor or encoder direction reversed.";
            if (pSettled >= SATURATED) {
                return String.format(Locale.US, "Flat out at ~%.0f. Gains can't fix that: lower the target, "
                        + "check friction and battery, run FF FIT.", vSettled);
            }
            if (error > tolerance && slope > tolerance) {
                return String.format(Locale.US, "Still speeding up at the end (%+.0f t/s/s): raise kP, "
                        + "or a longer Run time.", slope);
            }
            double kVFit = vSettled == 0 ? 0 : (pSettled - kS) / vSettled;
            if (error > tolerance) {
                return String.format(Locale.US, "Settled %.0f LOW with power to spare: kV too low. Try kV %s.",
                        error, plain(threeFigures(kVFit)));
            }
            if (error < -tolerance) {
                return String.format(Locale.US, "Settled %.0f HIGH: kV too high. Try kV %s.",
                        -error, plain(threeFigures(kVFit)));
            }
            if (peak - target > 2 * tolerance) {
                return String.format(Locale.US, "On target, but overshot %.0f: lower kP a little.", peak - target);
            }
            return "On target. Next: SHOT.";
        }
    }

    /** One wheel's FF FIT: average speed at each power, then power = kS + kV * speed by least squares. */
    private static final class FitStats {
        final String name;
        final double target;
        final double[] speeds = new double[FIT_POWERS.length];
        double kS = Double.NaN, kV = Double.NaN;

        private double sum;
        private int n;

        FitStats(String name, double target) {
            this.name = name;
            this.target = target;
        }

        boolean ok() {
            return !Double.isNaN(kV);
        }

        void sample(double v) {
            sum += v;
            n++;
        }

        void endStep(int index) {
            speeds[index] = n == 0 ? 0 : sum / n;
            sum = 0;
            n = 0;
        }

        List<String> finish(double volts) {
            List<String> lines = new ArrayList<>();
            StringBuilder table = new StringBuilder("   t/s");
            for (double s : speeds) table.append(String.format(Locale.US, " %.0f", s));
            double fullSpeed = speeds[speeds.length - 1];

            if (fullSpeed < -FIT_MIN_TICKS_PER_SEC) {
                lines.add(name + " FIT: turns backwards. Motor or encoder direction reversed.");
                lines.add(table.toString());
                return lines;
            }

            int m = 0;
            double sv = 0, sp = 0, svv = 0, svp = 0;
            for (int i = 0; i < speeds.length; i++) {
                if (speeds[i] < FIT_MIN_TICKS_PER_SEC) continue;
                double v = speeds[i], p = FIT_POWERS[i];
                m++;
                sv += v;
                sp += p;
                svv += v * v;
                svp += v * p;
            }
            double denominator = m * svv - sv * sv;
            double slope = m >= 2 && denominator > 0 ? (m * svp - sv * sp) / denominator : 0;
            if (slope <= 0) {
                lines.add(name + " FIT: the wheel barely turned. Check the motor, its power cable and encoder cable.");
                lines.add(table.toString());
                return lines;
            }
            double intercept = (sp - slope * sv) / m;
            double meanPower = sp / m, ssTotal = 0, ssResidual = 0;
            for (int i = 0; i < speeds.length; i++) {
                if (speeds[i] < FIT_MIN_TICKS_PER_SEC) continue;
                double p = FIT_POWERS[i];
                ssResidual += Math.pow(p - (intercept + slope * speeds[i]), 2);
                ssTotal += Math.pow(p - meanPower, 2);
            }
            double r2 = ssTotal == 0 ? 1 : 1 - ssResidual / ssTotal;

            kV = threeFigures(slope);
            kS = threeFigures(Math.max(0, intercept));
            lines.add(String.format(Locale.US, "%s FIT  kS %s  kV %s  R2 %.3f", name, plain(kS), plain(kV), r2));
            lines.add("   pwr  .2 .4 .6 .8 1.0");
            lines.add(table.toString());
            lines.add(String.format(Locale.US, "   full power %.0f t/s @ %.2fV", fullSpeed, volts));
            if (fullSpeed > 0) {
                double percent = 100 * target / fullSpeed;
                lines.add(String.format(Locale.US, "   target %.0f = %.0f%% of that%s", target, percent,
                        percent > 90 ? "  !! no room to recover" : ""));
            }
            if (r2 < 0.95) lines.add("   !! poor fit: a wheel rubbing or a belt slipping?");
            if (m < speeds.length) lines.add(String.format(Locale.US, "   (%d low step(s) left out: wheel didn't turn)", speeds.length - m));
            return lines;
        }
    }

    /**
     * One ball through the wheels, timed from the moment the first wheel dropped below its band. Each
     * wheel's recovery is measured from that same moment: how long after the ball it was ready again.
     */
    private static final class Dip {
        final double startMs;
        final WheelDip fw = new WheelDip(), rl = new WheelDip();

        Dip(double startMs) {
            this.startMs = startMs;
        }

        String line(int number, double fwTarget, double rlTarget) {
            return String.format(Locale.US, "#%d FW %s | RL %s", number, fw.text(fwTarget), rl.text(rlTarget));
        }
    }

    private static final class WheelDip {
        double lowest = Double.MAX_VALUE;
        boolean saturated;
        private boolean left;
        private double backMs = -1;

        void add(double tMs, Wheel w) {
            lowest = Math.min(lowest, w.velocity);
            if (Math.abs(w.power) >= SATURATED) saturated = true;
            if (!left) {
                left = !w.inBand();
            } else if (backMs < 0 && w.inBand()) {
                backMs = tMs;
            }
        }

        boolean recovered() {
            return !left || backMs >= 0;
        }

        /** 0 for a wheel that never left its band. */
        double recoveryMs() {
            return left && backMs >= 0 ? backMs : 0;
        }

        String text(double target) {
            String drop = String.format(Locale.US, "-%.0f", Math.max(0, target - lowest));
            if (!left) return drop + " ok";
            return drop + (backMs < 0 ? " NOT back" : String.format(Locale.US, " %.0fms", backMs))
                    + (saturated ? " SAT" : "");
        }
    }

    /** Loop time: average and worst over each second. */
    private static final class LoopTimer {
        double avgMs, maxMs;
        private double lastMs = -1, windowStartMs = 0, sum, worst;
        private int n;

        double tick(double nowMs) {
            double dt = lastMs < 0 ? 0 : nowMs - lastMs;
            lastMs = nowMs;
            if (dt > 0) {
                sum += dt;
                n++;
                worst = Math.max(worst, dt);
            }
            if (nowMs - windowStartMs >= 1000 && n > 0) {
                avgMs = sum / n;
                maxMs = worst;
                windowStartMs = nowMs;
                sum = 0;
                worst = 0;
                n = 0;
            }
            return dt;
        }
    }

    /** One CSV per test. A file that will not open does not stop the test. */
    private static final class CsvLog {
        final String path;
        private final BufferedWriter out;

        private CsvLog(File file, String header) throws IOException {
            path = file.getAbsolutePath();
            out = new BufferedWriter(new FileWriter(file));
            out.write(header + "\n");
            out.write("t_ms,loop_ms,battery_v,fw_target,fw_vel,fw_power,rl_target,rl_vel,rl_power\n");
        }

        static CsvLog open(String tag, String header) {
            try {
                File dir = AppUtil.ROBOT_DATA_DIR;
                //noinspection ResultOfMethodCallIgnored
                dir.mkdirs();
                return new CsvLog(new File(dir, "shooter_" + tag + "_" + System.currentTimeMillis() + ".csv"), header);
            } catch (IOException e) {
                return null;
            }
        }

        void row(double tMs, double loopMs, double volts, Wheel fw, Wheel rl) {
            try {
                out.write(String.format(Locale.US, "%.1f,%.1f,%.2f,%.0f,%.1f,%.3f,%.0f,%.1f,%.3f\n",
                        tMs, loopMs, volts, fw.target, fw.velocity, fw.power, rl.target, rl.velocity, rl.power));
            } catch (IOException ignored) {
                // a missed row is not worth stopping a test for
            }
        }

        void close() {
            try {
                out.close();
            } catch (IOException ignored) {
                // nothing left to do with it
            }
        }
    }
}
