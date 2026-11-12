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
 * Shooter Tuner: a tuning dashboard for the shooter flywheel on the Driver Station. Keep hands clear.
 *
 * <pre>
 * Gamepad 1
 *   dpad up/down     pick a line            A  HOLD on/off: the wheel holds the target and
 *   dpad left/right  change it                 every change you make applies at once
 *   LB / RB          smaller / bigger step  X  run the Test picked in the menu
 *                                           Y  apply the kS / kV the last FF FIT found
 *                                           B  STOP everything
 * </pre>
 *
 * Tuning order:
 * <ol>
 *   <li><b>Test = FF FIT, X.</b> Runs the wheel at five fixed powers and fits kS and kV to what it
 *       does. <b>Y</b> puts them in.</li>
 *   <li><b>A (HOLD).</b> The average error should already be small. Raise kP until "reach" stops
 *       getting shorter without the ripple growing. Bump the target +-50 to time it again.</li>
 *   <li><b>Test = STEP, X.</b> From rest: time to reach, time to settle, overshoot, settled error,
 *       and what to change.</li>
 *   <li><b>Test = SHOT, X.</b> Once it says READY, drop balls in by hand, one at a time: each dip
 *       and how long it took to recover is listed.</li>
 *   <li>Type the line at the bottom into {@code Shooter.java}. Values set here only last until the
 *       Robot Controller app restarts.</li>
 * </ol>
 *
 * The menu is locked while a test runs (STEP and FF FIT wait for the wheel to stop first). While B is
 * held no other button counts. Every test writes a CSV to /sdcard/FIRST/data (over USB:
 * {@code adb pull /sdcard/FIRST/data}). Builds only the {@link Shooter}, nothing else.
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
    /** In tolerance this long before SHOT calls it READY. */
    private static final double READY_DWELL_MS = 150;
    /** A dip lasts at least this long: an in-band reading just after the ball hits is ripple, not recovery. */
    private static final double MIN_DIP_MS = 100;
    private static final double DIP_TIMEOUT_MS = 2000;

    private static final double VOLTAGE_PERIOD_MS = 250;   // a voltage read is its own bus transaction
    private static final double SLOW_LOOP_MS = 25;

    private enum Test {
        STEP("STEP"), FF_FIT("FF FIT"), SHOT("SHOT (hand)");
        final String label;
        Test(String label) { this.label = label; }
    }

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

    // The menu: the Test line, then the number rows.
    private Test test = Test.FF_FIT;
    private double runTimeSec = 3.0;
    private final List<Row> rows = new ArrayList<>();
    private int cursor = 0;

    private Mode mode = Mode.IDLE;
    private Test pendingTest = Test.STEP;   // what WAIT_FOR_REST starts once the wheel has stopped
    private double modeStartMs;
    private String note = "";
    private final List<String> result = new ArrayList<>();
    private CsvLog log;

    private final Wheel wheel = new Wheel();

    private StepStats step;
    private FitStats fit;
    private int fitIndex;
    private double fitStepStartMs;
    private boolean shotReady;
    private double readySinceMs = -1;
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

        rows.add(new Row("Target", () -> Shooter.TARGET_TICKS_PER_SEC,
                v -> Shooter.TARGET_TICKS_PER_SEC = v, 1, 10, 50, 100, 250));
        rows.add(new Row("kS", () -> Shooter.kS, v -> Shooter.kS = v, 1, 0.0001, 0.001, 0.01, 0.1));
        rows.add(new Row("kV", () -> Shooter.kV, v -> Shooter.kV = v, 1, 0.000001, 0.00001, 0.0001));
        rows.add(new Row("kP", () -> Shooter.kP, v -> Shooter.kP = v, 1, 0.0001, 0.001, 0.01, 0.1));
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
        addPasteLine();
    }

    @Override
    public void start() {
        cur.copy(gamepad1);   // a button held through START is not a press
    }

    @Override
    public void loop() {
        // 1. One snapshot of the encoder, the clock and the battery.
        double dtMs = beginLoop();

        // 2. Buttons. While B is held nothing else counts, so a thumb that also catches A or X
        //    cannot spin the wheel straight back up.
        if (cur.b) {
            if (pressed(cur.b, prev.b)) stopAll("Stopped (B).");
        } else {
            handleMenu();
            if (pressed(cur.a, prev.a)) toggleHold();
            if (pressed(cur.x, prev.x)) startOrEndTest();
            if (pressed(cur.y, prev.y)) applyFit();
        }

        // 3. The mode decides the target (or the raw power) and moves its test along.
        switch (mode) {
            case HOLD:          spin(true); break;
            case WAIT_FOR_REST: runWaitForRest(); break;
            case STEP:          runStep(dtMs); break;
            case FF_FIT:        runFit(); break;
            case SHOT:          runShot(); break;
            default:            spin(false); break;
        }

        // 4. Log this loop's speed with the power that was on while it was measured (last loop's).
        if (log != null) log.row(nowMs - modeStartMs, dtMs, volts, wheel);

        // 5. Write. FF FIT writes raw power itself and update() would overwrite it.
        if (mode != Mode.FF_FIT) shooter.update();
        wheel.power = shooter.getPower();
        wheel.measure(nowMs);

        addScreen();
    }

    @Override
    public void stop() {
        // The SDK zeroes the motors itself; an iterative OpMode may not write them here.
        closeLog();
    }

    // ------------------------------------------------------------------ every loop

    /** Clears the bulk cache, ticks the clock, samples the battery and the wheel. Returns the loop time. */
    private double beginLoop() {
        for (LynxModule hub : hubs) hub.clearBulkCache();
        nowMs = (System.nanoTime() - startNs) / 1e6;
        double dtMs = loopTimer.tick(nowMs);
        if (nowMs - lastVoltageMs >= VOLTAGE_PERIOD_MS) {
            volts = battery.getVoltage();
            lastVoltageMs = nowMs;
        }
        wheel.velocity = shooter.getVelocity();
        prev.copy(cur);
        cur.copy(gamepad1);
        return dtMs;
    }

    private static boolean pressed(boolean now, boolean before) {
        return now && !before;
    }

    /** On: closed loop at the menu target. Off: target 0 and disarmed, so the wheel coasts. */
    private void spin(boolean on) {
        wheel.target = on ? Shooter.TARGET_TICKS_PER_SEC : 0;
        shooter.setTarget(wheel.target);
        if (on) shooter.arm();
        else shooter.disarm();
    }

    /** Any test, SHOT included. The menu is locked so the numbers being measured stay put. */
    private boolean testRunning() {
        return mode == Mode.WAIT_FOR_REST || mode == Mode.STEP || mode == Mode.FF_FIT || mode == Mode.SHOT;
    }

    // ------------------------------------------------------------------ buttons

    private void handleMenu() {
        int lines = 1 + rows.size();
        if (pressed(cur.dpad_up, prev.dpad_up)) cursor = (cursor + lines - 1) % lines;
        if (pressed(cur.dpad_down, prev.dpad_down)) cursor = (cursor + 1) % lines;
        if (testRunning()) return;   // B still stops it

        int change = pressed(cur.dpad_right, prev.dpad_right) ? 1
                : pressed(cur.dpad_left, prev.dpad_left) ? -1 : 0;
        if (change != 0) {
            if (cursor == 0) {
                test = Test.values()[(test.ordinal() + change + Test.values().length) % Test.values().length];
            } else {
                rows.get(cursor - 1).nudge(change);
            }
        }
        if (cursor >= 1) {
            if (pressed(cur.right_bumper, prev.right_bumper)) rows.get(cursor - 1).changeStep(1);
            if (pressed(cur.left_bumper, prev.left_bumper)) rows.get(cursor - 1).changeStep(-1);
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
        if (test == Test.SHOT) {
            startShot();
            return;
        }
        if (test == Test.STEP && Shooter.TARGET_TICKS_PER_SEC <= 0) {
            note = "STEP needs a target above 0.";
            return;
        }
        // STEP and FF FIT both start from rest: a wheel still coasting would skew the timing or the fit.
        result.clear();
        pendingTest = test;
        spin(false);
        mode = Mode.WAIT_FOR_REST;
        modeStartMs = nowMs;
    }

    /** B: whatever is running ends now. The last result stays on screen. */
    private void stopAll(String why) {
        if (mode == Mode.SHOT) {
            finishShot(why);
            return;
        }
        boolean wasTest = testRunning();
        endMode();
        if (wasTest) result.add(why);
        else note = why;
    }

    /** The end of every test and of HOLD: wheel off (it coasts), CSV closed, back to IDLE. */
    private void endMode() {
        shooter.disarm();
        wheel.target = 0;
        mode = Mode.IDLE;
        note = "";
        closeLog();
    }

    private void applyFit() {
        if (testRunning()) {
            note = "Y works once the test is over.";
            return;
        }
        if (fit == null || !fit.ok()) {
            note = "No fit to apply: run Test = FF FIT first.";
            return;
        }
        Shooter.kS = fit.kS;
        Shooter.kV = fit.kV;
        note = String.format(Locale.US, "Applied kS %s kV %s", plain(fit.kS), plain(fit.kV));
    }

    // ------------------------------------------------------------------ STEP

    /** A floating wheel coasts for a while; STEP and FF FIT start from rest, not from a roll. */
    private void runWaitForRest() {
        spin(false);
        if (Math.abs(wheel.velocity) > AT_REST_TICKS_PER_SEC && nowMs - modeStartMs < MAX_WAIT_FOR_REST_MS) return;

        if (pendingTest == Test.FF_FIT) {
            startFit();
            runFit();   // power on in this same loop
            return;
        }
        step = new StepStats(Shooter.TARGET_TICKS_PER_SEC, Shooter.kS, wheel.velocity);
        mode = Mode.STEP;
        modeStartMs = nowMs;
        openLog("step");
        spin(true);
    }

    private void runStep(double dtMs) {
        spin(true);
        double t = nowMs - modeStartMs;
        double runMs = runTimeSec * 1000;
        // wheel.power is what was sent last loop, which is the power behind this loop's velocity.
        step.add(t, dtMs, wheel.velocity, wheel.power, t > runMs - SETTLED_WINDOW_MS);
        if (t < runMs) return;

        result.addAll(step.summary());
        addLogPath();
        endMode();
    }

    // ------------------------------------------------------------------ FF FIT

    private void startFit() {
        fit = new FitStats(Shooter.TARGET_TICKS_PER_SEC);
        fitIndex = 0;
        fitStepStartMs = nowMs;
        mode = Mode.FF_FIT;
        modeStartMs = nowMs;
        wheel.target = 0;
        openLog("fit");
    }

    private void runFit() {
        shooter.setRawPower(FIT_POWERS[fitIndex]);
        double t = nowMs - fitStepStartMs;
        // Average only the end of each step, once it has levelled off.
        if (t >= FIT_STEP_MS - SETTLED_WINDOW_MS) fit.sample(wheel.velocity);
        if (t < FIT_STEP_MS) return;

        fit.endStep(fitIndex);
        fitIndex++;
        fitStepStartMs = nowMs;
        if (fitIndex < FIT_POWERS.length) return;

        result.addAll(fit.finish(volts));
        if (fit.ok()) result.add("Y = apply kS and kV.");
        addLogPath();
        endMode();
    }

    // ------------------------------------------------------------------ SHOT (hand feed)

    private void startShot() {
        if (Shooter.TARGET_TICKS_PER_SEC <= 0) {
            note = "SHOT needs a target above 0.";
            return;
        }
        result.clear();
        dips.clear();
        dip = null;
        shotReady = false;
        readySinceMs = -1;
        lowSinceMs = -1;
        shotIdleSinceMs = nowMs;
        mode = Mode.SHOT;
        modeStartMs = nowMs;
        openLog("shot");
        spin(true);
    }

    private void runShot() {
        spin(true);
        if (dip == null) {
            if (!shotReady) {
                // READY once the wheel has stayed in tolerance for a moment, not on one lucky sample.
                readySinceMs = wheel.inBand() ? (readySinceMs < 0 ? nowMs : readySinceMs) : -1;
                if (readySinceMs >= 0 && nowMs - readySinceMs >= READY_DWELL_MS) {
                    shotReady = true;
                    shotIdleSinceMs = nowMs;
                }
            } else if (wheel.isLow()) {
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
        dip.add(wheel);
        // Back = in tolerance again, which is what Shoot waits for before the next ball.
        boolean back = t >= MIN_DIP_MS && wheel.inBand();
        if (!back && t < DIP_TIMEOUT_MS) return;

        dip.recoveryMs = back ? t : -1;
        dips.add(dip);
        result.add(dip.line(dips.size(), Shooter.TARGET_TICKS_PER_SEC));
        dip = null;
        shotReady = false;   // READY has to be earned again before the next ball counts
        readySinceMs = -1;
        lowSinceMs = -1;
        shotIdleSinceMs = nowMs;
        if (dips.size() >= MAX_SHOTS) finishShot("");
    }

    private void finishShot(String why) {
        if (dip != null) {   // ended while a ball was still going through: keep what was seen
            dip.cutShort = true;
            dips.add(dip);
            result.add(dip.line(dips.size(), Shooter.TARGET_TICKS_PER_SEC));
            dip = null;
        }
        if (!why.isEmpty()) result.add(why);

        int complete = 0;
        double worst = -1;
        boolean saturated = false;
        boolean notBack = false;
        for (Dip d : dips) {
            if (d.cutShort) continue;
            complete++;
            if (d.recoveryMs < 0) notBack = true;
            else worst = Math.max(worst, d.recoveryMs);
            saturated |= d.saturated;
        }
        if (complete == 0) {
            result.add("No complete dips. Drop the ball into the wheel once it says READY.");
        } else if (notBack) {
            result.add("-> Not back in 2 s: the wheel can't hold this target. Lower it or check FF FIT.");
        } else if (worst > Shoot.RECOVER_TIMEOUT_MS) {
            result.add(String.format(Locale.US, "-> Slowest %.0f ms > match wait %d ms. %s", worst,
                    Shoot.RECOVER_TIMEOUT_MS, saturated ? "It hit full power: lower the target." : "Raise kP."));
        } else {
            result.add(String.format(Locale.US, "-> Slowest recovery %.0f ms, inside the match's %d ms.",
                    worst, Shoot.RECOVER_TIMEOUT_MS));
        }
        addLogPath();
        endMode();
    }

    // ------------------------------------------------------------------ CSV

    private void openLog(String tag) {
        closeLog();
        log = CsvLog.open(tag, String.format(Locale.US,
                "# kS=%s kV=%s kP=%s target=%.0f tolerance=%.0f battery=%.2fV",
                plain(Shooter.kS), plain(Shooter.kV), plain(Shooter.kP),
                Shooter.TARGET_TICKS_PER_SEC, Shooter.TOLERANCE_TICKS_PER_SEC, volts));
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
                mode == Mode.WAIT_FOR_REST ? pendingTest.label : mode.name().replace('_', ' '),
                volts, loopTimer.avgMs, loopTimer.maxMs));
        addWarnings();
        addLive();
        telemetry.addLine(status());
        if (testRunning() && !note.isEmpty()) telemetry.addLine(note);
        addSettings();
        if (!result.isEmpty()) {
            telemetry.addLine("-- result -------------------------------");
            for (String line : result) telemetry.addLine(line);
        }
        String next = nextStep();
        if (!next.isEmpty()) telemetry.addLine(next);
        addPasteLine();
        telemetry.addLine("A hold  X run test  Y apply fit  B STOP");
    }

    private void addWarnings() {
        if (loopTimer.avgMs > SLOW_LOOP_MS) {
            telemetry.addLine(String.format(Locale.US, "!! loop %.0f ms: the control is lagging", loopTimer.avgMs));
        }
        if (wheel.noEncoder) {
            telemetry.addLine("!! wheel powered but reads 0: encoder cable unplugged or in the wrong port");
        }
        if (wheel.backwards) {
            telemetry.addLine("!! wheel turns backwards: motor or encoder direction reversed");
        }
    }

    private void addLive() {
        if (wheel.target > 0) {
            telemetry.addLine(String.format(Locale.US, "speed %5.0f / %-5.0f err %+5.0f  %s",
                    wheel.velocity, wheel.target, wheel.target - wheel.velocity, wheel.inBand() ? "READY" : ""));
            telemetry.addLine(String.format(Locale.US, "   pwr %s  avg %s  rip %s  reach %s",
                    powerText(wheel.power), dash("%+.0f", wheel.window.avg), dash("%.0f", wheel.window.ripple),
                    wheel.reachMs < 0 ? "--" : String.format(Locale.US, "%.0fms", wheel.reachMs)));
        } else {
            telemetry.addLine(String.format(Locale.US, "speed %5.0f t/s   %s", wheel.velocity,
                    wheel.power == 0 ? "off" : "raw power"));
            telemetry.addLine("   pwr " + powerText(wheel.power));
        }
    }

    private String status() {
        double t = (nowMs - modeStartMs) / 1000;
        switch (mode) {
            case HOLD:
                return "HOLD: change values live. " + note;
            case WAIT_FOR_REST:
                return String.format(Locale.US, "%s: waiting for the wheel to stop (%.1f s)", pendingTest.label, t);
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
        for (int i = 0; i < rows.size(); i++) {
            Row row = rows.get(i);
            telemetry.addLine(menuLine(i + 1, row.label, String.format(Locale.US, "%-10s +-%s",
                    plain(row.get.getAsDouble()), plain(row.step()))));
        }
        telemetry.addLine("  FF power at target: " + feedforwardCheck());
    }

    private String menuLine(int index, String label, String value) {
        return String.format(Locale.US, "%s %-10s %s", index == cursor ? ">" : " ", label, value);
    }

    /** What kV and kS alone would send at the target. At 1 or more the wheel can't get there. */
    private static String feedforwardCheck() {
        if (Shooter.kV == 0) return "kV=0!";
        double ff = Shooter.kS + Shooter.kV * Shooter.TARGET_TICKS_PER_SEC;
        return String.format(Locale.US, "%.2f%s", ff, ff >= 1 ? " >1!" : "");
    }

    private String nextStep() {
        if (mode != Mode.IDLE && mode != Mode.HOLD) return "";
        if (Shooter.kV == 0) return "Next: Test = FF FIT, X. It finds kS and kV.";
        if (mode == Mode.HOLD) return "Next: raise kP until reach stops improving.";
        if (Shooter.kP == 0) return "Next: A (HOLD) and raise kP.";
        return "Next: STEP, then SHOT, then copy the line below.";
    }

    private void addPasteLine() {
        telemetry.addLine("-- Shooter.java -------------------------");
        telemetry.addLine(String.format(Locale.US, "public static double kS = %s, kV = %s, kP = %s;",
                plain(Shooter.kS), plain(Shooter.kV), plain(Shooter.kP)));
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

    /** The flywheel, every loop: what it is doing, the live average/ripple, the reach timer and the wiring checks. */
    private static final class Wheel {
        double target;     // closed-loop target this loop; 0 = off (or raw power in FF FIT)
        double velocity;   // ticks/sec, from this loop's bulk read
        double power;      // last power sent; can be above 1
        final Window window = new Window();
        double reachMs = -1;
        boolean noEncoder, backwards;

        private double lastTarget = Double.NaN;
        private double reachFromMs;
        private double noEncoderSinceMs = -1, backwardsSinceMs = -1;

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

    /** The numbers over a STEP run, from rest. */
    private static final class StepStats {
        final double target;
        final double kS;
        final double startVelocity;
        double reachMs = -1;

        private double lastOutOfBandMs = 0;
        private boolean inBandAtEnd;
        private double peak = -Double.MAX_VALUE;
        private double saturatedMs = 0;
        // The settled window: error mean and spread, power, and a least-squares slope of velocity (t in seconds).
        private int n;
        private double eSum, eeSum, pSum, vSum, tSum, ttSum, tvSum;

        StepStats(double target, double kS, double startVelocity) {
            this.target = target;
            this.kS = kS;
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
            lines.add(String.format(Locale.US, "STEP to %.0f%s", target, fromRest));
            lines.add(String.format(Locale.US, "   reach %s  settle %s",
                    reachMs < 0 ? "never" : String.format(Locale.US, "%.0fms", reachMs),
                    reachMs >= 0 && inBandAtEnd ? String.format(Locale.US, "%.0fms", lastOutOfBandMs) : "never"));
            lines.add(String.format(Locale.US, "   peak %.0f (%+.0f)  full power %.0fms", peak, peak - target, saturatedMs));
            if (n > 0) {
                double mean = eSum / n;
                double spread = Math.sqrt(Math.max(0, eeSum / n - mean * mean));
                lines.add(String.format(Locale.US, "   settled err %+.0f +-%.0f  pwr %.2f", mean, spread, pSum / n));
                lines.add(String.format(Locale.US, "   still changing %+.0f t/s per s (0 = flat)", slope()));
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

    /** FF FIT: average speed at each power, then power = kS + kV * speed by least squares. */
    private static final class FitStats {
        final double target;
        final double[] speeds = new double[FIT_POWERS.length];
        double kS = Double.NaN, kV = Double.NaN;

        private double sum;
        private int n;

        FitStats(double target) {
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
                lines.add("FIT: turns backwards. Motor or encoder direction reversed.");
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
                lines.add("FIT: the wheel barely turned. Check the motor, its power cable and encoder cable.");
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
            lines.add(String.format(Locale.US, "FIT  kS %s  kV %s  R2 %.3f", plain(kS), plain(kV), r2));
            lines.add("   pwr  .2 .4 .6 .8 1.0");
            lines.add(table.toString());
            lines.add(String.format(Locale.US, "   full power %.0f t/s @ %.2fV", fullSpeed, volts));
            if (fullSpeed > 0) {
                double percent = 100 * target / fullSpeed;
                lines.add(String.format(Locale.US, "   target %.0f = %.0f%% of that%s", target, percent,
                        percent > 90 ? "  !! no room to recover" : ""));
            }
            if (r2 < 0.95) lines.add("   !! poor fit: the wheel rubbing or a belt slipping?");
            if (m < speeds.length) lines.add(String.format(Locale.US, "   (%d low step(s) left out: wheel didn't turn)", speeds.length - m));
            return lines;
        }
    }

    /**
     * One ball through the wheel, timed from the moment it dropped below its band until it is back in
     * tolerance (-1 if not within {@link #DIP_TIMEOUT_MS}).
     */
    private static final class Dip {
        final double startMs;
        double recoveryMs = -1;
        boolean cutShort, saturated;
        private double lowest = Double.MAX_VALUE;

        Dip(double startMs) {
            this.startMs = startMs;
        }

        /** How far the wheel dropped, and whether it was flat out at any point. */
        void add(Wheel w) {
            lowest = Math.min(lowest, w.velocity);
            if (Math.abs(w.power) >= SATURATED) saturated = true;
        }

        /** "#1 back 240ms  dip -180 SAT". */
        String line(int number, double target) {
            String back = cutShort ? "cut short" : recoveryMs < 0 ? "NOT back in 2 s"
                    : String.format(Locale.US, "back %.0fms", recoveryMs);
            return String.format(Locale.US, "#%d %s  dip -%.0f%s", number, back,
                    Math.max(0, target - lowest), saturated ? " SAT" : "");
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
            out.write("t_ms,loop_ms,battery_v,target,vel,power\n");
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

        void row(double tMs, double loopMs, double volts, Wheel w) {
            try {
                out.write(String.format(Locale.US, "%.1f,%.1f,%.2f,%.0f,%.1f,%.3f\n",
                        tMs, loopMs, volts, w.target, w.velocity, w.power));
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
