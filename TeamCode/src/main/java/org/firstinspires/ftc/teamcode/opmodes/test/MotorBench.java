package org.firstinspires.ftc.teamcode.opmodes.test;

import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.pedro.Constants;
import org.firstinspires.ftc.teamcode.subsystems.Shooter;
import org.firstinspires.ftc.teamcode.util.hardware.HardwareNames;

import java.util.ArrayList;
import java.util.List;
import java.util.Locale;
import java.util.Set;

/**
 * Spins every motor the code knows about, one at a time, at {@link #POWER}, to check the wiring:
 * each name in {@link HardwareNames} is plugged into the motor it claims to be, turns the way the code
 * expects, and has its own encoder cable.
 *
 * <pre>
 * Gamepad 1, after START. WHEELS OFF THE FLOOR.
 *   dpad up/down   pick a motor (moving stops the one that is on)
 *   A              the picked motor on / off
 *   B              stop
 * </pre>
 *
 * Each motor gets the same direction the real code gives it, so "on" should turn it the way the hint
 * says. If it turns the other way, flip the direction in the file the screen names. If a different
 * motor turns, its cable is in the wrong port (or its name is on the wrong port in the configuration).
 *
 * <p>Unlike the match code, a name missing from the configuration does not crash: its line says
 * NOT IN CONFIG, so one run lists every missing name.
 */
@TeleOp(name = "Bench: Motors", group = "Bench")
public class MotorBench extends OpMode {
    /** Power for the motor that is on. Low on purpose. */
    public static double POWER = 0.1;

    /** Slower than this counts as not moving. */
    private static final double MOVING_TICKS_PER_SEC = 20;
    /** The encoder check waits this long after a motor turns on, so it has time to get going. */
    private static final long CHECK_AFTER_MS = 500;

    private List<LynxModule> hubs;
    private final List<Motor> motors = new ArrayList<>();
    private int cursor = 0;
    /** The motor that has power, or -1 for none. Only one ever does. */
    private int on = -1;
    private long onSinceMs;
    private String note = "";

    @Override
    public void init() {
        // MANUAL: one bulk read per loop for all eight encoders, cleared at the top of each loop.
        hubs = hardwareMap.getAll(LynxModule.class);
        for (LynxModule hub : hubs) hub.setBulkCachingMode(LynxModule.BulkCachingMode.MANUAL);

        // The same directions the real code uses. The drive ones are read from pedro/Constants, not copied.
        add(HardwareNames.FRONT_LEFT_MOTOR, "front left wheel: robot FORWARD",
                Constants.drivetrainConfig.frontLeftDirection.get(), "pedro/Constants");
        add(HardwareNames.FRONT_RIGHT_MOTOR, "front right wheel: robot FORWARD",
                Constants.drivetrainConfig.frontRightDirection.get(), "pedro/Constants");
        add(HardwareNames.BACK_LEFT_MOTOR, "back left wheel: robot FORWARD",
                Constants.drivetrainConfig.backLeftDirection.get(), "pedro/Constants");
        add(HardwareNames.BACK_RIGHT_MOTOR, "back right wheel: robot FORWARD",
                Constants.drivetrainConfig.backRightDirection.get(), "pedro/Constants");
        add(HardwareNames.INTAKE_MOTOR, "intake: roller + tunnel pull IN",
                DcMotorSimple.Direction.FORWARD, "Intake.java");
        add(HardwareNames.FLYWHEEL_MOTOR, "flywheel: ball OUT the rear",
                DcMotorSimple.Direction.FORWARD, "Shooter.java");
        add(HardwareNames.COUNTER_ROLLER_MOTOR, "counter-roller: ball OUT too",
                Shooter.ROLLER_REVERSED ? DcMotorSimple.Direction.REVERSE : DcMotorSimple.Direction.FORWARD,
                "Shooter.ROLLER_REVERSED");
        add(HardwareNames.FEEDER_MOTOR, "up-wheels: carry balls UP",
                Shooter.FEEDER_REVERSED ? DcMotorSimple.Direction.REVERSE : DcMotorSimple.Direction.FORWARD,
                "Shooter.FEEDER_REVERSED");

        telemetry.setDisplayFormat(Telemetry.DisplayFormat.MONOSPACE);
        telemetry.setMsTransmissionInterval(100);
    }

    /** Looks the motor up without crashing if it is missing, and sets it up like the real code does. */
    private void add(String name, String hint, DcMotorSimple.Direction direction, String directionFrom) {
        DcMotorEx motor = hardwareMap.tryGet(DcMotorEx.class, name);
        String hub = "";
        if (motor != null) {
            motor.setDirection(direction);
            motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);           // POWER is plain power
            motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);  // a stopped motor coasts
            motor.setPower(0);
            hub = hubOf(motor);
        }
        motors.add(new Motor(name, hint, direction, directionFrom, motor, hub));
    }

    /** "Control Hub p0": the hub's name in the configuration and the motor port on it. */
    private String hubOf(DcMotorEx motor) {
        Set<String> names = hardwareMap.getNamesOf(motor.getController());
        String hub = names.isEmpty() ? "hub ?" : names.iterator().next();
        return hub + " p" + motor.getPortNumber();
    }

    /** Before START: nothing has power. The list shows what is missing, and the encoders still read. */
    @Override
    public void init_loop() {
        readEncoders();
        telemetry.addLine("MOTOR TEST   WHEELS OFF THE FLOOR");
        telemetry.addLine("Press START, then dpad + A.");
        telemetry.addLine("Turn a wheel by hand now: only its");
        telemetry.addLine("own line's t/s should change.");
        int missing = 0;
        for (Motor m : motors) if (m.motor == null) missing++;
        if (missing > 0) telemetry.addLine(String.format(Locale.US, "!! %d name(s) not in the config", missing));
        addList();
    }

    @Override
    public void start() {
        // *WasPressed() remembers presses until read: forget anything pressed during init.
        gamepad1.resetEdgeDetection();
    }

    @Override
    public void loop() {
        readEncoders();
        long now = System.currentTimeMillis();

        // Read every press once per loop. B wins: a thumb that also catches A cannot start a motor.
        boolean b = gamepad1.bWasPressed();
        boolean a = gamepad1.aWasPressed();
        boolean up = gamepad1.dpadUpWasPressed();
        boolean down = gamepad1.dpadDownWasPressed();
        if (b) {
            stopAll();
            note = "Stopped (B).";
        } else {
            if (up) moveCursor(-1);
            if (down) moveCursor(1);
            if (a) toggle(now);
        }

        telemetry.addLine(String.format(Locale.US, "MOTOR TEST  power %.2f  A on/off  B stop", POWER));
        addList();
        addDetail(now);
    }

    // No stop(): the SDK zeroes every motor itself and rejects motor writes from there.

    // ------------------------------------------------------------------ buttons

    /** A: the picked motor on, or off if it already is. Turning one on turns every other one off first. */
    private void toggle(long now) {
        boolean wasOn = on == cursor;
        stopAll();
        note = "";
        if (wasOn) return;

        Motor m = motors.get(cursor);
        if (m.motor == null) {
            note = m.name + " is not in the config: nothing to run.";
            return;
        }
        m.motor.setPower(POWER);
        on = cursor;
        onSinceMs = now;
    }

    /** Moving off a motor stops it, so the arrow always points at the one that is on. */
    private void moveCursor(int step) {
        stopAll();
        note = "";
        cursor = (cursor + step + motors.size()) % motors.size();
    }

    private void stopAll() {
        for (Motor m : motors) {
            if (m.motor != null) m.motor.setPower(0);
        }
        on = -1;
    }

    // ------------------------------------------------------------------ the screen

    private void readEncoders() {
        for (LynxModule hub : hubs) hub.clearBulkCache();
        for (Motor m : motors) m.velocity = m.motor == null ? 0 : m.motor.getVelocity();
    }

    /** One line per motor: on/off, its encoder speed, and the hub and port it is on. */
    private void addList() {
        for (int i = 0; i < motors.size(); i++) {
            Motor m = motors.get(i);
            String arrow = i == cursor ? ">" : " ";
            if (m.motor == null) {
                telemetry.addLine(String.format(Locale.US, "%s %-6s NOT IN CONFIG", arrow, m.name));
            } else {
                telemetry.addLine(String.format(Locale.US, "%s %-6s %-3s %+5.0f t/s  %s",
                        arrow, m.name, i == on ? "ON" : "off", m.velocity, m.hub));
            }
        }
    }

    /** The picked motor: what it should do, where its direction is set, and the encoder check. */
    private void addDetail(long now) {
        Motor m = motors.get(cursor);
        telemetry.addLine("-- " + m.name + " ------------------------------");
        if (m.motor == null) {
            telemetry.addLine("No motor named \"" + m.name + "\" in the");
            telemetry.addLine("Robot Controller configuration.");
        } else {
            telemetry.addLine(m.hint);
            telemetry.addLine("dir " + m.direction + ", from " + m.directionFrom);
            telemetry.addLine(m.motor.getConnectionInfo());
            if (on < 0) telemetry.addLine("A = turn it on");
            else telemetry.addLine("Wrong motor turning? Cable or port.");
        }
        if (!note.isEmpty()) telemetry.addLine(note);
        addEncoderWarning(now);
    }

    /** Once a motor has been on a moment: is its own encoder the one that moves? */
    private void addEncoderWarning(long now) {
        if (on < 0 || now - onSinceMs < CHECK_AFTER_MS) return;
        Motor running = motors.get(on);
        if (Math.abs(running.velocity) >= MOVING_TICKS_PER_SEC) return;
        for (Motor other : motors) {
            if (other != running && Math.abs(other.velocity) >= MOVING_TICKS_PER_SEC) {
                telemetry.addLine("!! " + running.name + " is on but " + other.name + "'s encoder moves:");
                telemetry.addLine("   encoder cables swapped?");
                return;
            }
        }
        telemetry.addLine("!! " + running.name + "'s encoder reads 0: not turning");
        telemetry.addLine("   at this power, or no encoder cable");
    }

    /** One motor from HardwareNames, and how the real code sets it up. */
    private static final class Motor {
        final String name;
        final String hint;
        final DcMotorSimple.Direction direction;
        final String directionFrom;
        /** Null when the name is not in the configuration. */
        final DcMotorEx motor;
        final String hub;
        double velocity;   // ticks/sec, from this loop's bulk read

        Motor(String name, String hint, DcMotorSimple.Direction direction, String directionFrom,
              DcMotorEx motor, String hub) {
            this.name = name;
            this.hint = hint;
            this.direction = direction;
            this.directionFrom = directionFrom;
            this.motor = motor;
            this.hub = hub;
        }
    }
}
