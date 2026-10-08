package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.VoltageSensor;

import org.firstinspires.ftc.teamcode.commands.Macros;
import org.firstinspires.ftc.teamcode.subsystems.Drivetrain;
import org.firstinspires.ftc.teamcode.subsystems.Intake;
import org.firstinspires.ftc.teamcode.subsystems.Limelight;
import org.firstinspires.ftc.teamcode.subsystems.Shooter;
import org.firstinspires.ftc.teamcode.util.time.MatchClock;

import java.util.ArrayList;
import java.util.List;

/**
 * The whole robot: four mechanisms, one camera, and the two loop halves.
 *
 * <p>Every subsystem is a {@code public final} field. There is no supplier wiring, no sensor policy
 * and no fail-soft layer, because there is nothing left to arbitrate: the robot has seven motors, a
 * Pinpoint and a Limelight, and a missing config name throws at init where it is easy to read.
 *
 * <h2>The loop is split in two halves, deliberately</h2>
 * The OpMode calls these on either side of the Ivy scheduler:
 * <ul>
 *   <li>{@link #readSensors()} — clear every hub's bulk cache, refresh the Limelight, tick the match
 *       clock, sample the battery. Top of the loop, before commands run.</li>
 *   <li>{@link #writeActuators()} — push every mechanism's intent to hardware, drivetrain last.
 *       Bottom of the loop, after commands run.</li>
 * </ul>
 * There is no {@code update()} that does both: one method would let a caller put observe-and-act on
 * the same side of {@code Scheduler.execute()}, and every command would then decide on last loop's
 * data. The order is structural, not a convention.
 *
 * <h2>Nothing counts game pieces</h2>
 * There is no sensor anywhere in the intake or the tunnel, so the code never claims to know how many
 * pieces are aboard. The operator decides when to stop intaking (BIOBUZZ G407, at most 4 controlled)
 * and when to fire. Every "shot count" in telemetry is a count of tunnel pulses, not of pieces.
 */
public class Robot {
    /**
     * How often the battery is re-read. {@link VoltageSensor} reads are not served from the Lynx bulk
     * cache, so each is its own bus transaction; four times a second is plenty to explain a weak shot.
     */
    public static long VOLTAGE_SAMPLE_MS = 250;

    public final Drivetrain drivetrain;
    public final Intake intake;
    public final Shooter shooter;
    public final Limelight limelight;
    public final Macros macros;

    private MatchClock matchClock;

    private final List<LynxModule> hubs;
    private final List<VoltageSensor> voltageSensors;
    private double batteryVolts = 0;
    private long lastVoltageSampleMs = 0;

    /** Loop time, averaged over {@link #LOOP_AVERAGE_OVER} loops so the figure on the card is readable. */
    public static int LOOP_AVERAGE_OVER = 10;
    private long lastLoopStampMs = 0;
    private int loopsSinceStamp = 0;
    private double loopMs = 0;

    public Robot(HardwareMap hardwareMap) {
        // MANUAL bulk caching batches every encoder and current read on a hub into one bus
        // transaction per loop. Without it each getVelocity()/getCurrent() is its own USB round trip.
        hubs = hardwareMap.getAll(LynxModule.class);
        for (LynxModule hub : hubs) {
            hub.setBulkCachingMode(LynxModule.BulkCachingMode.MANUAL);
        }

        // Resolved once rather than walking hardwareMap.voltageSensor every loop. It is a
        // DeviceMapping (Iterable, not a Collection), so it is copied element by element.
        voltageSensors = new ArrayList<>();
        for (VoltageSensor sensor : hardwareMap.voltageSensor) {
            voltageSensors.add(sensor);
        }

        drivetrain = new Drivetrain(hardwareMap);
        intake = new Intake(hardwareMap);
        shooter = new Shooter(hardwareMap);
        limelight = new Limelight(hardwareMap);

        macros = new Macros(this);   // last: it takes this, so every field must already be set
    }

    // ---- The two loop halves ----

    /**
     * Refreshes cached sensor data. Call at the TOP of the loop, before commands run. Clearing the
     * bulk caches here is what makes the whole loop see one consistent snapshot.
     */
    public void readSensors() {
        for (LynxModule hub : hubs) {
            hub.clearBulkCache();
        }
        limelight.update();

        long now = System.currentTimeMillis();
        if (matchClock != null) matchClock.update(now);
        sampleBattery(now);
        trackLoopTime(now);
    }

    /**
     * Pushes intent to hardware. Call at the BOTTOM of the loop, after commands run. The drivetrain
     * goes last: its update is the one {@code follower.update()} per loop.
     */
    public void writeActuators() {
        intake.update();
        shooter.update();
        drivetrain.update();
    }

    // ---- Match state ----

    /** Starts the match clock for the period. Call once from the OpMode's {@code start()}. */
    public void startMatch(MatchClock.Period period) {
        matchClock = MatchClock.forPeriod(period);
        matchClock.start(System.currentTimeMillis());
    }

    /** {@code null} before {@link #startMatch}; callers must null-check (init_loop, diagnostics). */
    public MatchClock getMatchClock() {
        return matchClock;
    }

    /** Lowest volts across all sensors, refreshed at most every {@link #VOLTAGE_SAMPLE_MS}; 0 if unreadable. */
    public double getBatteryVolts() {
        return batteryVolts;
    }

    /** Average loop time in milliseconds, or 0 until the first window completes. */
    public double getLoopMs() {
        return loopMs;
    }

    /** Loop rate in Hz — the first line of every telemetry card, because it is the number to watch. */
    public double getLoopHz() {
        return loopMs <= 0 ? 0 : 1000.0 / loopMs;
    }

    private void trackLoopTime(long nowMs) {
        loopsSinceStamp++;
        if (lastLoopStampMs == 0) {
            lastLoopStampMs = nowMs;
            loopsSinceStamp = 0;
        } else if (loopsSinceStamp >= LOOP_AVERAGE_OVER) {
            loopMs = (double) (nowMs - lastLoopStampMs) / loopsSinceStamp;
            lastLoopStampMs = nowMs;
            loopsSinceStamp = 0;
        }
    }

    private void sampleBattery(long nowMs) {
        // lastVoltageSampleMs == 0 means "never sampled": the first call always reads.
        if (lastVoltageSampleMs != 0 && nowMs - lastVoltageSampleMs < VOLTAGE_SAMPLE_MS) return;
        lastVoltageSampleMs = nowMs;

        double lowest = Double.MAX_VALUE;
        for (VoltageSensor sensor : voltageSensors) {
            double v = sensor.getVoltage();
            if (v > 0 && v < lowest) lowest = v;
        }
        batteryVolts = lowest == Double.MAX_VALUE ? 0 : lowest;
    }

    // ---- Shared cancel and shutdown paths ----

    /**
     * Cleanup shared by an operator abort and the auto's buzzer stop: hands the follower back to the
     * sticks (Ivy cannot stop a follower that was handed a path) and marks the macro cancelled.
     */
    public void abortMacro() {
        drivetrain.cancelPath();
        macros.markCancelled();
    }

    /**
     * Stops every mechanism now, from inside the loop. Called by the autonomous buzzer safety net
     * (G403: no powered movement after the period ends). Never from an OpMode's {@code stop()}: it
     * writes motors, which the SDK rejects there (see {@link #stop()}).
     */
    public void stopMechanisms() {
        intake.stop();
        shooter.disarm();
        drivetrain.cancelPath();
        intake.update();
        shooter.update();
    }

    /**
     * Releases non-actuator hardware at OpMode stop. Deliberately no motor writes: the SDK rejects
     * them from an iterative OpMode's {@code stop()} (CANCELLED_FOR_SAFETY) and zeroes the motors
     * itself. Use {@link #stopMechanisms()} for a mid-OpMode stop.
     */
    public void stop() {
        limelight.stop();
    }
}
