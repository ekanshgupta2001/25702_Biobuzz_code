package org.firstinspires.ftc.teamcode;

import com.pedropathing.algorithm.Foresight;
import com.pedropathing.follower.Follower;
import com.pedropathing.math.Pose;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.VoltageSensor;

import org.firstinspires.ftc.teamcode.commands.Macros;
import org.firstinspires.ftc.teamcode.game.PieceType;
import org.firstinspires.ftc.teamcode.subsystems.ColorSensor;
import org.firstinspires.ftc.teamcode.subsystems.Drivetrain;
import org.firstinspires.ftc.teamcode.subsystems.Intake;
import org.firstinspires.ftc.teamcode.subsystems.Limelight;
import org.firstinspires.ftc.teamcode.subsystems.OpenLoopDrive;
import org.firstinspires.ftc.teamcode.subsystems.Shooter;
import org.firstinspires.ftc.teamcode.subsystems.Storage;
import org.firstinspires.ftc.teamcode.subsystems.Transfer;
import org.firstinspires.ftc.teamcode.util.diagnostics.MatchLogger;
import org.firstinspires.ftc.teamcode.util.field.Alliance;
import org.firstinspires.ftc.teamcode.util.field.PoseFusion;
import org.firstinspires.ftc.teamcode.util.hardware.Hardware;
import org.firstinspires.ftc.teamcode.util.hardware.HardwareNames;
import org.firstinspires.ftc.teamcode.util.time.Clock;
import org.firstinspires.ftc.teamcode.util.time.MatchClock;

import java.util.ArrayList;
import java.util.Collections;
import java.util.List;

/**
 * Top-level composition of the BIOBUZZ V1 robot. Owns every subsystem as a public final field, the
 * {@link Macros}, the {@link PoseFusion} and the {@link MatchClock}, and wires cross-subsystem
 * suppliers in its constructor so that no subsystem ever imports another.
 *
 * <p>The loop is split in two halves that the OpMode calls on either side of the Ivy scheduler:
 * <ul>
 *   <li>{@link #readSensors()} — clear every hub's bulk cache, refresh the Limelight and the colour
 *       sensors, tick the match clock, sample the battery. Top of the loop, before commands run.</li>
 *   <li>{@link #writeActuators()} — push every mechanism's target to hardware, drivetrain last.
 *       Bottom of the loop, after commands run.</li>
 * </ul>
 * There is deliberately no {@code update()} that does both: a single method would let a caller put
 * observe-and-act on the same side of {@code Scheduler.execute()}, and every command would then
 * decide on last loop's data. See docs/03-software-architecture.md sections 1 and 4.
 *
 * <p>Three constructors: {@link #Robot(HardwareMap)} for OpModes, {@link #Robot(HardwareMap, Clock)}
 * to inject time, and a composition constructor that takes pre-built subsystems so JVM tests can
 * assemble a robot from fakes.
 *
 * <p>Every device fails soft: a missing config entry disables one subsystem
 * ({@code isAvailable()} false) and is listed by {@link #getMissingHardware()} for SelfTest and the
 * init telemetry; the OpMode still runs.
 */
public class Robot {
    /**
     * How often the battery is re-read. {@link VoltageSensor} reads are <em>not</em> served from the
     * Lynx bulk cache, so each one is its own bus transaction; once every quarter second is plenty.
     */
    public static long VOLTAGE_SAMPLE_MS = 250;
    /**
     * A presence sensor with a distance reading says "piece here" within this many inches.
     * Placeholder: measure the reading with and without a piece in {@code Bench: ColorSensor}.
     */
    public static double PRESENCE_DISTANCE_INCHES = 2.0;

    public final Drivetrain drivetrain;
    /**
     * Open-loop driving through Pedro's motor layer, for before the follower is tuned. Built on the
     * real motors only while {@link #drivetrain} is unavailable; once {@code Constants.create()}
     * returns a follower this is an unfitted stub, so the two can never share the drive motors.
     */
    public final OpenLoopDrive openLoopDrive;
    public final Intake intake;
    public final Storage storage;
    public final Transfer transfer;
    public final Shooter shooter;
    public final Limelight limelight;

    /** Storage entrance: counts pieces into the queue and identifies POLLEN vs NECTAR (G408). */
    public final ColorSensor storageEntranceSensor;
    /** Fourth storage slot occupied: ORed into {@link Storage#isFull()} (G407). */
    public final ColorSensor storageFullSensor;
    /** A piece is in the vertical transfer: storage exit edge and the lift interlock. */
    public final ColorSensor transferSensor;
    /** A piece is staged at the flywheel. Absent means the transfer falls back to timed pulses. */
    public final ColorSensor shooterFeedSensor;

    public final Macros macros;
    public final PoseFusion poseFusion = new PoseFusion();

    private MatchClock matchClock;
    private PieceType blobTarget = PieceType.POLLEN;
    /** Our alliance, set by the OpMode; {@code null} until known. Drives the G408 reject. */
    private Alliance alliance = null;

    /** The fitted presence sensors, read one per loop in turn (see {@link #readSensors()}). */
    private final List<ColorSensor> presenceSensors = new ArrayList<>();
    private int nextPresenceSensor = 0;

    private final Clock clock;
    private final List<LynxModule> hubs;
    private final List<VoltageSensor> voltageSensors;
    private double batteryVolts = 0;
    private long lastVoltageSampleMs = 0;

    public Robot(HardwareMap hardwareMap) {
        this(hardwareMap, Clock.system());
    }

    public Robot(HardwareMap hardwareMap, Clock clock) {
        // The missing-device registry is static and would otherwise accumulate across OpMode runs.
        Hardware.reset();
        this.clock = clock;

        // MANUAL bulk caching batches every encoder/current read on a hub into one bus transaction
        // per loop. Without it each getVelocity()/getCurrent() is its own USB round-trip, and the
        // four velocity mechanisms are each read several times per loop (control, telemetry, log).
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

        drivetrain = new Drivetrain(hardwareMap, clock);
        // One motor layer at a time: the open-loop drive is built on the real motors only while there
        // is no follower, so two Pedro Mecanum objects can never share the four drive motors. Once
        // Constants.create() returns a follower this is an unavailable stub and every call no-ops.
        openLoopDrive = drivetrain.isAvailable()
                ? new OpenLoopDrive((com.pedropathing.drivetrain.Drivetrain) null, clock)
                : new OpenLoopDrive(hardwareMap, clock);
        intake = new Intake(hardwareMap, HardwareNames.INTAKE_MOTOR, clock);
        storage = new Storage(hardwareMap, HardwareNames.STORAGE_MOTOR, HardwareNames.STORAGE_MOTOR_2, clock);
        transfer = new Transfer(hardwareMap, HardwareNames.TRANSFER_MOTOR, clock);
        shooter = new Shooter(hardwareMap, HardwareNames.SHOOTER_MOTOR, HardwareNames.SHOOTER_MOTOR_2, clock);
        limelight = new Limelight(hardwareMap);   // starts polling itself when present

        storageEntranceSensor = new ColorSensor(hardwareMap, HardwareNames.SENSOR_STORAGE_ENTRANCE,
                ColorSensor.DEFAULT_GAIN, true);
        storageFullSensor = new ColorSensor(hardwareMap, HardwareNames.SENSOR_STORAGE_FULL,
                ColorSensor.DEFAULT_GAIN, true);
        transferSensor = new ColorSensor(hardwareMap, HardwareNames.SENSOR_TRANSFER,
                ColorSensor.DEFAULT_GAIN, true);
        shooterFeedSensor = new ColorSensor(hardwareMap, HardwareNames.SENSOR_SHOOTER_FEED,
                ColorSensor.DEFAULT_GAIN, true);

        collectPresenceSensors();
        wireSuppliers();
        macros = new Macros(this);   // last: it takes this, so every field must already be set
    }

    /**
     * Composes already-built subsystems. For tests, and for any robot whose hardware is resolved
     * somewhere other than the hardware map. No hubs or voltage sensors are known, so bulk caching
     * is untouched and {@link #getBatteryVolts()} reads 0. Does not reset the {@link Hardware}
     * registry, since the subsystems were constructed before this call. Sensor arguments may be
     * {@code new ColorSensor(null)} for "not fitted".
     */
    public Robot(Drivetrain drivetrain, OpenLoopDrive openLoopDrive, Intake intake, Storage storage, Transfer transfer,
                 Shooter shooter, Limelight limelight,
                 ColorSensor storageEntranceSensor, ColorSensor storageFullSensor,
                 ColorSensor transferSensor, ColorSensor shooterFeedSensor, Clock clock) {
        this.clock = clock;
        hubs = Collections.emptyList();
        voltageSensors = Collections.emptyList();
        this.drivetrain = drivetrain;
        this.openLoopDrive = openLoopDrive;
        this.intake = intake;
        this.storage = storage;
        this.transfer = transfer;
        this.shooter = shooter;
        this.limelight = limelight;
        this.storageEntranceSensor = storageEntranceSensor;
        this.storageFullSensor = storageFullSensor;
        this.transferSensor = transferSensor;
        this.shooterFeedSensor = shooterFeedSensor;
        collectPresenceSensors();
        wireSuppliers();
        macros = new Macros(this);
    }

    private void collectPresenceSensors() {
        for (ColorSensor sensor : new ColorSensor[] {storageFullSensor, transferSensor, shooterFeedSensor}) {
            if (sensor.isAvailable()) presenceSensors.add(sensor);
        }
    }

    /**
     * The only place two subsystems are connected, and the only place that decides which sensor
     * is <em>trusted</em>, as opposed to merely present in the configuration. Each supplier is a
     * sensor question answered by a private predicate below, so swapping a colour sensor for a
     * beam-break at any point is a one-line change here and nowhere else. The setters stay public,
     * so a test may override any of them after construction.
     *
     * <p>The trust policy, in one place (fixthese R2-A3, R2-B1):
     * <ul>
     *   <li>A {@code null} supplier means "no sensor" to Storage and Transfer, and every fallback
     *       follows from that: no entrance, the count cannot rise, so zero is "unknown" and the
     *       shooting macros fire blind rather than refuse; no exit, the shooting macro dead-reckons
     *       the count down; no feed, the transfer runs timed pulses; no full sensor, only the count
     *       (or the operator) can say full. Never pass an always-false supplier for any of these.</li>
     *   <li>The entrance sensor counts pieces by <b>distance</b> when it has a distance reading (a
     *       REV V3's proximity read needs no calibration) and by hue only once
     *       {@link PieceType#HUES_CALIBRATED} says the windows were measured. A hue-only sensor with
     *       unmeasured windows is therefore <em>not fitted</em> as far as the count is concerned:
     *       plugging it in must not turn a shooting robot into one that reports NO_TARGET.</li>
     *   <li>The G408 reject (opponent NECTAR) needs the hue, so it is wired only once the hues are
     *       measured; {@code Intake.REJECT_ENABLED} still has to be switched on as well.</li>
     * </ul>
     * {@link #sensingSummary()} prints the result on every init card.
     */
    private void wireSuppliers() {
        boolean countTrusted = storageEntranceSensor.isAvailable()
                && (storageEntranceSensor.hasDistance() || PieceType.HUES_CALIBRATED);
        boolean classifyTrusted = countTrusted && PieceType.HUES_CALIBRATED;
        intake.setCapturedSupplier(countTrusted ? this::pieceEnteringStorage : null);
        intake.setFullSupplier(storage::isFull);                    // G407: roller held still at 4
        intake.setRejectSupplier(classifyTrusted ? this::opponentNectarAtEntrance : null);   // G408
        storage.setEntranceSupplier(countTrusted ? this::pieceEnteringStorage : null);  // rising edge -> count + 1
        storage.setFullSupplier(storageFullSensor.isAvailable() ? this::storageFullSensorSees : null);  // ORed with count >= CAPACITY
        transfer.setInLiftSupplier(this::pieceInTransfer);
        storage.setExitSupplier(transferSensor.isAvailable() ? this::pieceInTransfer : null);
        transfer.setAtFeedSupplier(shooterFeedSensor.isAvailable() ? this::pieceAtShooterFeed : null);
    }

    /** Which alliance we are, for the G408 reject. The OpMode sets it whenever it changes. */
    public void setAlliance(Alliance alliance) {
        this.alliance = alliance;
    }

    public Alliance getAlliance() {
        return alliance;
    }

    /**
     * One line for the init cards: what each sensor point is actually doing, as decided by
     * {@link #wireSuppliers()}. "Fitted" and "trusted" are different things and the drivers should
     * see which one they have.
     */
    public String sensingSummary() {
        String entrance;
        if (!storageEntranceSensor.isAvailable()) {
            entrance = "none (count unknown, shoots blind)";
        } else if (!storage.hasEntranceSensor()) {
            entrance = "fitted, NOT trusted (hues not measured, no distance)";
        } else {
            entrance = storageEntranceSensor.hasDistance() ? "count by distance" : "count by hue";
            entrance += PieceType.HUES_CALIBRATED ? ", G408 reject wired" : " (hues not measured: no G408 reject)";
        }
        return "entrance=" + entrance
                + " | full=" + (storage.hasFullSensor() ? "fitted" : "none")
                + " | transfer=" + (storage.hasExitSensor() ? "fitted" : "none")
                + " | feed=" + (transfer.hasFeedSensor() ? "fitted" : "none (timed pulses)");
    }

    // ---- Sensor predicates: the seam between a reserved sensor point and its sensor type ----

    /**
     * A piece is entering the storage: presence first (distance where the sensor has it, otherwise
     * a hue match), then classification, because a piece being thrown back out (G408) is not
     * counted. Presence and classification are different questions; only this point asks both.
     */
    private boolean pieceEnteringStorage() {
        if (!pieceNear(storageEntranceSensor)) return false;
        return !(Intake.REJECT_ENABLED && opponentNectarAtEntrance());
    }

    private boolean opponentNectarAtEntrance() {
        if (alliance == null) return false;
        Alliance seen = PieceType.nectarAllianceAt(storageEntranceSensor);
        return seen != null && seen != alliance;
    }

    private boolean storageFullSensorSees() {
        return pieceNear(storageFullSensor);
    }

    private boolean pieceInTransfer() {
        return pieceNear(transferSensor);
    }

    private boolean pieceAtShooterFeed() {
        return pieceNear(shooterFeedSensor);
    }

    /**
     * "A piece is here" for every sensor point. Distance when the sensor has it (a REV V3
     * proximity read does not care about lighting or which colour the piece is); the hue match only
     * as a fallback, because a hue window nobody has measured fails silently to "no piece", which
     * the interlocks read as "keep going" and the count reads as "nothing ever entered".
     */
    private boolean pieceNear(ColorSensor sensor) {
        if (sensor.hasDistance()) {
            double inches = sensor.getDistanceInches();
            return !Double.isNaN(inches) && inches <= PRESENCE_DISTANCE_INCHES;
        }
        return PieceType.anyAtSensor(sensor);
    }

    // ---- The two loop halves ----

    /**
     * Refreshes cached sensor data. Call at the TOP of the loop, before commands run.
     *
     * <p>Clearing the bulk caches here is what makes the whole loop see one consistent snapshot.
     *
     * <p>Colour sensors are I2C and outside the bulk read: each costs two transactions (colour and
     * distance), several milliseconds on a Control Hub. The entrance sensor is read every loop
     * because a passing piece is a short edge and the count must not miss it. The three presence
     * points (full, transfer, feed) watch pieces that sit for hundreds of milliseconds, so the
     * <em>fitted</em> ones are read one per loop in rotation: with one fitted it is read every
     * loop, with three each is refreshed every third loop, about 60 ms of latency at most, for
     * half the bus time (fixthese R2-A1). Watch the loop line in {@code Bench: Color sensors}.
     */
    public void readSensors() {
        for (LynxModule hub : hubs) {
            hub.clearBulkCache();
        }
        // Pushed every loop so an edit to the target piece (or its size) reaches the camera
        // geometry without a redeploy.
        limelight.setTargetHeightInches(blobTarget.targetHeightInches());
        limelight.update();
        storageEntranceSensor.update();
        if (!presenceSensors.isEmpty()) {
            presenceSensors.get(nextPresenceSensor).update();
            nextPresenceSensor = (nextPresenceSensor + 1) % presenceSensors.size();
        }

        long now = clock.nowMs();
        if (matchClock != null) matchClock.update(now);
        sampleBattery(now);
    }

    /**
     * Pushes queued outputs to hardware. Call at the BOTTOM of the loop, after commands run.
     * {@link Storage#update()} also counts sensor edges, so it runs every loop even when idle.
     * The drivetrain goes last: its update is the one {@code follower.update()} per loop.
     */
    public void writeActuators() {
        intake.update();
        storage.update();
        transfer.update();
        shooter.update();
        openLoopDrive.update();
        drivetrain.update();
    }

    // ---- Localization ----

    /**
     * Blends any absolute fix into the pose estimate and writes the result back to the drivetrain.
     * In BIOBUZZ every AprilTag rides on a moving HIVE CELL, so {@link Limelight#getBotposeAsPedroPose()}
     * is expected to return {@code null} all season and this runs odometry-only; the plumbing stays
     * so a static reference can be added without touching the loop.
     */
    public void updateLocalization() {
        Pose odometry = drivetrain.getPose();
        if (odometry == null) return;

        Pose vision = limelight.getBotposeAsPedroPose();   // already null unless trustworthy
        Pose corrected = poseFusion.update(
                clock.nowMs(), odometry, vision, limelight.getVisionLatencyMs());

        // Write back only when fusion actually changed the estimate. On the odometry-only path the
        // fused x/y equal the follower's own (the filter's correction term is exactly zero), and a
        // write every loop released the heading hold each tick, so it never corrected anything, and
        // re-wrote the Pinpoint over I2C for nothing (fixthese B1). setPose releases the hold, which
        // is right for a real correction: the old setpoint was in the old frame.
        if (corrected != null && poseFusion.getLastResult() == PoseFusion.Result.ACCEPTED) {
            drivetrain.setPose(corrected);
        }
    }

    /**
     * Hard-sets the pose from an AprilTag fix and re-seeds fusion. Returns {@code false} until the
     * AprilTag pipeline is active and a trustworthy botpose exists; callers retry next loop. Expected
     * to report no fix in BIOBUZZ (see {@link #updateLocalization()}).
     */
    public boolean tryLocalizeFromAprilTag() {
        if (limelight.getPipelineIndex() != Limelight.APRILTAG_PIPELINE_INDEX) {
            // The switch takes several frames to take effect; the next loop retries.
            limelight.activateAprilTagPipeline();
            return false;
        }
        Pose botpose = limelight.getBotposeAsPedroPose();
        if (botpose == null) return false;
        drivetrain.setPose(botpose);
        poseFusion.seed(botpose);
        return true;
    }

    // ---- Match state ----

    /** Starts the match clock for the period. Call once from the OpMode's {@code start()}. */
    public void startMatch(MatchClock.Period period) {
        matchClock = MatchClock.forPeriod(period);
        matchClock.start(clock.nowMs());
    }

    /** {@code null} before {@link #startMatch}; callers MUST null-check (init_loop, diagnostics). */
    public MatchClock getMatchClock() {
        return matchClock;
    }

    public Clock getClock() {
        return clock;
    }

    /** The piece the colour pipeline is looking for; its height is pushed to the camera each loop. */
    public PieceType getBlobTarget() {
        return blobTarget;
    }

    public void setBlobTarget(PieceType target) {
        if (target != null) blobTarget = target;
    }

    /** Lowest volts across all sensors, refreshed at most every {@link #VOLTAGE_SAMPLE_MS}; 0 if unreadable. */
    public double getBatteryVolts() {
        return batteryVolts;
    }

    /** Config names that failed to resolve, in lookup order. Empty when everything is present. */
    public List<String> getMissingHardware() {
        return Hardware.getMissing();
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
     * Cleanup shared by the natural end of a vision macro and by an operator abort: hands the
     * follower back to the sticks, restores the AprilTag pipeline, and marks the macro cancelled.
     */
    public void abortMacro() {
        drivetrain.cancelPath();
        limelight.activateAprilTagPipeline();
        macros.markCancelled();
    }

    /**
     * Sets every mechanism's intent to stopped and hands the follower back. Velocity mechanisms
     * only record the intent; follow with {@link #writeActuators()} to push the zeros. For
     * SelfTest's {@code finally} and any mid-OpMode emergency stop.
     */
    public void stopMechanisms() {
        intake.stop();
        storage.stop();
        transfer.stop();
        shooter.stop();
        openLoopDrive.stop();
        drivetrain.cancelPath();
    }

    /**
     * Releases non-actuator hardware at OpMode stop. Deliberately no motor writes: the SDK rejects
     * them from an iterative OpMode's {@code stop()} (CANCELLED_FOR_SAFETY) and zeroes the motors
     * itself. A caller that needs motors stopped mid-OpMode uses {@link #stopMechanisms()} followed
     * by {@link #writeActuators()}.
     */
    public void stop() {
        limelight.stop();
        storageEntranceSensor.setLight(false);
        storageFullSensor.setLight(false);
        transferSensor.setLight(false);
        shooterFeedSensor.setLight(false);
    }

    // ---- Match log ----

    /**
     * One row for {@link MatchLogger}, in {@link MatchLogger#BIOBUZZ_COLUMNS} order. The logger
     * knows nothing about the robot; this is the one place that knows every subsystem. Follower-
     * derived cells are {@code NaN} when there is no follower, so a run whose drivetrain failed to
     * build is visibly missing data rather than looking like a perfect zero-error run.
     */
    public Object[] logCells(double loopMs) {
        Pose pose = drivetrain.getPose();
        Follower follower = drivetrain.getFollower();
        Foresight foresight = follower != null && follower.algorithm() instanceof Foresight
                ? (Foresight) follower.algorithm() : null;
        return new Object[] {
                clock.nowMs(),
                matchClock == null ? "NONE" : matchClock.getPhase().toString(),
                matchClock == null ? Double.NaN : matchClock.getRemainingSeconds(),
                loopMs,
                batteryVolts,
                pose == null ? Double.NaN : pose.x(),
                pose == null ? Double.NaN : pose.y(),
                pose == null ? Double.NaN : pose.heading(),
                pathMode(),
                follower == null ? Double.NaN : follower.completion(),
                foresight == null ? Double.NaN : foresight.translationalError(),
                foresight == null ? Double.NaN : foresight.headingError(),
                intake.getMode(),
                intake.getVelocityTicksPerSec(),
                intake.getCurrentAmps(),
                storage.count(),
                transfer.getMode(),
                drivetrain.isHeadingHoldActive() ? Math.toDegrees(drivetrain.getHeldHeading()) : Double.NaN,
                drivetrain.isAimLocked() ? 1 : 0,
                shooter.getRpm(),
                shooter.getTargetRpm(),
                limelight.hasTarget() ? 1 : 0,
                limelight.getTx(),
                limelight.getTy(),
                poseFusion.getLastResult(),
                macros.getActiveName(),
                macros.getOutcome(),
        };
    }

    private String pathMode() {
        if (!drivetrain.isAvailable()) return "NONE";
        if (drivetrain.isFollowingPath()) return "FOLLOW";
        if (drivetrain.isHoldingPose()) return "HOLD";
        return "OTHER";
    }
}
