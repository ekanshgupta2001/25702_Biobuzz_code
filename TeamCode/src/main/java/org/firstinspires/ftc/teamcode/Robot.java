package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.VoltageSensor;

import com.pedropathing.math.Pose;

import org.firstinspires.ftc.teamcode.game.Field;
import org.firstinspires.ftc.teamcode.subsystems.Drivetrain;
import org.firstinspires.ftc.teamcode.subsystems.Intake;
import org.firstinspires.ftc.teamcode.subsystems.Limelight;
import org.firstinspires.ftc.teamcode.subsystems.Shooter;
import org.firstinspires.ftc.teamcode.util.field.Alliance;

import java.util.ArrayList;
import java.util.List;

/**
 * The whole robot: the drivetrain, the intake, the shooter, the Limelight, and the two loop halves.
 *
 * <p>Every subsystem is a {@code public final} field. A config name that is missing from the Robot
 * Controller configuration (including {@code limelight}) throws at init, where it is easy to read.
 *
 * <h2>The loop has two halves</h2>
 * <ul>
 *   <li>{@link #readSensors()}, top of the loop: clear every hub's bulk cache, read the Limelight,
 *       work out the distance to the CELL and set both wheel speeds from it, sample the battery.</li>
 *   <li>{@link #writeActuators()}, bottom of the loop: every mechanism writes its motors.</li>
 * </ul>
 * The OpMode decides what the mechanisms should do in between, so it always decides on this loop's
 * sensor data.
 *
 * <h2>Nothing counts game pieces</h2>
 * There is no sensor anywhere in the intake or the tunnel. The driver decides when to stop intaking
 * (BIOBUZZ G407, at most 4 controlled) and when to fire.
 *
 * <h2>The shot distance</h2>
 * {@link #updateShotDistance()} runs every loop and is the whole story.
 *
 * <p><b>Today, with no Pinpoint config:</b> the shot distance is the Limelight's distance to one of
 * our CELL's tags, smoothed a little. No tag = no distance, and the shooter goes to its fixed speeds.
 *
 * <p><b>Once {@code Constants.localizerConfig} is filled in:</b>
 * <pre>
 * position distance = from the Pinpoint pose to the CELL (game/Field)
 * correction        = tag distance - position distance, updated only while a tag is in view
 * shot distance     = position distance + correction      -> Shooter.SPEED_TABLE
 * </pre>
 * So the speed follows the robot's position (a lost tag does not change it), and every time the
 * camera sees the CELL the position distance is pulled back onto the tag distance. Every number is on
 * the Teleop card.
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

    /**
     * How fast the shot distance follows the tag: each loop with a tag in view it moves this fraction
     * of the way there. 1 = jump there at once, smaller = smoother.
     */
    public static double CORRECTION_BLEND = 0.2;
    /** A correction bigger than this means the pose or the Field numbers are wrong: the card warns. */
    public static double CORRECTION_WARN_INCHES = 12;

    private Alliance alliance = Alliance.BLUE;
    /** The CELL the distance is measured to: the one the camera last saw, the starting up-CELL until then. */
    private Field.CellSide targetCell = Field.startingUpCellSide(Alliance.BLUE);

    // The shot distance and the numbers it is made of, all inches. NaN = not known this loop.
    private double tagInches = Double.NaN;        // Limelight: camera to the tag
    private double positionInches = Double.NaN;   // Pinpoint pose to the CELL
    private double correctionInches = 0;          // tag - position, from the last time a tag was seen
    private double shotInches = Double.NaN;       // what the speed table is given

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
    }

    /**
     * Once, when the alliance is known: switches the Limelight to that alliance's tag pipeline and
     * starts the shot distance over, measured to the CELL that starts the match facing up.
     */
    public void setAlliance(Alliance alliance) {
        this.alliance = alliance;
        targetCell = Field.startingUpCellSide(alliance);
        correctionInches = 0;
        shotInches = Double.NaN;
        limelight.setAlliance(alliance);
    }

    // ---- The two loop halves ----

    /**
     * Refreshes the sensor data and sets both wheel speeds. Call at the TOP of the loop. Clearing the
     * bulk caches here is what makes the whole loop see one consistent snapshot.
     */
    public void readSensors() {
        for (LynxModule hub : hubs) {
            hub.clearBulkCache();
        }

        limelight.update();
        updateShotDistance();
        shooter.setSpeedFromDistance(shotInches);   // NaN = no distance: the fixed speeds

        long now = System.currentTimeMillis();
        sampleBattery(now);
        trackLoopTime(now);
    }

    /** Writes every motor. Call at the BOTTOM of the loop. The drivetrain goes last. */
    public void writeActuators() {
        intake.update();
        shooter.update();
        drivetrain.update();
    }

    // ---- Shot distance ----

    /** See the class comment. Every number it works out is kept in a field so the card can show it. */
    private void updateShotDistance() {
        // 1. The Limelight: how far the tag is, and which of our two CELLs it is on. A tag that is
        //    not on one of our CELLs is ignored: tagInches is NaN for it.
        Field.CellSide seen = cellOfTag(limelight.getShotTagId());   // null = no tag of ours in view
        tagInches = seen == null ? Double.NaN : limelight.getShotDistanceInches();
        boolean tagSeen = !Double.isNaN(tagInches);
        if (tagSeen) targetCell = seen;

        // 2. No pose (no Pinpoint config, or the Pinpoint sent a bad reading this loop): the tag is
        //    all there is. Smoothed, so an inch of camera noise does not move the speeds every frame.
        Pose pose = drivetrain.getPose();
        if (pose == null || Double.isNaN(pose.x()) || Double.isNaN(pose.y())) {
            positionInches = Double.NaN;
            if (!tagSeen) shotInches = Double.NaN;            // the shooter holds, then goes to fixed
            else if (Double.isNaN(shotInches)) shotInches = tagInches;   // first sight: no lag
            else shotInches += CORRECTION_BLEND * (tagInches - shotInches);
            return;
        }

        // 3. The position: how far the pose is from that CELL.
        Pose cell = Field.cell(alliance, targetCell);
        positionInches = Math.hypot(cell.x() - pose.x(), cell.y() - pose.y());

        // 4. The Limelight corrects it, only while it sees a tag.
        if (tagSeen) {
            double wanted = tagInches - positionInches;
            correctionInches += CORRECTION_BLEND * (wanted - correctionInches);
        }
        shotInches = positionInches + correctionInches;
    }

    /** Which of this alliance's two CELLs a tag ID is on (game/Field.tagRange), or null if neither. */
    private Field.CellSide cellOfTag(int tagId) {
        for (Field.CellSide side : Field.CellSide.values()) {
            int[] range = Field.tagRange(alliance, side);
            if (tagId >= range[0] && tagId <= range[1]) return side;
        }
        return null;
    }

    /** The distance the speed table is given this loop, inches; NaN = none (fixed speeds). */
    public double getShotInches() {
        return shotInches;
    }

    /** The Limelight's distance to the tag, inches; NaN = no tag in view. */
    public double getTagInches() {
        return tagInches;
    }

    /** The distance from the Pinpoint pose to the CELL, inches; NaN = no pose. */
    public double getPositionInches() {
        return positionInches;
    }

    /** What the Limelight has added to the position distance, inches. */
    public double getCorrectionInches() {
        return correctionInches;
    }

    /** The CELL the distance is measured to. */
    public Field.CellSide getTargetCell() {
        return targetCell;
    }

    /**
     * True when the camera sees a tag of that CELL of ours right now. A CELL's tags are only visible
     * while it is UP, so this is how the Limelight autos tell whether the HIVE has tipped.
     */
    public boolean seesCell(Field.CellSide side) {
        return !Double.isNaN(tagInches) && targetCell == side;
    }

    /** True when the Limelight has had to move the position distance a long way: check the pose. */
    public boolean isPositionOff() {
        return !Double.isNaN(positionInches) && Math.abs(correctionInches) > CORRECTION_WARN_INCHES;
    }

    // ---- Battery and loop time ----

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
        shooter.setBatteryVolts(batteryVolts);   // only used when Shooter.TUNED_AT_VOLTS is set
    }
}
