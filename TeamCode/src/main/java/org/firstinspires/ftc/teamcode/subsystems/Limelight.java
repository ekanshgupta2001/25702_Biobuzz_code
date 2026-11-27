package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.teamcode.util.field.Alliance;
import org.firstinspires.ftc.teamcode.util.hardware.HardwareNames;

import java.util.Collections;
import java.util.List;
import java.util.Locale;

/**
 * Wraps the Limelight 3A, which reads the HIVE CELL AprilTags.
 *
 * <p>Front-mounted ({@link #CAMERA_YAW_OFFSET_DEGREES} = 0), facing the way the shot leaves, so the
 * tags stay in view while the robot is lined up to shoot. Its main job is the shot distance:
 * {@link #getShotDistanceInches()} is the distance to the tag the robot points at, and
 * {@code Robot.readSensors()} sets the wheel speeds from it. Every BIOBUZZ tag rides on a moving CELL and
 * the SDK v12.0 notes say they are unsuitable for field localisation, so there is no botpose here.
 * Which IDs belong to which CELL lives in {@code game/Field}.
 *
 * <h2>The one subsystem that still fails soft</h2>
 * Every other lookup in this codebase is a bare {@code hardwareMap.get} that throws on a bad config
 * name, because a crash at init is the loudest possible diagnosis. The lookup here is no different.
 * What <em>is</em> different is that this device is reached over USB-Ethernet: {@code start()} and
 * {@code getLatestResult()} are calls to another computer, and a correctly configured camera that is
 * unpowered or booting makes them throw or block on socket timeouts. Killing the OpMode over that
 * would cost a match for a mechanism the robot can drive and shoot without (the flywheel falls back
 * to its fixed speed), so those calls — and only those — are caught, recorded in
 * {@link #isConnected()}, and reported on the driver station.
 */
public class Limelight {
    /** Limelight web UI pipelines, one per alliance; each passes only that alliance's tags. */
    public static int BLUE_PIPELINE = 0;   // tags 38-45, both blue CELLs
    public static int RED_PIPELINE = 1;    // tags 30-37, both red CELLs
    /** The camera's forward axis relative to the robot's, degrees CCW: 0 = front, 180 = rear. */
    public static double CAMERA_YAW_OFFSET_DEGREES = 0.0;
    /** A result older than this means the camera has stopped delivering frames. */
    public static long MAX_STALENESS_MS = 250;

    private final Limelight3A limelight;
    private LLResult latestResult;
    private List<LLResultTypes.FiducialResult> tagDetections = Collections.emptyList();
    /**
     * Set by a call the camera does not answer, cleared by the next fresh frame. If {@code start()}
     * itself failed at init there never is a frame, so it stays set: power-cycle the camera and
     * restart the OpMode.
     */
    private boolean cameraFailed = false;
    /** The pipeline setAlliance asked for. A distance is only trusted while the camera is on it. */
    private int wantedPipeline = BLUE_PIPELINE;
    private long lastSwitchMs = 0;
    /** Times update() has re-sent the pipeline since setAlliance. It stops at {@link #MAX_SWITCH_RETRIES}. */
    private int switchRetries = 0;
    public static int MAX_SWITCH_RETRIES = 5;

    public Limelight(HardwareMap hardwareMap) {
        this(hardwareMap, HardwareNames.LIMELIGHT);
    }

    public Limelight(HardwareMap hardwareMap, String name) {
        limelight = hardwareMap.get(Limelight3A.class, name);
        try {
            limelight.pipelineSwitch(BLUE_PIPELINE);   // BLUE until setAlliance says otherwise
            limelight.start();
        } catch (RuntimeException e) {
            cameraFailed = true;
        }
    }

    /** Switches to the alliance's pipeline. Once, when the alliance is known: it is a network call. */
    public void setAlliance(Alliance alliance) {
        wantedPipeline = pipelineFor(alliance);
        switchRetries = 0;
        sendPipeline();
    }

    private void sendPipeline() {
        lastSwitchMs = System.currentTimeMillis();
        try {
            limelight.pipelineSwitch(wantedPipeline);
        } catch (RuntimeException e) {
            cameraFailed = true;
        }
    }

    public static int pipelineFor(Alliance alliance) {
        return alliance == Alliance.RED ? RED_PIPELINE : BLUE_PIPELINE;
    }

    /** Takes the camera's latest frame; once per loop from {@code Robot.readSensors()}. */
    public void update() {
        // Everything that touches a frame is inside the try: a bad frame must never stop the OpMode.
        try {
            latestResult = limelight.getLatestResult();
            if (latestResult != null && !isStale()) cameraFailed = false;   // it is talking again

            // A camera that was booting when the switch was sent is still on its old pipeline: ask
            // again, once a second, a few times (the switch is a network call, so not for ever).
            int running = getPipelineIndex();
            if (running >= 0 && running != wantedPipeline && switchRetries < MAX_SWITCH_RETRIES
                    && System.currentTimeMillis() - lastSwitchMs > 1000) {
                switchRetries++;
                sendPipeline();
            }

            List<LLResultTypes.FiducialResult> tags =
                    latestResult != null && latestResult.isValid() ? latestResult.getFiducialResults() : null;
            tagDetections = tags == null ? Collections.<LLResultTypes.FiducialResult>emptyList() : tags;
        } catch (RuntimeException e) {
            cameraFailed = true;
            latestResult = null;
            tagDetections = Collections.emptyList();
        }
    }

    /** False when the camera is configured but not answering (unplugged, unpowered, still booting). */
    public boolean isConnected() {
        if (cameraFailed) return false;
        try {
            return limelight.isConnected();
        } catch (RuntimeException e) {
            return false;
        }
    }

    /** True while the camera reports the pipeline of the alliance last asked for. */
    public boolean isOnWantedPipeline() {
        return getPipelineIndex() == wantedPipeline;
    }

    /** True when the last frame carried a target and is fresh. */
    public boolean hasTarget() {
        return latestResult != null && latestResult.isValid() && !isStale();
    }

    public boolean isStale() {
        return latestResult != null && latestResult.getStaleness() > MAX_STALENESS_MS;
    }

    /** The pipeline the camera reports in its latest fresh frame (tag or not), or -1 when there is none. */
    public int getPipelineIndex() {
        return latestResult == null || isStale() ? -1 : latestResult.getPipelineIndex();
    }

    public void stop() {
        try {
            limelight.stop();
        } catch (RuntimeException e) {
            cameraFailed = true;
        }
    }

    // ---- AprilTags: shot distance ----

    /**
     * Straight-line distance in inches from the camera to the visible tag nearest the crosshair
     * (smallest |tx|), which is the CELL the robot points at. NaN when there is no fresh tag, or when
     * the pipeline has 3D off and sends a zero pose, or while the camera is still on the other
     * alliance's pipeline (those would be the wrong HIVE's tags). Called every loop.
     */
    public double getShotDistanceInches() {
        LLResultTypes.FiducialResult nearest = nearestTag();
        if (nearest == null) return Double.NaN;
        Pose3D pose = nearest.getTargetPoseCameraSpace();
        if (pose == null || pose.getPosition() == null) return Double.NaN;
        Position p = pose.getPosition().toUnit(DistanceUnit.INCH);
        double inches = Math.sqrt(p.x * p.x + p.y * p.y + p.z * p.z);
        // Under an inch is not a real tag: it is the all-zero pose sent when 3D is off.
        return inches < 1 ? Double.NaN : inches;
    }

    /**
     * The ID of that same tag (the one {@link #getShotDistanceInches()} measures to), or -1 when there
     * is none. {@code Robot} uses it to tell which of the two CELLs the robot is pointing at.
     */
    public int getShotTagId() {
        LLResultTypes.FiducialResult nearest = nearestTag();
        return nearest == null ? -1 : nearest.getFiducialId();
    }

    /**
     * How far off centre that same tag is, in degrees: + = the tag is to the RIGHT of where the robot
     * points, 0 = dead ahead. NaN when there is no tag. The Limelight autos turn until this is near 0.
     */
    public double getShotTagTxDegrees() {
        LLResultTypes.FiducialResult nearest = nearestTag();
        return nearest == null ? Double.NaN : nearest.getTargetXDegrees();
    }

    /**
     * The visible tag nearest the crosshair (smallest |tx|), or null when there is no fresh tag or the
     * camera is still on the other alliance's pipeline.
     */
    private LLResultTypes.FiducialResult nearestTag() {
        if (!hasTarget() || !isOnWantedPipeline()) return null;
        LLResultTypes.FiducialResult nearest = null;
        for (int i = 0; i < tagDetections.size(); i++) {
            LLResultTypes.FiducialResult tag = tagDetections.get(i);
            if (nearest == null
                    || Math.abs(tag.getTargetXDegrees()) < Math.abs(nearest.getTargetXDegrees())) {
                nearest = tag;
            }
        }
        return nearest;
    }

    /**
     * Every tag in the fresh frame, with its 3D pose; empty when the frame is stale or has none.
     * For {@code Bench: Limelight} only. Match code never localises from a tag, because every
     * BIOBUZZ tag rides a moving CELL.
     */
    public List<LLResultTypes.FiducialResult> getTags() {
        return hasTarget() ? tagDetections : Collections.<LLResultTypes.FiducialResult>emptyList();
    }

    /** Number of tags of any ID in the fresh frame. */
    public int getTagCount() {
        return hasTarget() ? tagDetections.size() : 0;
    }

    /** One line for a telemetry card: whether the camera is talking, and what it can see. */
    public String getStatusLine() {
        if (!isConnected()) return "NOT RESPONDING";
        if (getPipelineIndex() >= 0 && !isOnWantedPipeline()) {
            return "WRONG PIPELINE " + getPipelineIndex() + " (want " + wantedPipeline + ")";
        }
        if (!hasTarget()) return isStale() ? "stale" : "no target";
        return String.format(Locale.US, "%d tag(s)  tx %.1f", getTagCount(), latestResult.getTx());
    }
}
