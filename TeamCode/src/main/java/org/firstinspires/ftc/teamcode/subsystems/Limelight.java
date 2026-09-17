package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.LLStatus;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.util.hardware.Hardware;
import org.firstinspires.ftc.teamcode.util.hardware.HardwareNames;

import java.util.Collections;
import java.util.List;

/**
 * Wraps the Limelight 3A. One job: AprilTag <em>aiming</em> at the HIVE CELL clusters.
 *
 * <p>Front-mounted on V1 ({@link #CAMERA_YAW_OFFSET_DEGREES} = 0), so it sees the tags while the
 * robot faces the HIVE and loses them once the rear shooter is turned toward it; {@code Macros}
 * keeps the correction it took while they were visible. Every BIOBUZZ tag rides on a moving CELL
 * and the SDK v12.0 notes say they are unsuitable for field localisation, so there is no botpose
 * here. The output is {@link #getTagTx(int, int)}, the horizontal error to the cluster the shooter
 * must hit; which IDs belong to which CELL lives in {@code game/Field}.
 */
public class Limelight {
    public static int APRILTAG_PIPELINE_INDEX = 0;
    /** The camera's forward axis relative to the robot's, degrees CCW: 0 = front, 180 = rear. */
    public static double CAMERA_YAW_OFFSET_DEGREES = 0.0;
    /** A result older than this means the camera has stopped delivering frames. */
    public static long MAX_STALENESS_MS = 250;

    private final Limelight3A limelight;
    private LLResult latestResult;
    private List<LLResultTypes.FiducialResult> tagDetections = Collections.emptyList();

    public Limelight(HardwareMap hardwareMap) {
        this(hardwareMap, HardwareNames.LIMELIGHT);
    }

    public Limelight(HardwareMap hardwareMap, String name) {
        limelight = Hardware.get(hardwareMap, Limelight3A.class, name);
        if (limelight == null) return;
        // pipelineSwitch() and start() are synchronous HTTP calls to the camera. If it is unpowered
        // these block on socket timeouts inside OpMode init(), so fail soft instead.
        try {
            limelight.pipelineSwitch(APRILTAG_PIPELINE_INDEX);
            limelight.start();
        } catch (RuntimeException e) {
            Hardware.recordFailure(name, "did not respond during init: " + e.getMessage());
        }
    }

    /** False when the camera is missing from the robot configuration. All reads then return empty. */
    public boolean isAvailable() {
        return limelight != null;
    }

    /** Takes the camera's latest frame; once per loop from {@code Robot.readSensors()}. */
    public void update() {
        if (limelight == null) return;
        latestResult = limelight.getLatestResult();
        if (latestResult == null || !latestResult.isValid()) {
            tagDetections = Collections.emptyList();
            return;
        }
        List<LLResultTypes.FiducialResult> tags = latestResult.getFiducialResults();
        tagDetections = tags == null ? Collections.<LLResultTypes.FiducialResult>emptyList() : tags;
    }

    public LLStatus getStatus() {
        return limelight == null ? null : limelight.getStatus();
    }

    /** True when the last frame carried a target and is fresh. */
    public boolean hasTarget() {
        return latestResult != null && latestResult.isValid() && !isStale();
    }

    public boolean isStale() {
        return latestResult != null && latestResult.getStaleness() > MAX_STALENESS_MS;
    }

    /** Raw horizontal offset of the current target in degrees, or NaN with no target. */
    public double getTx() {
        return hasTarget() ? latestResult.getTx() : Double.NaN;
    }

    /** Raw vertical offset in degrees, or NaN with no target. */
    public double getTy() {
        return hasTarget() ? latestResult.getTy() : Double.NaN;
    }

    /** Target area as a percentage of the image (0-100), or NaN with no target. */
    public double getTa() {
        return hasTarget() ? latestResult.getTa() : Double.NaN;
    }

    public void stop() {
        if (limelight != null) limelight.stop();
    }

    // ---- AprilTags: aiming ----

    /**
     * Mean horizontal offset, in degrees, of the visible tags with IDs in {@code [minId, maxId]},
     * or NaN when none is visible. For a BIOBUZZ cluster this approximates the CELL opening's
     * centre; the drivetrain aim law consumes it directly ({@code Macros.aimHeading}).
     */
    public double getTagTx(int minId, int maxId) {
        if (!hasTarget()) return Double.NaN;
        double sum = 0;
        int n = 0;
        for (int i = 0; i < tagDetections.size(); i++) {
            LLResultTypes.FiducialResult tag = tagDetections.get(i);
            int id = tag.getFiducialId();
            if (id < minId || id > maxId) continue;
            sum += tag.getTargetXDegrees();
            n++;
        }
        return n == 0 ? Double.NaN : sum / n;
    }

    /** Number of tags of any ID in the fresh frame. */
    public int getTagCount() {
        return hasTarget() ? tagDetections.size() : 0;
    }

    /** Every tag in the fresh frame (read-only; empty when stale). */
    public List<LLResultTypes.FiducialResult> getTags() {
        return hasTarget() ? Collections.unmodifiableList(tagDetections)
                : Collections.<LLResultTypes.FiducialResult>emptyList();
    }
}
