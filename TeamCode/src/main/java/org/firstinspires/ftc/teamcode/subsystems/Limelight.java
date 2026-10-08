package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.util.hardware.HardwareNames;

import java.util.Collections;
import java.util.List;
import java.util.Locale;

/**
 * Wraps the Limelight 3A. One job: AprilTag <em>aiming</em> at the HIVE CELL clusters.
 *
 * <p>Front-mounted on V1 ({@link #CAMERA_YAW_OFFSET_DEGREES} = 0), so it sees the tags while the
 * robot faces the HIVE and loses them once the rear shooter is turned toward it; {@code Macros}
 * keeps the correction it took while they were visible. Every BIOBUZZ tag rides on a moving CELL
 * and the SDK v12.0 notes say they are unsuitable for field localisation, so there is no botpose
 * here. The output is {@link #getTagTx(int, int)}, the horizontal error to the cluster the shooter
 * must hit; which IDs belong to which CELL lives in {@code game/Field}.
 *
 * <h2>The one subsystem that still fails soft</h2>
 * Every other lookup in this codebase is a bare {@code hardwareMap.get} that throws on a bad config
 * name, because a crash at init is the loudest possible diagnosis. The lookup here is no different.
 * What <em>is</em> different is that this device is reached over USB-Ethernet: {@code start()} and
 * {@code getLatestResult()} are calls to another computer, and a correctly configured camera that is
 * unpowered or booting makes them throw or block on socket timeouts. Killing the OpMode over that
 * would cost a match for a mechanism the robot can drive and shoot without, so those calls — and
 * only those — are caught, recorded in {@link #isConnected()}, and reported on the driver station.
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
    /**
     * Set by the first call the camera does not answer, and never cleared: {@code start()} happens
     * once, at init, so a camera that was not talking then will not begin on its own. Power-cycle the
     * camera and restart the OpMode.
     */
    private boolean cameraFailed = false;

    public Limelight(HardwareMap hardwareMap) {
        this(hardwareMap, HardwareNames.LIMELIGHT);
    }

    public Limelight(HardwareMap hardwareMap, String name) {
        limelight = hardwareMap.get(Limelight3A.class, name);
        try {
            limelight.pipelineSwitch(APRILTAG_PIPELINE_INDEX);
            limelight.start();
        } catch (RuntimeException e) {
            cameraFailed = true;
        }
    }

    /** Takes the camera's latest frame; once per loop from {@code Robot.readSensors()}. */
    public void update() {
        try {
            latestResult = limelight.getLatestResult();
        } catch (RuntimeException e) {
            cameraFailed = true;
            latestResult = null;
        }
        if (latestResult == null || !latestResult.isValid()) {
            tagDetections = Collections.emptyList();
            return;
        }
        List<LLResultTypes.FiducialResult> tags = latestResult.getFiducialResults();
        tagDetections = tags == null ? Collections.<LLResultTypes.FiducialResult>emptyList() : tags;
    }

    /** False once a call to the camera has failed: it is configured but not answering. */
    public boolean isConnected() {
        return !cameraFailed;
    }

    /** True when the last frame carried a target and is fresh. */
    public boolean hasTarget() {
        return latestResult != null && latestResult.isValid() && !isStale();
    }

    public boolean isStale() {
        return latestResult != null && latestResult.getStaleness() > MAX_STALENESS_MS;
    }

    public void stop() {
        try {
            limelight.stop();
        } catch (RuntimeException e) {
            cameraFailed = true;
        }
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

    /**
     * Every tag in the fresh frame, with its 3D pose; empty when the frame is stale or has none.
     * For {@code Bench: Limelight} only. Match code aims from {@link #getTagTx} and never localises
     * from a tag, because every BIOBUZZ tag rides a moving CELL.
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
        if (cameraFailed) return "NOT RESPONDING";
        if (!hasTarget()) return isStale() ? "stale" : "no target";
        return String.format(Locale.US, "%d tag(s)  tx %.1f", getTagCount(), latestResult.getTx());
    }
}
