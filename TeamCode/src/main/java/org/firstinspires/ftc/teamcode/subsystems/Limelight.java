package org.firstinspires.ftc.teamcode.subsystems;

import com.pedropathing.math.Pose;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.LLStatus;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;
import org.firstinspires.ftc.teamcode.util.field.FieldConstants;
import org.firstinspires.ftc.teamcode.util.hardware.Hardware;
import org.firstinspires.ftc.teamcode.util.hardware.HardwareNames;
import org.firstinspires.ftc.teamcode.util.math.Angles;
import org.firstinspires.ftc.teamcode.util.math.MedianFilter;
import org.firstinspires.ftc.teamcode.util.math.VisionMath;

import java.util.Collections;
import java.util.List;

/**
 * Wraps the Limelight 3A. Two jobs, one per pipeline: AprilTag <em>aiming</em> at the HIVE CELL
 * clusters, and a colour-blob detector for game pieces on the floor.
 *
 * <p>Game-agnostic on purpose. This class talks about a "blob", a "target height" and a tag-ID
 * range; what the blob is, how tall it is, and which IDs belong to which alliance live in
 * {@code game/}, and {@code Robot} pushes the height in every loop via
 * {@link #setTargetHeightInches(double)}.
 *
 * <p><b>BIOBUZZ:</b> every AprilTag rides on a moving HIVE CELL, and the SDK v12.0 notes say they
 * are unsuitable for field localisation. {@link #getBotposeAsPedroPose()} keeps its gates and is
 * expected to return {@code null} all season; the useful AprilTag output is
 * {@link #getTagTx(int, int)}, the horizontal error to the cluster the shooter must hit.
 */
public class Limelight {
    /** Added to the tag-derived yaw, for a camera whose forward axis is not the robot's. */
    public static double BOTPOSE_HEADING_OFFSET_RAD = 0.0;
    /**
     * Set true only after the camera mount constants below have been measured on the built robot
     * ({@code Bench: Limelight}, estimated distance against a tape measure). Until then the
     * approach-pose geometry is a guess, and Teleop refuses the collect/align macros rather than
     * drive four seconds toward a wrong spot.
     */
    public static boolean MOUNT_CALIBRATED = false;
    /**
     * Set true only once the Limelight field frame has been checked against ours on a real field.
     * The Limelight's frame has its origin at the field centre with X along the red wall; ours has
     * its origin at the A1 corner with +X toward column F. The conversion in
     * {@link #getBotposeAsPedroPose()} adds the half-field offset and uses the yaw as is, which is
     * a guess; nothing this season needs it (every BIOBUZZ tag moves), so it stays gated to null.
     */
    public static boolean BOTPOSE_FRAME_VERIFIED = false;

    public static int APRILTAG_PIPELINE_INDEX = 0;
    public static int BLOB_PIPELINE_INDEX = 1;

    // Camera mount geometry. The mount is rigid, so these are constants; measure them on the real
    // robot before trusting any distance estimate. Pitch is positive toward the floor (the
    // Limelight docs define their mount angle positive upward).
    public static double CAMERA_HEIGHT_INCHES = 12.0;
    public static double CAMERA_PITCH_DEGREES = 20.0;
    public static double CAMERA_FORWARD_OFFSET_INCHES = 6.0;
    public static double CAMERA_LEFT_OFFSET_INCHES = 0.0;
    public static double CAMERA_YAW_OFFSET_DEGREES = 0.0;

    /** Stop this far short of the piece, so the path ends with it at the intake, not under us. */
    public static double PICKUP_STANDOFF_INCHES = 8.0;
    /** Any estimate beyond this is rejected rather than driven to. */
    public static double MAX_VALID_DISTANCE_INCHES = 120.0;
    /** A result older than this means the camera has stopped delivering frames. */
    public static long MAX_STALENESS_MS = 250;

    /** Frames of agreement required before a detection is trusted enough to drive at. */
    public static int DETECTION_WINDOW = 5;
    /** Reject the window if the blob is jumping around by more than this many degrees. */
    public static double MAX_DETECTION_SPREAD_DEGREES = 6.0;

    private final Limelight3A limelight;
    private double targetHeightInches = 0.0;
    private LLResult latestResult;
    private int currentPipeline;
    private List<LLResultTypes.ColorResult> blobDetections = Collections.emptyList();
    private LLResultTypes.ColorResult primaryBlob = null;
    private List<LLResultTypes.FiducialResult> tagDetections = Collections.emptyList();

    private final MedianFilter txFilter = new MedianFilter(DETECTION_WINDOW);
    private final MedianFilter tyFilter = new MedianFilter(DETECTION_WINDOW);

    public Limelight(HardwareMap hardwareMap) {
        this(hardwareMap, HardwareNames.LIMELIGHT, APRILTAG_PIPELINE_INDEX);
    }

    public Limelight(HardwareMap hardwareMap, String name, int pipeline) {
        limelight = Hardware.get(hardwareMap, Limelight3A.class, name);
        currentPipeline = pipeline;
        if (limelight == null) return;
        // pipelineSwitch() and start() are synchronous HTTP calls to the camera. If it is unpowered
        // these block on socket timeouts inside OpMode init(), so fail soft instead.
        try {
            limelight.pipelineSwitch(pipeline);
            limelight.start();
        } catch (RuntimeException e) {
            Hardware.recordFailure(name, "did not respond during init: " + e.getMessage());
        }
    }

    /** False when the camera is missing from the robot configuration. All reads then return empty. */
    public boolean isAvailable() {
        return limelight != null;
    }

    /** Height of the blob target's centre above the floor, pushed by {@code Robot} from the game. */
    public void setTargetHeightInches(double inches) {
        targetHeightInches = inches;
    }

    public void update() {
        if (limelight == null) return;
        latestResult = limelight.getLatestResult();
        if (latestResult == null || !latestResult.isValid()) {
            clearDetections();
            return;
        }

        List<LLResultTypes.FiducialResult> tags = latestResult.getFiducialResults();
        tagDetections = tags == null ? Collections.<LLResultTypes.FiducialResult>emptyList() : tags;

        List<LLResultTypes.ColorResult> colors = latestResult.getColorResults();
        if (colors == null || colors.isEmpty()) {
            blobDetections = Collections.emptyList();
            primaryBlob = null;
            txFilter.reset();
            tyFilter.reset();
            return;
        }

        // Pick the largest blob in a single pass, with no per-loop allocation.
        blobDetections = colors;
        LLResultTypes.ColorResult largest = colors.get(0);
        for (int i = 1; i < colors.size(); i++) {
            if (colors.get(i).getTargetArea() > largest.getTargetArea()) largest = colors.get(i);
        }
        primaryBlob = largest;

        txFilter.add(largest.getTargetXDegrees());
        tyFilter.add(largest.getTargetYDegrees());
    }

    private void clearDetections() {
        blobDetections = Collections.emptyList();
        primaryBlob = null;
        tagDetections = Collections.emptyList();
        txFilter.reset();
        tyFilter.reset();
    }

    public LLResult getLatestResult() {
        return latestResult;
    }

    public LLStatus getStatus() {
        return limelight == null ? null : limelight.getStatus();
    }

    /** True when the last frame carried a target of any kind and is fresh. Pipeline-agnostic. */
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

    private boolean switchPipeline(int pipeline) {
        if (limelight == null) return false;
        if (pipeline == currentPipeline) return true;
        boolean ok;
        try {
            ok = limelight.pipelineSwitch(pipeline);
        } catch (RuntimeException e) {
            return false;
        }
        if (ok) {
            // Only cached on success, so a failed switch is retried rather than short-circuited.
            currentPipeline = pipeline;
            latestResult = null;
            clearDetections();
        }
        return ok;
    }

    public String getPipelineName() {
        if (currentPipeline == APRILTAG_PIPELINE_INDEX) return "apriltag";
        if (currentPipeline == BLOB_PIPELINE_INDEX) return "blob";
        return "pipeline " + currentPipeline;
    }

    public int getPipelineIndex() {
        return currentPipeline;
    }

    public void start() {
        if (limelight != null) limelight.start();
    }

    public void stop() {
        if (limelight != null) limelight.stop();
    }

    // ---- AprilTags: aiming ----

    public boolean activateAprilTagPipeline() {
        return switchPipeline(APRILTAG_PIPELINE_INDEX);
    }

    /** True when at least one tag with an ID in {@code [minId, maxId]} is in the fresh frame. */
    public boolean seesTag(int minId, int maxId) {
        return !Double.isNaN(getTagTx(minId, maxId));
    }

    /**
     * Mean horizontal offset, in degrees, of the visible tags with IDs in {@code [minId, maxId]},
     * or NaN when none is visible. For a BIOBUZZ cluster this approximates the CELL opening's
     * centre; the drivetrain aim law consumes it directly ({@code Macros.aimHeading}).
     */
    public double getTagTx(int minId, int maxId) {
        if (!hasTarget() || currentPipeline != APRILTAG_PIPELINE_INDEX) return Double.NaN;
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

    /** Every tag in the fresh frame (read-only; empty when stale or on the blob pipeline). */
    public List<LLResultTypes.FiducialResult> getTags() {
        return hasTarget() ? Collections.unmodifiableList(tagDetections)
                : Collections.<LLResultTypes.FiducialResult>emptyList();
    }

    public Pose3D getBotpose() {
        return hasTarget() ? latestResult.getBotpose() : null;
    }

    /**
     * The AprilTag-derived field pose in Pedro's corner-origin frame, or {@code null}. Gated on a
     * fresh frame, the AprilTag pipeline, {@code getBotposeTagCount() >= 1} (the SDK returns an
     * all-zero botpose, never null, when no tag localised) and the field bounds. In BIOBUZZ the
     * tags move, so this is expected to stay {@code null}; it remains for a future static map.
     */
    public Pose getBotposeAsPedroPose() {
        if (!BOTPOSE_FRAME_VERIFIED) return null;
        if (!hasTarget()) return null;
        if (currentPipeline != APRILTAG_PIPELINE_INDEX) return null;
        if (latestResult.getBotposeTagCount() < 1) return null;

        Pose3D bp = latestResult.getBotpose();
        if (bp == null) return null;
        Position pos = bp.getPosition().toUnit(DistanceUnit.INCH);
        if (Double.isNaN(pos.x) || Double.isNaN(pos.y)) return null;

        double x = pos.x + FieldConstants.FIELD_CENTER_INCHES;
        double y = pos.y + FieldConstants.FIELD_CENTER_INCHES;
        if (!FieldConstants.isInsideField(x, y)) return null;

        YawPitchRollAngles ori = bp.getOrientation();
        double yaw = ori.getYaw(AngleUnit.RADIANS) + BOTPOSE_HEADING_OFFSET_RAD;
        return new Pose(x, y, yaw);
    }

    public int getBotposeTagCount() {
        return hasTarget() ? latestResult.getBotposeTagCount() : 0;
    }

    /** Age of the current result: capture plus targeting latency, in milliseconds. */
    public long getVisionLatencyMs() {
        if (latestResult == null) return 0;
        return (long) (latestResult.getCaptureLatency() + latestResult.getTargetingLatency());
    }

    // ---- Colour-blob detection ----

    public boolean activateBlobPipeline() {
        return switchPipeline(BLOB_PIPELINE_INDEX);
    }

    public boolean seesBlob() {
        return !blobDetections.isEmpty();
    }

    private double getBlobTx() {
        return primaryBlob == null ? 0 : primaryBlob.getTargetXDegrees();
    }

    private double getBlobTy() {
        return primaryBlob == null ? 0 : primaryBlob.getTargetYDegrees();
    }

    private static VisionMath.Mount mount() {
        return new VisionMath.Mount(CAMERA_HEIGHT_INCHES, CAMERA_PITCH_DEGREES,
                CAMERA_FORWARD_OFFSET_INCHES, CAMERA_LEFT_OFFSET_INCHES, CAMERA_YAW_OFFSET_DEGREES);
    }

    /**
     * True when several consecutive frames agree on where the blob is: a full window and a tight
     * one. A single frame is not enough to commit the robot to a path. Gate motion on this.
     */
    public boolean hasStableBlob() {
        return seesBlob()
                && txFilter.isReady()
                && tyFilter.isReady()
                && txFilter.spread() <= MAX_DETECTION_SPREAD_DEGREES
                && tyFilter.spread() <= MAX_DETECTION_SPREAD_DEGREES;
    }

    /** Median tx over the detection window; use this, not the raw frame, for aiming. */
    public double getFilteredBlobTx() {
        return txFilter.median();
    }

    public double getFilteredBlobTy() {
        return tyFilter.median();
    }

    public double getBlobSpreadDegrees() {
        return Math.max(txFilter.spread(), tyFilter.spread());
    }

    /** The largest blob's area as a percentage of the image, or NaN with none. */
    public double getBlobArea() {
        return primaryBlob == null ? Double.NaN : primaryBlob.getTargetArea();
    }

    /** Ground distance to the blob along the robot's forward axis, or NaN with no usable detection. */
    public double estimateBlobDistanceInches() {
        if (blobDetections.isEmpty()) return Double.NaN;
        double ty = tyFilter.getCount() > 0 ? tyFilter.median() : getBlobTy();
        double d = VisionMath.forwardDistanceInches(
                ty, CAMERA_PITCH_DEGREES, CAMERA_HEIGHT_INCHES - targetHeightInches);
        if (Double.isNaN(d) || d > MAX_VALID_DISTANCE_INCHES) return Double.NaN;
        return d;
    }

    /** Where the blob sits relative to the robot, as {@code {forward, left}} inches, or null. */
    public double[] estimateBlobInRobotFrame() {
        if (blobDetections.isEmpty()) return null;
        double tx = txFilter.getCount() > 0 ? txFilter.median() : getBlobTx();
        double ty = tyFilter.getCount() > 0 ? tyFilter.median() : getBlobTy();
        return VisionMath.targetInRobotFrame(
                tx, ty, mount(), targetHeightInches, MAX_VALID_DISTANCE_INCHES);
    }

    /**
     * Pose to drive to in order to pick up the blob, or {@code null} when there is no usable
     * detection: faces the piece but stops {@link #PICKUP_STANDOFF_INCHES} short. Never returns the
     * robot's own pose, which would build a zero-length path that silently does nothing.
     */
    public Pose estimateBlobApproachPose(Pose robotPose) {
        if (robotPose == null) return null;
        double[] robotFrame = estimateBlobInRobotFrame();
        if (robotFrame == null) return null;

        double heading = Angles.headingToward(robotPose.heading(), robotFrame[0], robotFrame[1]);
        double[] approach = VisionMath.applyStandoff(robotFrame, PICKUP_STANDOFF_INCHES);
        double[] field = VisionMath.toFieldFrame(robotPose.x(), robotPose.y(),
                robotPose.heading(), approach[0], approach[1]);

        if (!FieldConstants.isInsideField(field[0], field[1])) return null;
        return new Pose(field[0], field[1], heading);
    }
}
