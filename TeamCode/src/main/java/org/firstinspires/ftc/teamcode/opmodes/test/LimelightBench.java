package org.firstinspires.ftc.teamcode.opmodes.test;

import com.pedropathing.math.Pose;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.teamcode.game.Field;
import org.firstinspires.ftc.teamcode.subsystems.Drivetrain;
import org.firstinspires.ftc.teamcode.subsystems.Limelight;
import org.firstinspires.ftc.teamcode.util.field.Alliance;
import org.firstinspires.ftc.teamcode.util.hardware.HardwareNames;

import java.util.List;
import java.util.Locale;

/**
 * Shows what the Limelight sees. For every AprilTag in view: which CELL it is on, the distance
 * along the floor, the angle (tx, + = right), and x / y from the robot (forward / left, inches).
 * Then the robot's field position worked back from the closest tag's CELL. Drives nothing.
 *
 * <h2>Pipelines: one per CELL, two per alliance</h2>
 * Each pipeline (set up in the Limelight web UI) finds only one CELL's four tags. The camera runs one
 * pipeline at a time, so only the selected alliance's two are ever used:
 * <ul>
 *   <li>gamepad 1 dpad left = RED, dpad right = BLUE. Picking an alliance starts on its starting
 *       up-CELL: BLUE pipeline 0, RED pipeline 3;</li>
 *   <li>dpad down = the alliance's other CELL (what a HIVE tip does): BLUE 0 / 1, RED 3 / 2.</li>
 * </ul>
 * The card shows the pipeline asked for and the one the camera says it is running. A tag that is not
 * on the active pipeline's CELL is marked, which means that pipeline's ID filter is wrong.
 *
 * <p>Needs the camera's position on the robot entered in the Limelight web UI, or every distance is
 * off by the mount offset. The field position also needs Pedro tuned (for the heading), uses the
 * CELL's placeholder position in {@code game/Field}, and is wrong once that HIVE tips. Test number only.
 */
@TeleOp(name = "Bench: Limelight", group = "Bench")
public class LimelightBench extends OpMode {
    /**
     * Limelight's robot space is X forward, Y to the RIGHT; ours is Y to the left, so Y is flipped.
     * If a tag on the robot's left shows a negative y here, set this to +1.
     */
    public static double ROBOT_SPACE_Y_SIGN = -1;

    /** The pipeline numbers in the Limelight web UI, one per CELL. */
    public static int BLUE_FAR_PIPELINE = 0;        // tags 42-45, blue's starting up-CELL
    public static int BLUE_AUDIENCE_PIPELINE = 1;   // tags 38-41
    public static int RED_FAR_PIPELINE = 2;         // tags 30-33
    public static int RED_AUDIENCE_PIPELINE = 3;    // tags 34-37, red's starting up-CELL

    private Limelight limelight;
    /** The same camera the subsystem holds, for switching pipelines. */
    private Limelight3A camera;
    private Drivetrain drivetrain;

    private Alliance alliance = Alliance.BLUE;
    private Field.CellSide side = Field.startingUpCellSide(Alliance.BLUE);
    private int activePipeline = -1;

    @Override
    public void init() {
        limelight = new Limelight(hardwareMap);
        camera = hardwareMap.get(Limelight3A.class, HardwareNames.LIMELIGHT);
        drivetrain = new Drivetrain(hardwareMap);   // only for the heading; nothing is driven
    }

    @Override
    public void loop() {
        pickPipeline();
        limelight.update();
        drivetrain.update();
        telemetry.addData("Limelight", limelight.getStatusLine());

        int[] range = Field.tagRange(alliance, side);
        telemetry.addLine("ALLIANCE: " + alliance + "   (dpad left = RED, right = BLUE)");
        telemetry.addLine(String.format(Locale.US, "Pipeline %d = %s %s CELL, tags %d-%d   (dpad down = other CELL)",
                activePipeline, alliance, side, range[0], range[1]));
        telemetry.addData("Camera is running", cameraPipeline());
        telemetry.addLine();

        List<LLResultTypes.FiducialResult> tags = limelight.getTags();
        if (tags.isEmpty()) {
            telemetry.addLine("No AprilTag in view");
            return;
        }

        LLResultTypes.FiducialResult closest = null;
        double closestDistance = Double.MAX_VALUE;
        for (LLResultTypes.FiducialResult tag : tags) {
            int id = tag.getFiducialId();
            double x = forward(tag);
            double y = left(tag);
            double distance = Math.hypot(x, y);   // along the floor: the tag's height is left out
            Cell cell = cellOf(id);
            boolean onThisPipeline = id >= range[0] && id <= range[1];
            telemetry.addData("Tag " + id,
                    "%s   distance %.1f in   angle %+.1f deg   x %.1f  y %.1f in%s",
                    cell == null ? "not a CELL tag" : cell.name, distance, tag.getTargetXDegrees(), x, y,
                    onThisPipeline ? "" : "   <- not this pipeline");
            if (distance < closestDistance) {
                closestDistance = distance;
                closest = tag;
            }
        }
        showPoseFromTags(tags, cellOf(closest.getFiducialId()));
    }

    /** Reads the dpad and switches the camera when the wanted pipeline changes. */
    private void pickPipeline() {
        if (gamepad1.dpadLeftWasPressed() && alliance != Alliance.RED) {
            alliance = Alliance.RED;
            side = Field.startingUpCellSide(alliance);
        }
        if (gamepad1.dpadRightWasPressed() && alliance != Alliance.BLUE) {
            alliance = Alliance.BLUE;
            side = Field.startingUpCellSide(alliance);
        }
        if (gamepad1.dpadDownWasPressed()) side = side.opposite();

        int wanted = pipelineFor(alliance, side);
        if (wanted == activePipeline) return;
        try {
            camera.pipelineSwitch(wanted);
        } catch (RuntimeException e) {
            // Camera not answering; the Limelight line on the card says so.
        }
        activePipeline = wanted;
    }

    private static int pipelineFor(Alliance alliance, Field.CellSide side) {
        if (alliance == Alliance.BLUE) {
            return side == Field.CellSide.FAR ? BLUE_FAR_PIPELINE : BLUE_AUDIENCE_PIPELINE;
        }
        return side == Field.CellSide.FAR ? RED_FAR_PIPELINE : RED_AUDIENCE_PIPELINE;
    }

    /** The pipeline the camera says it is on, from its latest fresh frame. */
    private String cameraPipeline() {
        try {
            LLResult result = camera.getLatestResult();
            if (result == null || result.getStaleness() > Limelight.MAX_STALENESS_MS) return "no fresh frame";
            int index = result.getPipelineIndex();
            return "pipeline " + index + (index == activePipeline ? "" : "   (switching...)");
        } catch (RuntimeException e) {
            return "not answering";
        }
    }

    /**
     * Robot position = CELL position minus where the CELL sits from the robot, turned into the field
     * frame by the robot's heading. Uses the average of that CELL's tags in view, since the cluster
     * of four sits around the CELL's centre.
     */
    private void showPoseFromTags(List<LLResultTypes.FiducialResult> tags, Cell cell) {
        telemetry.addLine();
        Pose odometry = drivetrain.getPose();
        if (cell == null) {
            telemetry.addLine("Pose from tags: the closest tag is not on a CELL");
            return;
        }
        if (odometry == null) {
            telemetry.addLine("Pose from tags: needs Pedro tuned (for the heading)");
            return;
        }

        double x = 0, y = 0;
        int n = 0;
        for (LLResultTypes.FiducialResult tag : tags) {
            Cell other = cellOf(tag.getFiducialId());
            if (other == null || !other.name.equals(cell.name)) continue;
            x += forward(tag);
            y += left(tag);
            n++;
        }
        x /= n;
        y /= n;

        double h = odometry.heading();
        double fieldX = cell.pose.x() - (x * Math.cos(h) - y * Math.sin(h));
        double fieldY = cell.pose.y() - (x * Math.sin(h) + y * Math.cos(h));

        telemetry.addData("Pose from tags", "x %.1f  y %.1f  (%s, %d tag%s)",
                fieldX, fieldY, cell.name, n, n == 1 ? "" : "s");
        telemetry.addData("Pinpoint pose", "x %.1f  y %.1f  heading %.1f deg",
                odometry.x(), odometry.y(), Math.toDegrees(h));
        telemetry.addData("Difference", "dx %+.1f  dy %+.1f in", fieldX - odometry.x(), fieldY - odometry.y());
        telemetry.addLine("  uses the Pinpoint heading: only meaningful if the pose is a field pose (e.g. after Auto)");
    }

    private static Position inches(LLResultTypes.FiducialResult tag) {
        return tag.getTargetPoseRobotSpace().getPosition().toUnit(DistanceUnit.INCH);
    }

    private static double forward(LLResultTypes.FiducialResult tag) {
        return inches(tag).x;
    }

    private static double left(LLResultTypes.FiducialResult tag) {
        return ROBOT_SPACE_Y_SIGN * inches(tag).y;
    }

    /** The CELL a tag is stuck under, or null for any other tag. */
    private static Cell cellOf(int id) {
        for (Alliance alliance : Alliance.values()) {
            for (Field.CellSide side : Field.CellSide.values()) {
                int[] range = Field.tagRange(alliance, side);
                if (id >= range[0] && id <= range[1]) {
                    return new Cell(alliance + " " + side, Field.cell(alliance, side));
                }
            }
        }
        return null;
    }

    private static final class Cell {
        final String name;
        final Pose pose;

        Cell(String name, Pose pose) {
            this.name = name;
            this.pose = pose;
        }
    }

    @Override
    public void stop() {
        if (limelight != null) limelight.stop();
    }
}
