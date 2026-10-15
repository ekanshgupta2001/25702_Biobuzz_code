package org.firstinspires.ftc.teamcode.opmodes.test;

import com.pedropathing.math.Pose;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.teamcode.game.Field;
import org.firstinspires.ftc.teamcode.subsystems.Drivetrain;
import org.firstinspires.ftc.teamcode.subsystems.Limelight;
import org.firstinspires.ftc.teamcode.util.field.Alliance;

import java.util.List;
import java.util.Locale;

/**
 * Shows what the Limelight sees. For every AprilTag in view: which CELL it is on, the distance
 * along the floor, the angle (tx, + = right), and x / y from the robot (forward / left, inches).
 * Then the robot's field position worked back from the closest tag's CELL. Drives nothing.
 *
 * <h2>Pipelines: one per alliance</h2>
 * Each pipeline (set up in the Limelight web UI) finds only its alliance's eight tags, both CELLs:
 * BLUE = pipeline 0 (tags 38-45), RED = pipeline 1 (tags 30-37). Gamepad 1 dpad left = RED,
 * dpad right = BLUE; the switch goes through {@code Limelight.setAlliance}, the same call Teleop and
 * Auto make. The card shows the pipeline asked for and the one the camera says it is running. A tag
 * that is not the selected alliance's is marked, which means that pipeline's ID filter is wrong.
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

    private Limelight limelight;
    private Drivetrain drivetrain;

    private Alliance alliance = Alliance.BLUE;

    @Override
    public void init() {
        limelight = new Limelight(hardwareMap);
        limelight.setAlliance(alliance);
        drivetrain = new Drivetrain(hardwareMap);   // only for the heading; nothing is driven
    }

    @Override
    public void loop() {
        pickAlliance();
        limelight.update();
        drivetrain.update();
        telemetry.addData("Limelight", limelight.getStatusLine());

        int[] range = allianceTagRange(alliance);
        telemetry.addLine("ALLIANCE: " + alliance + "   (dpad left = RED, right = BLUE)");
        telemetry.addLine(String.format(Locale.US, "Pipeline %d = %s, tags %d-%d (both %s CELLs)",
                Limelight.pipelineFor(alliance), alliance, range[0], range[1], alliance));
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
            boolean onThisPipeline = cell != null && cell.alliance == alliance;
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

    /** Reads the dpad and switches the camera when the alliance changes. */
    private void pickAlliance() {
        Alliance wanted = alliance;
        if (gamepad1.dpadLeftWasPressed()) wanted = Alliance.RED;
        if (gamepad1.dpadRightWasPressed()) wanted = Alliance.BLUE;
        if (wanted == alliance) return;
        alliance = wanted;
        limelight.setAlliance(alliance);
    }

    /** Both of the alliance's CELLs together: the IDs its pipeline should pass. */
    private static int[] allianceTagRange(Alliance alliance) {
        int[] audience = Field.tagRange(alliance, Field.CellSide.AUDIENCE);
        int[] far = Field.tagRange(alliance, Field.CellSide.FAR);
        return new int[] {Math.min(audience[0], far[0]), Math.max(audience[1], far[1])};
    }

    /** The pipeline the camera says it is on, from its latest fresh frame. */
    private String cameraPipeline() {
        if (!limelight.isConnected()) return "not answering";
        int index = limelight.getPipelineIndex();
        if (index < 0) return "no fresh frame";
        return "pipeline " + index + (index == Limelight.pipelineFor(alliance) ? "" : "   (switching...)");
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
                    return new Cell(alliance, alliance + " " + side, Field.cell(alliance, side));
                }
            }
        }
        return null;
    }

    private static final class Cell {
        final Alliance alliance;
        final String name;
        final Pose pose;

        Cell(Alliance alliance, String name, Pose pose) {
            this.alliance = alliance;
            this.name = name;
            this.pose = pose;
        }
    }

    @Override
    public void stop() {
        if (limelight != null) limelight.stop();
    }
}
