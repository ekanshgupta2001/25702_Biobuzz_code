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

/**
 * Shows what the Limelight sees. For every AprilTag in view: which CELL it is on, the distance
 * along the floor, the angle (tx, + = right), and x / y from the robot (forward / left, inches).
 * Then the robot's field position worked back from the closest tag's CELL. Drives nothing.
 *
 * <p>Needs pipeline {@code Limelight.APRILTAG_PIPELINE_INDEX} set up as a 3D AprilTag pipeline, and
 * the camera's position on the robot entered in the Limelight web UI, or every distance is off by
 * the mount offset. The field position also needs Pedro tuned (for the heading), uses the CELL's
 * placeholder position in {@code game/Field}, and is wrong once that HIVE tips. Test number only.
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

    @Override
    public void init() {
        limelight = new Limelight(hardwareMap);
        drivetrain = new Drivetrain(hardwareMap);   // only for the heading; nothing is driven
    }

    @Override
    public void loop() {
        limelight.update();
        drivetrain.update();
        telemetry.addData("Limelight", limelight.getStatusLine());

        List<LLResultTypes.FiducialResult> tags = limelight.getTags();
        if (tags.isEmpty()) {
            telemetry.addLine("No AprilTag in view");
            return;
        }

        LLResultTypes.FiducialResult closest = null;
        double closestDistance = Double.MAX_VALUE;
        for (LLResultTypes.FiducialResult tag : tags) {
            double x = forward(tag);
            double y = left(tag);
            double distance = Math.hypot(x, y);   // along the floor: the tag's height is left out
            Cell cell = cellOf(tag.getFiducialId());
            telemetry.addData("Tag " + tag.getFiducialId(),
                    "%s   distance %.1f in   angle %+.1f deg   x %.1f  y %.1f in",
                    cell == null ? "not a CELL tag" : cell.name, distance, tag.getTargetXDegrees(), x, y);
            if (distance < closestDistance) {
                closestDistance = distance;
                closest = tag;
            }
        }
        showPoseFromTags(tags, cellOf(closest.getFiducialId()));
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
