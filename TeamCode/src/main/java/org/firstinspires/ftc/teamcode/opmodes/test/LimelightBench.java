package org.firstinspires.ftc.teamcode.opmodes.test;

import com.pedropathing.math.Pose;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.teamcode.game.Field;
import org.firstinspires.ftc.teamcode.opmodes.MatchOpMode;
import org.firstinspires.ftc.teamcode.subsystems.Shooter;
import org.firstinspires.ftc.teamcode.util.field.Alliance;
import org.firstinspires.ftc.teamcode.util.math.Angles;
import org.firstinspires.ftc.teamcode.util.time.MatchClock;

import java.util.List;

/**
 * Runs the Limelight 3A against one alliance's HIVE tags and turns what it sees into numbers a shot
 * needs: how far away the CELL is, which way to turn, what flywheel speed the table gives for that
 * distance, and where on the field that puts the robot. Drives nothing, so it is safe on a cart.
 *
 * <h2>Alliance</h2>
 * Picked during init with dpad left (RED) / right (BLUE). Only that alliance's tags are used (RED
 * 30-37, BLUE 38-45, from {@link Field#tagRange}); everything else in view is listed as ignored.
 *
 * <h2>Distance and the shot come from the tag, not from odometry</h2>
 * The tags of the CELL with the most tags in view are averaged into one point in the robot frame.
 * Its horizontal distance {@code d = hypot(x, y)} (height left out on purpose: the table is by floor
 * distance) feeds {@link Shooter#tableTargetFor}, and its bearing gives the turn that points the
 * rear shooter at it. This works with or without a tuned Pinpoint.
 *
 * <h2>Field position assumes where the CELL is</h2>
 * BIOBUZZ tags ride on moving CELLs and there is no official tag map, so the field pose here is
 * {@code Field.cell(...)} (a placeholder until measured) minus the tag's offset, rotated by the
 * robot's heading. Heading is the Pinpoint's when tuned, otherwise the tag's yaw plus
 * {@link #TAG_YAW_OFFSET_DEG}. It is only valid before that HIVE tips. It is shown next to the
 * Pinpoint pose so the two can be compared, and it is a test number: match code does not use it.
 *
 * <h2>Setup the Limelight needs</h2>
 * Pipeline {@code Limelight.APRILTAG_PIPELINE_INDEX} must be an AprilTag pipeline with 3D enabled,
 * and the camera's pose on the robot must be entered in the Limelight web UI; without it "robot
 * space" is really camera space and every distance is off by the mount offset.
 */
@TeleOp(name = "Bench: Limelight", group = "Bench")
public class LimelightBench extends MatchOpMode {
    /**
     * +1 if the Limelight's robot-space +Y points LEFT, -1 if it points right. Limelight documents
     * robot space as X forward, Y right, so the default converts to this codebase's left-positive
     * frame. The card checks it against {@code tx} (positive = target to the right) and says FLIP if
     * the two disagree.
     */
    public static double ROBOT_SPACE_Y_SIGN = -1;
    /**
     * Field heading = tag yaw (robot space) + this, for the untuned case only. Calibrate by facing
     * the robot at a known heading under the BLUE HIVE and setting this until the card agrees. The
     * RED value follows by the field's 180-degree rotation.
     */
    public static double TAG_YAW_OFFSET_DEG = 0;

    private Alliance side = Alliance.BLUE;

    @Override
    protected Alliance alliance() {
        return Alliance.BLUE;   // the Robot's own; this bench keeps its choice in `side`
    }

    @Override
    protected MatchClock.Period matchPeriod() {
        return MatchClock.Period.TELEOP;
    }

    @Override
    protected boolean usesScheduler() {
        return false;
    }

    @Override
    protected void onInitLoop() {
        if (gamepad1.dpadLeftWasPressed()) side = Alliance.RED;
        if (gamepad1.dpadRightWasPressed()) side = Alliance.BLUE;

        int[] audience = Field.tagRange(side, Field.CellSide.AUDIENCE);
        int[] far = Field.tagRange(side, Field.CellSide.FAR);
        telemetry.addData("Alliance", "%s   (dpad left = RED, right = BLUE)", side);
        telemetry.addData("Tags used", "audience CELL %d-%d, far CELL %d-%d",
                audience[0], audience[1], far[0], far[1]);
        telemetry.addData("Limelight", robot.limelight.getStatusLine());
        telemetry.addLine("Needs: 3D AprilTag pipeline + camera pose set in the Limelight web UI.");
        showTags();
    }

    @Override
    protected void onTelemetry() {
        telemetry.addData("Loop", "%.0f Hz", robot.getLoopHz());
        telemetry.addData("Alliance", side);
        telemetry.addData("Limelight", robot.limelight.getStatusLine());
        showTags();
    }

    /** The whole card below the header; shared by init and run so it can be checked before START. */
    private void showTags() {
        List<LLResultTypes.FiducialResult> tags = robot.limelight.getTags();

        // Sum the alliance's tags per CELL; the CELL with more tags in view is the one used.
        Sum audience = new Sum();
        Sum far = new Sum();
        StringBuilder ignored = new StringBuilder();
        for (LLResultTypes.FiducialResult tag : tags) {
            int id = tag.getFiducialId();
            Field.CellSide cell = cellOf(id);
            if (cell == null) {
                ignored.append(id).append(' ');
                continue;
            }
            (cell == Field.CellSide.AUDIENCE ? audience : far).add(tag);
        }

        telemetry.addLine();
        for (LLResultTypes.FiducialResult tag : tags) {
            if (cellOf(tag.getFiducialId()) == null) continue;
            Position p = inches(tag.getTargetPoseRobotSpace());
            telemetry.addData("  tag " + tag.getFiducialId(),
                    "tx %+.1f ty %+.1f  robot x %.1f y %.1f z %.1f in",
                    tag.getTargetXDegrees(), tag.getTargetYDegrees(), p.x, p.y, p.z);
        }
        if (ignored.length() > 0) telemetry.addData("  ignored (other alliance)", ignored.toString());

        Sum use = audience.n >= far.n ? audience : far;
        Field.CellSide cell = use == audience ? Field.CellSide.AUDIENCE : Field.CellSide.FAR;
        if (use.n == 0) {
            telemetry.addLine();
            telemetry.addLine("No " + side + " tag in view.");
            showPinpoint(null);
            return;
        }

        double x = use.x / use.n;                        // forward, inches
        double y = ROBOT_SPACE_Y_SIGN * use.y / use.n;   // left, inches
        double d = Math.hypot(x, y);
        double bearing = Math.atan2(y, x);               // CCW from the robot's forward axis
        double aimTurn = Angles.angleError(0, bearing - Shooter.HEADING_OFFSET_RAD);

        telemetry.addLine();
        telemetry.addData("CELL", "%s  (%d tag%s)", cell, use.n, use.n == 1 ? "" : "s");
        telemetry.addData("Distance", "%.1f in   (forward %.1f, left %.1f, up %.1f)",
                d, x, y, use.z / use.n);
        telemetry.addData("Bearing", "%+.1f deg  (CCW)", Math.toDegrees(bearing));
        telemetry.addData("Flywheel for d", "%.0f t/s  (Shooter table)", robot.shooter.tableTargetFor(d));
        telemetry.addData("Turn to aim rear", "%+.1f deg  (+ = CCW)", Math.toDegrees(aimTurn));
        // tx is positive to the right, so a tag to the left must have y > 0 here.
        double tx = use.tx / use.n;
        boolean axisAgrees = Math.abs(tx) < 2 || Math.signum(y) == -Math.signum(tx);
        telemetry.addData("Axis check", axisAgrees ? "OK" : "!! FLIP ROBOT_SPACE_Y_SIGN");

        // Field pose: the CELL's placeholder position, minus where the tag sits from the robot.
        Pose odometry = robot.drivetrain.getPose();
        double heading;
        String headingSource;
        if (odometry != null) {
            heading = odometry.heading();
            headingSource = "Pinpoint";
        } else {
            double yaw = Math.toRadians(ROBOT_SPACE_Y_SIGN * use.yawDeg / use.n + TAG_YAW_OFFSET_DEG);
            heading = Angles.normalizeAngle(side == Alliance.RED ? yaw + Math.PI : yaw);
            headingSource = "tag yaw (calibrate TAG_YAW_OFFSET_DEG)";
        }
        Pose cellPose = Field.cell(side, cell);
        double fieldBearing = heading + bearing;
        Pose fromTags = new Pose(cellPose.x() - d * Math.cos(fieldBearing),
                cellPose.y() - d * Math.sin(fieldBearing), heading);

        telemetry.addLine();
        telemetry.addData("Pose from tags", "x %.1f  y %.1f  heading %.1f deg",
                fromTags.x(), fromTags.y(), Math.toDegrees(fromTags.heading()));
        telemetry.addData("  heading from", headingSource);
        telemetry.addLine("  assumes the CELL is at its Field placeholder; invalid after a TIP");
        showPinpoint(fromTags);
    }

    private void showPinpoint(Pose fromTags) {
        Pose odometry = robot.drivetrain.getPose();
        if (odometry == null) {
            telemetry.addData("Pinpoint", "not tuned");
            return;
        }
        telemetry.addData("Pinpoint", "x %.1f  y %.1f  heading %.1f deg",
                odometry.x(), odometry.y(), Math.toDegrees(odometry.heading()));
        if (fromTags != null) {
            telemetry.addData("  tags - Pinpoint", "dx %+.1f  dy %+.1f in",
                    fromTags.x() - odometry.x(), fromTags.y() - odometry.y());
        }
    }

    /** Which of this alliance's CELLs a tag belongs to, or null for any other tag. */
    private Field.CellSide cellOf(int id) {
        for (Field.CellSide cell : Field.CellSide.values()) {
            int[] range = Field.tagRange(side, cell);
            if (id >= range[0] && id <= range[1]) return cell;
        }
        return null;
    }

    private static Position inches(Pose3D pose) {
        return pose.getPosition().toUnit(DistanceUnit.INCH);
    }

    /** Running sums of one CELL's tags, in the Limelight's robot space. */
    private static final class Sum {
        double x, y, z, tx, yawDeg;
        int n;

        void add(LLResultTypes.FiducialResult tag) {
            Pose3D pose = tag.getTargetPoseRobotSpace();
            Position p = inches(pose);
            x += p.x;
            y += p.y;
            z += p.z;
            tx += tag.getTargetXDegrees();
            yawDeg += pose.getOrientation().getYaw(AngleUnit.DEGREES);
            n++;
        }
    }
}
