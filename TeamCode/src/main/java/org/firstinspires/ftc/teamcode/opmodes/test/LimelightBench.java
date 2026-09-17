package org.firstinspires.ftc.teamcode.opmodes.test;

import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.LLStatus;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.subsystems.Limelight;

/**
 * What the code sees through the Limelight, pipeline by pipeline. The camera's own web UI shows
 * tx/ty/fps too; what only this can show is the code's verdicts (fresh or stale, stable blob or not)
 * and the mount check for fixthese C10: put a piece a measured distance in front of the camera and
 * compare the estimated forward/left inches with the tape measure. When they agree, set
 * {@code Limelight.MOUNT_CALIBRATED = true} and the collect/align macros come alive.
 */
@TeleOp(name = "Bench: Limelight", group = "Bench")
public class LimelightBench extends BenchOpMode {
    private static final String[] CONTROLS = {
            "dpad left: AprilTag pipeline    dpad right: blob pipeline",
            "Put a piece a measured distance ahead; compare 'estimate' with the tape measure.",
    };

    @Override
    protected String title() {
        return "BENCH: LIMELIGHT  (Limelight.java mount constants)";
    }

    @Override
    protected String[] controls() {
        return CONTROLS;
    }

    @Override
    protected void onBench() {
        Limelight ll = robot.limelight;
        if (gamepad1.dpadLeftWasPressed()) ll.activateAprilTagPipeline();
        if (gamepad1.dpadRightWasPressed()) ll.activateBlobPipeline();

        if (!ll.isAvailable()) {
            telemetry.addLine("MISSING from the configuration (Ethernet device 'limelight')");
            return;
        }
        LLStatus status = ll.getStatus();
        if (status != null) {
            telemetry.addData("Status", "%.0f fps  pipeline %d (%s)  temp %.0f  cpu %.0f  ram %.0f",
                    status.getFps(), status.getPipelineIndex(), status.getPipelineType(),
                    status.getTemp(), status.getCpu(), status.getRam());
        }
        telemetry.addData("Pipeline", "%s (%d)   frame %s", ll.getPipelineName(), ll.getPipelineIndex(),
                ll.hasTarget() ? "fresh" : (ll.isStale() ? "STALE" : "none"));
        telemetry.addData("Target", "tx %s  ty %s  ta %s", num(ll.getTx(), "%.1f"), num(ll.getTy(), "%.1f"), num(ll.getTa(), "%.1f"));

        telemetry.addData("Tags", "%d in frame", ll.getTagCount());
        for (LLResultTypes.FiducialResult tag : ll.getTags()) {
            telemetry.addData("  tag", "id %d  tx %.1f  ty %.1f", tag.getFiducialId(), tag.getTargetXDegrees(), tag.getTargetYDegrees());
        }

        telemetry.addData("Blob", "seen %s  stable %s  area %s  spread %.1f deg   tx %s ty %s",
                ll.seesBlob(), ll.hasStableBlob(), num(ll.getBlobArea(), "%.1f"), ll.getBlobSpreadDegrees(),
                num(ll.getFilteredBlobTx(), "%.1f"), num(ll.getFilteredBlobTy(), "%.1f"));
        double[] where = ll.estimateBlobInRobotFrame();
        telemetry.addData("Estimate", where == null ? "n/a" : fmt("forward %.1f in  left %.1f in   (tape measure?)", where[0], where[1]));
        telemetry.addData("Mount", "h %.1f  pitch %.1f  fwd %.1f  left %.1f  yaw %.1f   MOUNT_CALIBRATED %s",
                Limelight.CAMERA_HEIGHT_INCHES, Limelight.CAMERA_PITCH_DEGREES, Limelight.CAMERA_FORWARD_OFFSET_INCHES,
                Limelight.CAMERA_LEFT_OFFSET_INCHES, Limelight.CAMERA_YAW_OFFSET_DEGREES, Limelight.MOUNT_CALIBRATED);
    }
}
