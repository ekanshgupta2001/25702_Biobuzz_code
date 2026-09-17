package org.firstinspires.ftc.teamcode.opmodes.test;

import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.LLStatus;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.subsystems.Limelight;

/**
 * What the code sees through the Limelight: fps, frame freshness and every tag in view with its
 * tx, which is what {@code Macros.aimHeading} consumes. The camera's own web UI shows tx/ty/fps
 * too; what only this can show is the code's verdict (fresh or stale) and the IDs it will aim by.
 */
@TeleOp(name = "Bench: Limelight", group = "Bench")
public class LimelightBench extends BenchOpMode {
    private static final String[] CONTROLS = {
            "Point the camera at a HIVE CELL cluster: every tag in view is listed with its tx.",
            "The aim law uses the mean tx of the tags in the target CELL's ID range (game/Field).",
    };

    @Override
    protected String title() {
        return "BENCH: LIMELIGHT  (Limelight.java)";
    }

    @Override
    protected String[] controls() {
        return CONTROLS;
    }

    @Override
    protected void onBench() {
        Limelight ll = robot.limelight;
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
        telemetry.addData("Frame", ll.hasTarget() ? "fresh" : (ll.isStale() ? "STALE" : "none"));
        telemetry.addData("Target", "tx %s  ty %s  ta %s", num(ll.getTx(), "%.1f"), num(ll.getTy(), "%.1f"), num(ll.getTa(), "%.1f"));
        telemetry.addData("Tags", "%d in frame", ll.getTagCount());
        for (LLResultTypes.FiducialResult tag : ll.getTags()) {
            telemetry.addData("  tag", "id %d  tx %.1f  ty %.1f", tag.getFiducialId(), tag.getTargetXDegrees(), tag.getTargetYDegrees());
        }
        telemetry.addData("Mount", "yaw offset %.0f deg (0 = front, 180 = rear)", Limelight.CAMERA_YAW_OFFSET_DEGREES);
    }
}
