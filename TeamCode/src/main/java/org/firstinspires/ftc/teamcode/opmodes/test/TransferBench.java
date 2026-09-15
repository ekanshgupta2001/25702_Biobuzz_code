package org.firstinspires.ftc.teamcode.opmodes.test;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.commands.Macros;
import org.firstinspires.ftc.teamcode.subsystems.Transfer;

/**
 * The vertical transfer on its own. Y fires one {@code Macros.SENSORLESS_FEED_PULSE_MS} feed pulse,
 * the same command the sensorless shooting cycle uses; run it with a piece in the lift and the
 * flywheel off to see whether one pulse moves exactly one piece to the top. LB/RB tune it.
 */
@TeleOp(name = "Bench: Transfer", group = "Bench")
public class TransferBench extends BenchOpMode {
    private static final String[] CONTROLS = {
            "hold A lift / B feed / X reverse; release = stop",
            "Y: one SENSORLESS_FEED_PULSE_MS feed pulse;  LB/RB: pulse -/+ 50 ms",
    };

    private long pulseUntilMs = -1;

    @Override
    protected String title() {
        return "BENCH: TRANSFER  (Transfer.java, Macros.SENSORLESS_FEED_PULSE_MS)";
    }

    @Override
    protected String[] controls() {
        return CONTROLS;
    }

    @Override
    protected void onBench() {
        Transfer transfer = robot.transfer;
        Macros.SENSORLESS_FEED_PULSE_MS = (long) adjust(Macros.SENSORLESS_FEED_PULSE_MS,
                gamepad1.rightBumperWasPressed(), gamepad1.leftBumperWasPressed(), 50, 100, 3000);
        if (gamepad1.yWasPressed()) pulseUntilMs = nowMs + Macros.SENSORLESS_FEED_PULSE_MS;

        boolean pulsing = pulseUntilMs >= 0 && nowMs < pulseUntilMs;
        if (!pulsing) pulseUntilMs = -1;
        if (pulsing || gamepad1.b) transfer.feed();
        else if (gamepad1.a) transfer.liftPiece();
        else if (gamepad1.x) transfer.reverse();
        else transfer.stop();

        telemetry.addData("Mode", "%s%s", transfer.getMode(), pulsing ? "  (pulse)" : "");
        telemetry.addData("Pulse", "%d ms  (LB/RB; paste into Macros.SENSORLESS_FEED_PULSE_MS)", Macros.SENSORLESS_FEED_PULSE_MS);
        telemetry.addData("Velocity", "target %.0f  measured %.0f t/s   %.2f A",
                transfer.getTargetVelocity(), transfer.getVelocityTicksPerSec(), transfer.getCurrentAmps());
        telemetry.addData("Sensors", "feed sensor %s   in lift %s   at feed %s",
                transfer.hasFeedSensor() ? "yes" : "NO (timed pulses)", transfer.hasPieceInLift(),
                transfer.hasFeedSensor() ? String.valueOf(transfer.pieceAtFeed()) : "n/a");
        telemetry.addData("Transfer point", StorageBench.sensorLine(robot.transferSensor));
        telemetry.addData("Feed point", StorageBench.sensorLine(robot.shooterFeedSensor));
    }
}
