package org.firstinspires.ftc.teamcode.opmodes.test;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.teamcode.commands.Macros;
import org.firstinspires.ftc.teamcode.game.PieceType;
import org.firstinspires.ftc.teamcode.subsystems.ColorSensor;
import org.firstinspires.ftc.teamcode.subsystems.Storage;

/**
 * The storage transport on its own. X fires one {@code Macros.SENSORLESS_FEED_PULSE_MS} pulse, the
 * exact command the sensorless shooting cycle uses, so the pulse can be tuned by watching how far
 * one piece travels; LB/RB change it in 50 ms steps and the value shown is the one to paste into
 * {@code Macros}. The dpad sets the count so the full-storage interlock can be checked. The
 * "in view" counter says for how many loops a passing piece was seen by the entrance sensor: fewer
 * than two and the edge counting will miss pieces at match speed.
 */
@TeleOp(name = "Bench: Storage", group = "Bench")
public class StorageBench extends BenchOpMode {
    private static final String[] CONTROLS = {
            "hold A advance / B reverse; release = stop",
            "X: one SENSORLESS_FEED_PULSE_MS advance pulse;  LB/RB: pulse -/+ 50 ms",
            "dpad up/down: count +/- 1      dpad right: flip the second transport motor's direction (live)",
    };

    private long pulseUntilMs = -1;
    private int inViewLoops = 0;
    private int longestInView = 0;

    @Override
    protected String title() {
        return "BENCH: STORAGE  (Storage.java, Macros.SENSORLESS_FEED_PULSE_MS)";
    }

    @Override
    protected String[] controls() {
        return CONTROLS;
    }

    @Override
    protected void onBench() {
        Storage storage = robot.storage;
        Macros.SENSORLESS_FEED_PULSE_MS = (long) adjust(Macros.SENSORLESS_FEED_PULSE_MS,
                gamepad1.rightBumperWasPressed(), gamepad1.leftBumperWasPressed(), 50, 100, 3000);
        if (gamepad1.dpadUpWasPressed()) storage.setCount(storage.count() + 1);
        if (gamepad1.dpadDownWasPressed()) storage.setCount(storage.count() - 1);
        if (gamepad1.dpadRightWasPressed()) {
            Storage.SECOND_MOTOR_DIRECTION = Storage.SECOND_MOTOR_DIRECTION == DcMotorSimple.Direction.FORWARD
                    ? DcMotorSimple.Direction.REVERSE : DcMotorSimple.Direction.FORWARD;
        }
        if (gamepad1.xWasPressed()) pulseUntilMs = nowMs + Macros.SENSORLESS_FEED_PULSE_MS;

        boolean pulsing = pulseUntilMs >= 0 && nowMs < pulseUntilMs;
        if (!pulsing) pulseUntilMs = -1;
        if (gamepad1.a || pulsing) storage.advance();
        else if (gamepad1.b) storage.reverse();
        else storage.stop();

        // How many consecutive loops the entrance sensor sees a piece: the edge-count sampling check.
        boolean seen = robot.storageEntranceSensor.isAvailable() && PieceType.anyAtSensor(robot.storageEntranceSensor);
        inViewLoops = seen ? inViewLoops + 1 : 0;
        if (seen && inViewLoops > longestInView) longestInView = inViewLoops;

        telemetry.addData("Count", "%d / %d  %s  (dpad)", storage.count(), Storage.CAPACITY,
                storage.isFull() ? "FULL" : "");
        telemetry.addData("Events", "entered %d  exited %d   sensors: entrance %s  exit %s",
                storage.getEntryEvents(), storage.getExitEvents(),
                storage.hasEntranceSensor() ? "yes" : "NO", storage.hasExitSensor() ? "yes" : "NO");
        telemetry.addData("Mode", "%s%s", storage.getMode(), pulsing ? "  (pulse)" : "");
        telemetry.addData("Pulse", "%d ms  (LB/RB; paste into Macros.SENSORLESS_FEED_PULSE_MS)", Macros.SENSORLESS_FEED_PULSE_MS);
        telemetry.addData("Velocity", "target %.0f  measured %.0f t/s   %.2f A",
                storage.getTargetVelocity(), storage.getVelocityTicksPerSec(), storage.getCurrentAmps());
        telemetry.addData("Second motor", storage.hasSecondMotor()
                ? fmt("%s  measured %.0f t/s  (dpad right flips; opposite sign to the first = fighting)",
                        Storage.SECOND_MOTOR_DIRECTION, storage.getSecondVelocityTicksPerSec())
                : "not fitted");
        telemetry.addData("Entrance", sensorLine(robot.storageEntranceSensor));
        telemetry.addData("Full", sensorLine(robot.storageFullSensor));
        telemetry.addData("Transfer (exit)", sensorLine(robot.transferSensor));
        telemetry.addData("In view", "%d loops now, longest %d  (need >= 2 to count reliably)", inViewLoops, longestInView);
    }

    static String sensorLine(ColorSensor sensor) {
        if (!sensor.isAvailable()) return "MISSING";
        return fmt("%s  hue %.0f sat %.2f val %.2f  dist %s in", PieceType.classify(sensor),
                sensor.getHue(), sensor.getSaturation(), sensor.getValue(), num(sensor.getDistanceInches(), "%.1f"));
    }
}
