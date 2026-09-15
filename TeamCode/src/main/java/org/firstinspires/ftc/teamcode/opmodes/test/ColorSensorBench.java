package org.firstinspires.ftc.teamcode.opmodes.test;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.game.PieceType;
import org.firstinspires.ftc.teamcode.subsystems.ColorSensor;
import org.firstinspires.ftc.teamcode.util.hardware.HardwareNames;

/**
 * The four colour-sensor points, one at a time. The SDK's TestHardware utility shows raw values;
 * this shows what the code makes of them: the HSV the classifier sees, which hue window matches,
 * the NECTAR alliance, and the presence verdict against {@code Robot.PRESENCE_DISTANCE_INCHES}.
 * Hold Y with a piece in place, then with none, and read the min/max off the card: that is where
 * {@code PieceType}'s hue windows and floors and the presence distance come from.
 */
@TeleOp(name = "Bench: Color sensors", group = "Bench")
public class ColorSensorBench extends BenchOpMode {
    private static final String[] CONTROLS = {
            "dpad left/right: select the sensor point",
            "LB/RB: gain -/+ 0.5     X: light on/off (REV V3 has none)",
            "hold Y: sample min/max of hue, sat, val, distance     B: reset the sample",
    };

    private int selected = 0;
    private final Peak hue = new Peak();
    private final Peak sat = new Peak();
    private final Peak val = new Peak();
    private final Peak dist = new Peak();

    @Override
    protected String title() {
        return "BENCH: COLOR SENSORS  (PieceType.java, Robot.PRESENCE_DISTANCE_INCHES)";
    }

    @Override
    protected String[] controls() {
        return CONTROLS;
    }

    private ColorSensor[] sensors() {
        return new ColorSensor[] {robot.storageEntranceSensor, robot.storageFullSensor,
                robot.transferSensor, robot.shooterFeedSensor};
    }

    private static final String[] NAMES = {
            HardwareNames.SENSOR_STORAGE_ENTRANCE, HardwareNames.SENSOR_STORAGE_FULL,
            HardwareNames.SENSOR_TRANSFER, HardwareNames.SENSOR_SHOOTER_FEED};

    @Override
    protected void onBench() {
        if (gamepad1.dpadRightWasPressed()) selected = (selected + 1) % NAMES.length;
        if (gamepad1.dpadLeftWasPressed()) selected = (selected + NAMES.length - 1) % NAMES.length;
        ColorSensor sensor = sensors()[selected];

        if (gamepad1.rightBumperWasPressed()) sensor.setGain(sensor.getGain() + 0.5f);
        if (gamepad1.leftBumperWasPressed()) sensor.setGain(Math.max(0.5f, sensor.getGain() - 0.5f));
        if (gamepad1.xWasPressed()) sensor.setLight(!sensor.isLightOn());
        if (gamepad1.bWasPressed()) {
            hue.reset();
            sat.reset();
            val.reset();
            dist.reset();
        }
        if (gamepad1.y && sensor.isAvailable()) {
            hue.add(sensor.getHue());
            sat.add(sensor.getSaturation());
            val.add(sensor.getValue());
            dist.add(sensor.getDistanceInches());
        }

        telemetry.addData("Point", "%d/%d  %s  (dpad)", selected + 1, NAMES.length, NAMES[selected]);
        if (!sensor.isAvailable()) {
            telemetry.addLine("MISSING from the configuration");
            return;
        }
        telemetry.addData("RGBA", "%.3f %.3f %.3f %.3f   gain %.1f  light %s",
                sensor.getRed(), sensor.getGreen(), sensor.getBlue(), sensor.getAlpha(), sensor.getGain(),
                sensor.hasLight() ? (sensor.isLightOn() ? "on" : "off") : "n/a");
        telemetry.addData("HSV", "hue %.0f  sat %.2f  val %.2f   (floors: sat %.2f val %.2f, window +/-%.0f)",
                sensor.getHue(), sensor.getSaturation(), sensor.getValue(),
                PieceType.MIN_SATURATION, PieceType.MIN_VALUE, PieceType.HUE_TOLERANCE_DEGREES);
        telemetry.addData("Classify", "%s   nectar alliance %s", PieceType.classify(sensor), PieceType.nectarAllianceAt(sensor));
        telemetry.addData("Windows", "POLLEN %s   NECTAR red %s  blue %s",
                PieceType.POLLEN.isAtSensor(sensor),
                sensor.matchesHue(PieceType.NECTAR_RED_HUE_DEGREES, PieceType.HUE_TOLERANCE_DEGREES, PieceType.MIN_SATURATION, PieceType.MIN_VALUE),
                sensor.matchesHue(PieceType.NECTAR_BLUE_HUE_DEGREES, PieceType.HUE_TOLERANCE_DEGREES, PieceType.MIN_SATURATION, PieceType.MIN_VALUE));
        double inches = sensor.getDistanceInches();
        telemetry.addData("Distance", "%s in   presence (<= %.1f) %s", num(inches, "%.2f"), Robot.PRESENCE_DISTANCE_INCHES,
                sensor.hasDistance() ? String.valueOf(!Double.isNaN(inches) && inches <= Robot.PRESENCE_DISTANCE_INCHES) : "no distance sensor");
        telemetry.addData("Sample (hold Y)", "hue %s  sat %s  val %s  dist %s",
                hue.status("%.0f"), sat.status("%.2f"), val.status("%.2f"), dist.status("%.1f"));
    }
}
