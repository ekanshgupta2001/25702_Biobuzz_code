package org.firstinspires.ftc.teamcode.opmodes.teleop;

import com.pedropathing.math.Pose;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.game.FieldPoses;
import org.firstinspires.ftc.teamcode.util.field.Alliance;
import org.firstinspires.ftc.teamcode.util.field.FieldConstants;
import org.firstinspires.ftc.teamcode.util.field.AllianceStorage;

import java.util.Locale;

/**
 * The match TeleOp. One gamepad (gamepad 1) runs everything. Drive is robot-centric. The shot leaves
 * the FRONT, so face the HIVE to shoot.
 *
 * <pre>
 * INIT     dpad left/right  alliance RED / BLUE
 * DRIVE    L-stick          drive / strafe (robot centric)
 *          R-stick X        turn
 *          LB (hold)        slow mode
 * SHOOTER  X                shooter on / off (flywheel + counter-roller)
 * INTAKE   RT (hold)        shooter off: intake in
 *                           shooter on:  intake + feeder = SHOOT, for as long as it is held
 *          LT (hold)        intake out, and the feeder backwards (backs a stuck ball out)
 * </pre>
 *
 * The wheel speeds are automatic: {@code robot.readSensors()} sets them every loop from the distance
 * the Limelight reads to our CELL's tag (the Distance line on the card shows the number it used).
 * No tag for a second = the fixed speeds in {@code Shooter}.
 *
 * <p>The controller buzzes once when both wheels first reach speed after X, and three times when the
 * intake gives up un-jamming.
 *
 * <p>Every loop, top to bottom: read sensors, {@link #drive()}, {@link #shooter()}, {@link #intake()},
 * write the motors, show the card. There are no commands and no scheduler here: each section tells
 * its mechanism what to do this loop, and that is all that happens.
 */
@TeleOp(name = "Teleop one driver", group = "Main")
public class Teleop extends OpMode {
    public static double SLOW_SCALE = 0.4;

    private Robot robot;
    private Alliance alliance;

    /** X toggles it. True = both wheels hold their speeds, and RT shoots. */
    private boolean shooterOn = false;

    /** The controller has already buzzed for this spin-up / for this jam, so each buzzes only once. */
    private boolean readyBuzzed = false;
    private boolean jamBuzzed = false;

    @Override
    public void init() {
        robot = new Robot(hardwareMap);

        // Starts on the side picked last time (BLUE after an app restart); the dpad changes it in init_loop.
        alliance = AllianceStorage.get(Alliance.BLUE);
        robot.setAlliance(alliance);
        setStartPose();
    }

    /**
     * Where the robot is when Teleop starts: the alliance's start spot (the hardcoded auto ends close
     * to it). It does not have to be exact: the Limelight corrects the shot distance as soon as
     * it sees the CELL. Does nothing until the Pinpoint is set up ({@code Constants.localizerConfig}).
     */
    private void setStartPose() {
        robot.drivetrain.setPose(FieldConstants.forAlliance(FieldPoses.BLUE_START_FACING_HIVE, alliance));
    }

    @Override
    public void init_loop() {
        robot.readSensors();   // so the card below shows what the camera sees before START

        Alliance picked = alliance;
        if (gamepad1.dpadLeftWasPressed()) picked = Alliance.RED;
        if (gamepad1.dpadRightWasPressed()) picked = Alliance.BLUE;
        if (picked != alliance) {
            alliance = picked;
            AllianceStorage.save(alliance);   // the next Teleop init starts on this side
            robot.setAlliance(alliance);          // a network call to the camera: only on a change
            setStartPose();
        }

        telemetry.addLine("=========================");
        telemetry.addLine("   ALLIANCE:  " + alliance);
        telemetry.addLine("=========================");
        telemetry.addLine("dpad left = RED, right = BLUE");
        telemetry.addLine();
        telemetry.addLine("DRIVE    sticks drive, LB hold = slow");
        telemetry.addLine("SHOOTER  X = on / off");
        telemetry.addLine("INTAKE   hold RT = in (shooter on: SHOOT), hold LT = out");
        telemetry.addLine("BUZZ     once = wheels ready, three = intake jammed");
        telemetry.addLine();
        // The pre-match check: face the CELL and these two lines should show a tag and a distance.
        telemetry.addData("Limelight", "%s   (%s)", robot.limelight.getStatusLine(), alliance);
        telemetry.addData("Distance", distanceLine());
        telemetry.addData("Battery", "%.1f V", robot.getBatteryVolts());
    }

    @Override
    public void start() {
        // *WasPressed() remembers presses until read: forget anything bumped during init.
        gamepad1.resetEdgeDetection();
        robot.drivetrain.onStart();
    }

    @Override
    public void loop() {
        robot.readSensors();

        drive();
        shooter();
        intake();

        robot.writeActuators();
        showTelemetry();
    }

    @Override
    public void stop() {
        if (robot != null) robot.limelight.stop();   // or the camera keeps being polled after the OpMode
        // No motor writes here: the SDK rejects them from stop() and stops the motors itself.
    }

    // ---- Drive ----

    private void drive() {
        double scale = gamepad1.left_bumper ? SLOW_SCALE : 1.0;
        // The SDK reports stick-up as negative; Pedro wants +forward, +strafe left, +turn CCW.
        double forward = -gamepad1.left_stick_y * scale;
        double strafe = -gamepad1.left_stick_x * scale;
        double turn = -gamepad1.right_stick_x * scale;
        robot.drivetrain.drive(forward, strafe, turn);
    }

    // ---- Shooter: X on / off ----

    private void shooter() {
        if (gamepad1.yWasPressed()) {
            shooterOn = !shooterOn;
            // Off: stops both wheels and the feeder right now. Only on the press, not every loop,
            // so it does not undo LT's feeder-backwards in intake() below.
            if (!shooterOn) robot.shooter.disarm();
        }
        if (shooterOn){
            robot.shooter.arm();
            gamepad1.rumbleBlips(3);
        }

        // One buzz per spin-up, when both wheels reach speed.
        if (!shooterOn) {
            readyBuzzed = false;
        } else if (!readyBuzzed && robot.shooter.atTarget()) {
            gamepad1.rumbleBlips(3);
            readyBuzzed = true;
        }
    }

    // ---- Intake and feeder: the triggers ----

    private void intake() {
        boolean in = gamepad1.right_trigger > 0.5;
        boolean out = gamepad1.left_trigger > 0.5;

        if (in && !out && shooterOn) {
            // SHOOT: the intake and the feeder carry balls into the wheels, nonstop while RT is held.
            robot.intake.feed();
            robot.shooter.feederOn();
        } else if (out && !in) {
            // OUT: the intake and the feeder both run backwards, so a stuck ball comes back down.
            robot.intake.out();
            robot.shooter.feederOut();
        } else {
            robot.shooter.feederOff();
            if (in && !out) robot.intake.in();
            else robot.intake.stop();   // neither, or both
        }

        // Three blips when the intake gives up un-jamming.
        boolean jammed = robot.intake.hasGivenUpUnjamming();
        if (jammed && !jamBuzzed) gamepad1.rumbleBlips(3);
        jamBuzzed = jammed;
    }

    // ---- Telemetry ----

    private void showTelemetry() {
        telemetry.addData("Loop", "%.0f Hz   battery %.1f V", robot.getLoopHz(), robot.getBatteryVolts());
        telemetry.addData("Shooter", "%s   %s", shooterOn ? "ON (X = off)" : "OFF (X = on)",
                robot.shooter.getStatusLine());
        telemetry.addData("Intake", "%s%s", robot.intake.getMode(),
                robot.shooter.isFeederOn() ? " + feeder = SHOOTING"
                        : robot.shooter.isFeederOut() ? " + feeder backwards" : "");
        telemetry.addData("Distance", distanceLine());
        telemetry.addData("Limelight", "%s   (%s)", robot.limelight.getStatusLine(), alliance);
        Pose pose = robot.drivetrain.getPose();
        telemetry.addData("Pose", pose == null ? "none (Pinpoint not set up)"
                : String.format(Locale.US, "%.1f, %.1f @ %.0f deg", pose.x(), pose.y(),
                        Math.toDegrees(pose.heading())));

        if (robot.isPositionOff()) {
            telemetry.addLine("!! POSITION OFF: the Limelight moved the distance more than "
                    + Math.round(Robot.CORRECTION_WARN_INCHES) + " in");
        }
        if (robot.intake.hasGivenUpUnjamming()) telemetry.addLine("!! INTAKE JAMMED - run it out (LT)");
    }

    /**
     * The distance the wheel speeds are set from, and what it is made of, e.g.
     * {@code 74 in = position 71 + correction 3   tag 74   target FAR}.
     */
    private String distanceLine() {
        double shot = robot.getShotInches();
        double tag = robot.getTagInches();
        String tagText = Double.isNaN(tag) ? "no tag" : String.format(Locale.US, "tag %.0f", tag);

        if (Double.isNaN(robot.getPositionInches())) {
            // No Pinpoint: the tag is the only distance there is. (The shooter keeps its last table
            // speeds for Shooter.TAG_HOLD_MS after the tag is lost; its own line says table / fixed.)
            return Double.isNaN(shot) ? "NO TAG of ours in view: fixed speeds"
                    : String.format(Locale.US, "%.0f in  TAG ONLY   (tag now %.0f)", shot, tag);
        }
        return String.format(Locale.US, "%.0f in = position %.0f + correction %.0f   %s   target %s",
                shot, robot.getPositionInches(), robot.getCorrectionInches(), tagText,
                robot.getTargetCell());
    }
}
