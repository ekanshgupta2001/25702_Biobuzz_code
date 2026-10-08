package org.firstinspires.ftc.teamcode.opmodes.teleop;

import com.pedropathing.ivy.Command;
import com.pedropathing.ivy.Scheduler;
import com.pedropathing.math.Pose;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.game.Field;
import org.firstinspires.ftc.teamcode.subsystems.Intake;
import org.firstinspires.ftc.teamcode.subsystems.Limelight;
import org.firstinspires.ftc.teamcode.subsystems.Shooter;
import org.firstinspires.ftc.teamcode.util.field.Alliance;
import org.firstinspires.ftc.teamcode.util.field.PoseStorage;
import org.firstinspires.ftc.teamcode.util.math.Angles;

import java.util.Locale;

/**
 * The match TeleOp, for both alliances. Pick the side during init on gamepad 1's dpad
 * (left = RED, right = BLUE); it starts on the side the last autonomous ran.
 *
 * <pre>
 * DRIVER (gamepad 1)                          OPERATOR (gamepad 2)
 *   L-stick / R-stick X  drive / turn            RB / LB / both    intake in / out / stop (+ cancel shot)
 *   L-trigger (hold)     slow mode               L-trigger         flywheel on / off
 *   R-trigger (hold)     aim the shooter         R-trigger / A     shoot one / shoot four
 *   options              field / robot centric   dpad up           speed from distance / fixed speed
 *   Y                    re-zero heading         dpad left / right fixed speed - / +
 *                        (face away from wall)   dpad down         our HIVE tipped: aim at the other CELL
 * </pre>
 *
 * Every loop: read sensors, handle both gamepads, run the shot macros ({@code Scheduler.execute()}),
 * then write the motors. Everything that needs a pose (field centric, aiming, speed from distance)
 * switches itself off until Pedro is tuned.
 */
@TeleOp(name = "Teleop", group = "Main")
public class Teleop extends OpMode {
    public static double SLOW_SCALE = 0.4;
    /** Aiming: turn power per radian of heading error, and the most it may turn. */
    public static double AIM_P = 1.5;
    public static double AIM_MAX_TURN = 0.5;
    /** How long a Limelight sighting keeps correcting the aim after the tags leave view. */
    public static long TAG_CORRECTION_MS = 5000;
    public static double SPEED_STEP_TICKS_PER_SEC = 100;

    private Robot robot;
    private Alliance alliance;

    private Intake.Mode intakeMode = Intake.Mode.OFF;
    /** Both bumpers were pressed: ignore a single bumper until both are let go. */
    private boolean waitForBumperRelease = false;
    private boolean flywheelOn = false;
    private boolean fixedSpeed = false;
    /** The shot macro last started, so both bumpers can cancel it. */
    private Command shot = null;

    /** How many times our HIVE has tipped; the up-facing CELL flips each time. */
    private int tips = 0;
    private Pose cell;
    private int minTag, maxTag;

    private boolean hadAutoPose = false;

    /** Camera bearing minus odometry bearing to the CELL, radians, and when it was measured (0 = never). */
    private double tagCorrection = 0;
    private long tagCorrectionAtMs = 0;

    @Override
    public void init() {
        Scheduler.reset();   // the scheduler is static and survives OpMode restarts
        robot = new Robot(hardwareMap);
        alliance = PoseStorage.getAlliance(Alliance.BLUE);
        hadAutoPose = PoseStorage.hasPose();
        if (hadAutoPose) {
            robot.drivetrain.setPose(PoseStorage.getPose());
            // Used once: a second Teleop must not snap back to where auto ended.
            PoseStorage.clearPose();
        }

        // The intake and flywheel follow these two fields every loop. A shot macro takes both over
        // while it runs and hands them back when it ends.
        Scheduler.schedule(
                robot.intake.operatorControlCommand(() -> intakeMode),
                robot.shooter.armedControlCommand(() -> flywheelOn));
    }

    @Override
    public void init_loop() {
        if (gamepad1.dpadLeftWasPressed()) alliance = Alliance.RED;
        if (gamepad1.dpadRightWasPressed()) alliance = Alliance.BLUE;

        telemetry.addLine("ALLIANCE: " + alliance + "   (gamepad 1 dpad: left = RED, right = BLUE)");
        telemetry.addLine(robot.drivetrain.hasFollower()
                ? "Pedro tuned"
                : "Pedro NOT tuned: robot centric, no aiming, fixed flywheel speed");
        telemetry.addLine(hadAutoPose ? "Using the pose auto left"
                : "No pose from auto: press Y once, facing away from your wall");
        telemetry.addLine("If auto tipped our HIVE: operator dpad down once");
        telemetry.addLine();
        telemetry.addLine("DRIVER: sticks drive, LT slow, RT aim, options field/robot, Y re-zero heading");
        telemetry.addLine("OPERATOR: RB in, LB out, both stop, LT flywheel, RT shoot 1, A shoot 4");
        telemetry.addLine("  dpad up distance/fixed speed, left/right speed -/+, down HIVE tipped");
    }

    @Override
    public void start() {
        // *WasPressed() remembers presses until read: forget anything bumped during init.
        gamepad1.resetEdgeDetection();
        gamepad2.resetEdgeDetection();
        robot.drivetrain.setDriverHeadingOffset(Field.driverForwardHeading(alliance));
        pickTargetCell();
        robot.drivetrain.onStart();
    }

    @Override
    public void loop() {
        robot.readSensors();
        Pose pose = robot.drivetrain.getPose();   // null until Pedro is tuned

        // Not while aiming: the robot is turning then, and a camera frame is a few loops old, so a
        // sample taken mid-turn would be off by the turn rate times the camera delay.
        boolean aiming = gamepad1.right_trigger > 0.5;
        if (!aiming) updateTagCorrection(pose);
        driver(pose, aiming);
        operator(pose);

        Scheduler.execute();
        robot.writeActuators();
        showTelemetry(pose);
    }

    @Override
    public void stop() {
        Scheduler.reset();
        // No motor writes here: the SDK rejects them from stop() and stops the motors itself.
        if (robot != null) robot.stop();
    }

    // ---- Driver ----

    private void driver(Pose pose, boolean aiming) {
        if (gamepad1.optionsWasPressed()) robot.drivetrain.toggleFieldCentric();
        if (gamepad1.yWasPressed()) {
            robot.drivetrain.resetHeading();
            tagCorrectionAtMs = 0;   // it was measured against the old heading
        }

        double scale = gamepad1.left_trigger > 0.5 ? SLOW_SCALE : 1.0;
        // The SDK reports stick-up as negative; Pedro wants +forward, +strafe left, +turn CCW.
        double forward = -gamepad1.left_stick_y * scale;
        double strafe = -gamepad1.left_stick_x * scale;
        double turn = -gamepad1.right_stick_x * scale;

        if (aiming && pose != null) turn = aimTurn(pose);

        robot.drivetrain.drive(forward, strafe, turn);
    }

    // ---- Aiming ----
    // The shooter fires out the rear, so aiming means turning the robot's back to the CELL. Odometry
    // gives the bearing. The front camera can refine it, but it only sees the CELL's tags while the
    // robot faces the CELL, i.e. before it turns around. So while the tags are in view we remember
    // how far the camera disagrees with odometry, and keep adding that for TAG_CORRECTION_MS.

    private double bearingToCell(Pose pose) {
        return Math.atan2(cell.y() - pose.y(), cell.x() - pose.x());
    }

    private void updateTagCorrection(Pose pose) {
        if (pose == null) return;
        double tx = robot.limelight.getTagTx(minTag, maxTag);   // degrees, + = right; NaN = not seen
        if (Double.isNaN(tx)) return;
        double cameraBearing = pose.heading() + Math.toRadians(Limelight.CAMERA_YAW_OFFSET_DEGREES - tx);
        tagCorrection = Angles.angleError(bearingToCell(pose), cameraBearing);
        tagCorrectionAtMs = System.currentTimeMillis();
    }

    private boolean hasTagCorrection() {
        return tagCorrectionAtMs != 0 && System.currentTimeMillis() - tagCorrectionAtMs < TAG_CORRECTION_MS;
    }

    /** Turn power that swings the rear-firing shooter onto the CELL. */
    private double aimTurn(Pose pose) {
        double bearing = bearingToCell(pose) + (hasTagCorrection() ? tagCorrection : 0);
        double wanted = bearing - Shooter.HEADING_OFFSET_RAD;
        double error = Angles.angleError(pose.heading(), wanted);   // + = turn CCW
        return Range.clip(AIM_P * error, -AIM_MAX_TURN, AIM_MAX_TURN);
    }

    // ---- Operator ----

    private void operator(Pose pose) {
        // Nobody lets go of two bumpers in the same loop, so after a "both" press the single bumper
        // released last is ignored until both are up; otherwise it would switch the intake back on.
        boolean in = gamepad2.right_bumper;
        boolean out = gamepad2.left_bumper;
        if (in && out) {
            intakeMode = Intake.Mode.OFF;
            cancelShot();
            waitForBumperRelease = true;
        } else if (!in && !out) {
            waitForBumperRelease = false;
        } else if (!waitForBumperRelease) {
            intakeMode = in ? Intake.Mode.IN : Intake.Mode.OUT;
        }

        if (gamepad2.leftTriggerWasPressed()) flywheelOn = !flywheelOn;

        if (gamepad2.dpadUpWasPressed()) fixedSpeed = !fixedSpeed;
        if (gamepad2.dpadRightWasPressed()) Shooter.MANUAL_TICKS_PER_SEC += SPEED_STEP_TICKS_PER_SEC;
        if (gamepad2.dpadLeftWasPressed()) {
            Shooter.MANUAL_TICKS_PER_SEC = Math.max(0, Shooter.MANUAL_TICKS_PER_SEC - SPEED_STEP_TICKS_PER_SEC);
        }
        if (gamepad2.dpadDownWasPressed()) {
            tips++;
            pickTargetCell();
        }

        // Flywheel speed, every loop: from the distance to the CELL, or the fixed speed when asked
        // for or when there is no pose. Set before a shot starts so it fires at this speed.
        if (fixedSpeed || pose == null) robot.shooter.setManualTarget();
        else robot.shooter.setTargetForDistance(pose.distance(cell));

        if (gamepad2.rightTriggerWasPressed()) startShot(robot.macros.shootOne());
        if (gamepad2.aWasPressed()) startShot(robot.macros.shootAll());
    }

    private void startShot(Command command) {
        if (shot != null && Scheduler.isScheduled(shot)) return;   // let the running shot finish
        shot = command;
        Scheduler.schedule(shot);
    }

    private void cancelShot() {
        if (shot != null && Scheduler.isScheduled(shot)) Scheduler.cancel(shot);
        shot = null;
    }

    /** The CELL to shoot at and its tag IDs. Flips each time our HIVE tips. */
    private void pickTargetCell() {
        Field.CellSide side = Field.upCellSide(alliance, tips);
        cell = Field.cell(alliance, side);
        int[] tags = Field.tagRange(alliance, side);
        minTag = tags[0];
        maxTag = tags[1];
        tagCorrectionAtMs = 0;   // it was measured against the old CELL
    }

    // ---- Telemetry ----

    private void showTelemetry(Pose pose) {
        telemetry.addData("Loop", "%.0f Hz   battery %.1f V", robot.getLoopHz(), robot.getBatteryVolts());
        telemetry.addData("Alliance", "%s   CELL %s (tags %d-%d), %d tip(s)",
                alliance, Field.upCellSide(alliance, tips), minTag, maxTag, tips);
        telemetry.addData("Drive", robot.drivetrain.isFieldCentric() ? "field centric" : "robot centric");
        telemetry.addData("Pose", pose == null ? "none (Pedro not tuned)"
                : String.format(Locale.US, "x %.1f  y %.1f  heading %.1f deg",
                        pose.x(), pose.y(), Math.toDegrees(pose.heading())));
        telemetry.addData("Aim", hasTagCorrection()
                ? String.format(Locale.US, "tag-corrected %+.1f deg", Math.toDegrees(tagCorrection))
                : "odometry only");

        telemetry.addData("Flywheel", robot.shooter.getStatusLine());
        telemetry.addData("Speed", fixedSpeed || pose == null
                ? String.format(Locale.US, "fixed %.0f t/s", Shooter.MANUAL_TICKS_PER_SEC)
                : String.format(Locale.US, "from distance, %.0f in", pose.distance(cell)));
        telemetry.addData("Intake", intakeMode);
        telemetry.addData("Shot", "%s  (%d pulses)", robot.macros.getStatus(), robot.macros.getShotsFired());

        if (robot.intake.hasGivenUpUnjamming()) telemetry.addLine("!! INTAKE JAMMED - run it out (LB)");
        if (!robot.limelight.isConnected()) telemetry.addLine("!! no Limelight: odometry aim only");
    }
}
