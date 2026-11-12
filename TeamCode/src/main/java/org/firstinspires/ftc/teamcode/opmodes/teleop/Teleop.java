package org.firstinspires.ftc.teamcode.opmodes.teleop;

import com.pedropathing.ivy.Command;
import com.pedropathing.ivy.Scheduler;
import com.pedropathing.math.Pose;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.RobotLog;

import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.subsystems.Intake;
import org.firstinspires.ftc.teamcode.subsystems.Shooter;
import org.firstinspires.ftc.teamcode.util.field.Alliance;
import org.firstinspires.ftc.teamcode.util.field.PoseStorage;

import java.util.Locale;

/**
 * The match TeleOp. One gamepad (gamepad 1) runs everything. Drive is robot-centric; the Pinpoint
 * pose is only shown on the card. The shot leaves the FRONT, so face the HIVE to shoot.
 *
 * <pre>
 * INIT     dpad left/right  alliance RED / BLUE (which HIVE's tags the camera reads)
 * DRIVE    L-stick        drive / strafe (robot centric)
 *          R-stick X      turn
 *          LB (hold)      slow mode
 * INTAKE   RT (hold)      intake in
 *          LT (hold)      intake out          (neither, or both = off)
 * SHOOTER  X              flywheel on / off
 *          options        AUTO / MANUAL shooting (starts in AUTO; switching cancels a running shot)
 *          AUTO    RB     shoot one   (macro)
 *                  A      shoot four  (macro)
 *          B              cancel a running shot
 *          MANUAL  RB     hold = feed (intake FEED), only while the flywheel is on
 * SPEED    Y              TAG (Limelight distance, starts here) / FIXED
 *          dpad up/down   FIXED only: fixed speed +/- 25 t/s
 * </pre>
 *
 * MANUAL is the fallback for when the shot macro misbehaves: the driver spins up with X, watches
 * for READY on the card, and holds RB to feed balls in by hand.
 *
 * <p>Building the speed table: Y to FIXED, park facing the CELL, dpad until the shots land, then copy
 * the tag distance and the speed from the Speed line into {@code Shooter.SPEED_TABLE}. The dpad edits
 * {@code Shooter.TARGET_TICKS_PER_SEC}, a static: it lasts until the Robot Controller app restarts, so
 * it also changes the speed Auto HARDCODED shoots at.
 *
 * <p>The controller buzzes once when the flywheel first reaches speed after X, and three times when
 * the intake gives up un-jamming, so the driver does not have to look at the card.
 *
 * <p>Every loop: read sensors (this also sets the flywheel speed from the tag), then one section per
 * mechanism ({@link #drive()}, {@link #intake()}, {@link #shooter()}), then the shot macros
 * ({@code Scheduler.execute()}), then write the motors. The sections only set fields; the intake's
 * and shooter's default commands apply those fields every loop.
 */
@TeleOp(name = "Teleop", group = "Main")
public class Teleop extends OpMode {
    public static double SLOW_SCALE = 0.4;
    /** One dpad press in FIXED speed. */
    public static double SPEED_STEP_TICKS_PER_SEC = 25;

    private Robot robot;
    private Alliance alliance;

    /** What the triggers ask for this loop. Nothing latches: let go and it is OFF. */
    private Intake.Mode intakeFromTriggers = Intake.Mode.OFF;

    private boolean flywheelOn = false;
    /** False = AUTO (RB / A run the shot macros). True = MANUAL (hold RB to feed). */
    private boolean manualShooting = false;
    /** MANUAL only: RB held with the flywheel on. Runs the intake as FEED. */
    private boolean feeding = false;
    /** The controller has already buzzed for this spin-up / for this jam, so each buzzes only once. */
    private boolean readyBuzzed = false;
    private boolean jamBuzzed = false;
    /** The shot macro last started, so it can be cancelled. */
    private Command shot = null;

    @Override
    public void init() {
        Scheduler.reset();   // the scheduler is static and survives OpMode restarts
        robot = new Robot(hardwareMap);

        // Starts on the side picked last time (BLUE after an app restart); the dpad changes it in init_loop.
        alliance = PoseStorage.getAlliance(Alliance.BLUE);
        robot.setAlliance(alliance);

        // The intake and shooter follow these fields every loop. A shot macro takes both over while
        // it runs and hands them back when it ends.
        Scheduler.schedule(
                robot.intake.operatorControlCommand(() -> feeding ? Intake.Mode.FEED : intakeFromTriggers),
                robot.shooter.armedControlCommand(() -> flywheelOn));
    }

    @Override
    public void init_loop() {
        Alliance picked = alliance;
        if (gamepad1.dpadLeftWasPressed()) picked = Alliance.RED;
        if (gamepad1.dpadRightWasPressed()) picked = Alliance.BLUE;
        if (picked != alliance) {
            alliance = picked;
            PoseStorage.saveAlliance(alliance);   // the next Teleop init starts on this side
            robot.setAlliance(alliance);   // a network call to the camera: only on a change
        }

        telemetry.addLine("=========================");
        telemetry.addLine("   ALLIANCE:  " + alliance);
        telemetry.addLine("=========================");
        telemetry.addLine("dpad left = RED, right = BLUE");
        telemetry.addLine();
        telemetry.addLine("ONE GAMEPAD (gamepad 1). Drive is robot centric.");
        telemetry.addLine(String.format(Locale.US, "Speed starts on TAG; fixed speed %.0f t/s",
                Shooter.TARGET_TICKS_PER_SEC));
        telemetry.addLine();
        telemetry.addLine("DRIVE    sticks drive, LB hold = slow");
        telemetry.addLine("INTAKE   hold RT = in, hold LT = out");
        telemetry.addLine("SHOOTER  X flywheel on/off, options = AUTO/MANUAL");
        telemetry.addLine("  AUTO    RB shoot one, A shoot four, B cancel");
        telemetry.addLine("  MANUAL  hold RB to feed (flywheel must be on)");
        telemetry.addLine("SPEED    Y = TAG/FIXED, FIXED: dpad up/down +/-25");
        telemetry.addLine("BUZZ     once = flywheel ready, three = intake jammed");
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
        intake();
        shooter();

        Scheduler.execute();
        robot.writeActuators();
        showTelemetry();
    }

    @Override
    public void stop() {
        Scheduler.reset();
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

    // ---- Intake ----

    private void intake() {
        boolean in = gamepad1.right_trigger > 0.5;
        boolean out = gamepad1.left_trigger > 0.5;
        if (in && !out) intakeFromTriggers = Intake.Mode.IN;
        else if (out && !in) intakeFromTriggers = Intake.Mode.OUT;
        else intakeFromTriggers = Intake.Mode.OFF;   // neither, or both
    }

    // ---- Shooter ----

    private void shooter() {
        // Read every press once per loop, whatever the mode: the SDK keeps a press until it is read,
        // so one skipped in one mode would fire later in the other.
        boolean xPressed = gamepad1.xWasPressed();
        boolean yPressed = gamepad1.yWasPressed();
        boolean optionsPressed = gamepad1.optionsWasPressed();
        boolean rbPressed = gamepad1.rightBumperWasPressed();
        boolean aPressed = gamepad1.aWasPressed();
        boolean bPressed = gamepad1.bWasPressed();
        boolean upPressed = gamepad1.dpadUpWasPressed();
        boolean downPressed = gamepad1.dpadDownWasPressed();

        if (xPressed) flywheelOn = !flywheelOn;

        // Speed source. robot.readSensors() applies it from the next loop.
        if (yPressed) {
            robot.autoSpeed = !robot.autoSpeed;
            RobotLog.ii("Teleop", "speed -> %s", robot.autoSpeed ? "TAG" : "FIXED");
        }
        if (!robot.autoSpeed) {
            if (upPressed) Shooter.TARGET_TICKS_PER_SEC += SPEED_STEP_TICKS_PER_SEC;
            if (downPressed) {
                Shooter.TARGET_TICKS_PER_SEC = Math.max(0, Shooter.TARGET_TICKS_PER_SEC - SPEED_STEP_TICKS_PER_SEC);
            }
        }

        // Feel it instead of looking at the card: one buzz per spin-up when the flywheel reaches
        // speed, three blips when the intake gives up un-jamming.
        if (!flywheelOn) readyBuzzed = false;
        else if (!readyBuzzed && robot.shooter.atTarget()) {
            gamepad1.rumble(200);
            readyBuzzed = true;
        }
        boolean jammed = robot.intake.hasGivenUpUnjamming();
        if (jammed && !jamBuzzed) gamepad1.rumbleBlips(3);
        jamBuzzed = jammed;

        if (optionsPressed) {
            manualShooting = !manualShooting;
            RobotLog.ii("Teleop", "shooting mode -> %s", manualShooting ? "MANUAL" : "AUTO");
            cancelShot();
        }
        if (bPressed) cancelShot();

        if (manualShooting) {
            feeding = gamepad1.right_bumper && flywheelOn;
        } else {
            feeding = false;
            // B wins over a shot pressed in the same loop, and so does the mode switch.
            boolean mayShoot = !bPressed && !optionsPressed;
            if (rbPressed && mayShoot) startShot(robot.macros.shootOne());
            if (aPressed && mayShoot) startShot(robot.macros.shootAll());
        }
    }

    private void startShot(Command command) {
        if (shot != null && Scheduler.isScheduled(shot)) return;   // let the running shot finish
        shot = command;
        Scheduler.schedule(shot);
    }

    private void cancelShot() {
        if (shot != null && Scheduler.isScheduled(shot)) {
            RobotLog.ii("Teleop", "shot cancelled");
            Scheduler.cancel(shot);
        }
        shot = null;
    }

    // ---- Telemetry ----

    private void showTelemetry() {
        telemetry.addData("Loop", "%.0f Hz   battery %.1f V", robot.getLoopHz(), robot.getBatteryVolts());
        telemetry.addData("SHOOTING", manualShooting
                ? (feeding ? "MANUAL - FEEDING" : "MANUAL (hold RB to feed)")
                : "AUTO (RB one, A four, B cancel)");
        telemetry.addData("Speed", speedLine());
        telemetry.addData("Flywheel", robot.shooter.getStatusLine());
        telemetry.addData("Intake", robot.intake.getMode());
        if (!manualShooting) {
            telemetry.addData("Shot", "%s [%s]  (%d pulses)", robot.macros.getStatus(), robot.macros.getShotState(),
                    robot.macros.getShotsFired());
        }
        telemetry.addData("Limelight", "%s   (%s)", robot.limelight.getStatusLine(), alliance);
        Pose pose = robot.drivetrain.getPose();
        telemetry.addData("Pose", pose == null ? "none (run Pinpoint Tuner)"
                : String.format(Locale.US, "%.1f, %.1f @ %.0f deg", pose.x(), pose.y(),
                        Math.toDegrees(pose.heading())));

        if (manualShooting && !flywheelOn && gamepad1.right_bumper) {
            telemetry.addLine("!! flywheel is off: press X first");
        }
        if (robot.intake.hasGivenUpUnjamming()) telemetry.addLine("!! INTAKE JAMMED - run it out (LT)");
    }

    /** Where the flywheel speed comes from. In FIXED the tag distance is shown too: a speed table row. */
    private String speedLine() {
        double distance = robot.limelight.getShotDistanceInches();
        boolean tagInView = !Double.isNaN(distance);
        if (!robot.autoSpeed) {
            String line = String.format(Locale.US, "FIXED %.0f t/s (dpad up/down)", Shooter.TARGET_TICKS_PER_SEC);
            return tagInView ? line + String.format(Locale.US, "   tag %.0f in", distance) : line;
        }
        if (tagInView) {
            return String.format(Locale.US, "TAG  %.0f in -> %.0f t/s", distance, robot.shooter.getTarget());
        }
        // The shooter keeps the last tag speed for a moment (Shooter.TAG_HOLD_MS), then goes to fixed.
        return String.format(Locale.US, "TAG  no tag: %s %.0f t/s",
                robot.shooter.isSpeedFromTag() ? "holding" : "fixed", robot.shooter.getTarget());
    }
}
