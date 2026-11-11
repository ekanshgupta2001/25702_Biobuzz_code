package org.firstinspires.ftc.teamcode.opmodes.teleop;

import com.pedropathing.ivy.Command;
import com.pedropathing.ivy.Scheduler;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.RobotLog;

import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.subsystems.Intake;
import org.firstinspires.ftc.teamcode.subsystems.Shooter;

import java.util.Locale;

/**
 * The match TeleOp. One gamepad (gamepad 1) runs everything. Drive is robot-centric only: there are
 * no odometry pods, so nothing here uses a pose.
 *
 * <pre>
 * DRIVE    L-stick        drive / strafe (robot centric)
 *          R-stick X      turn
 *          LT (hold)      slow mode
 * INTAKE   RB / LB        intake in / out (stays on until changed)
 *          RB + LB        intake stop, and cancels a running shot
 * SHOOTER  X              flywheel on / off
 *          options        AUTO / MANUAL shooting (starts in AUTO; switching cancels a running shot)
 *          AUTO    RT     shoot one   (macro)
 *                  A      shoot four  (macro)
 *          MANUAL  RT     hold = feed: intake in + up-wheels on, only while the flywheel is on
 * </pre>
 *
 * MANUAL is the fallback for when the shot macro misbehaves: the driver spins up with X, watches
 * for READY on the card, and holds RT to feed balls in by hand.
 *
 * <p>The controller buzzes once when both wheels first reach speed after X, and three times when the
 * intake gives up un-jamming, so the driver does not have to look at the card.
 *
 * <p>Every loop: read sensors, then one section per mechanism ({@link #drive()},
 * {@link #intake()}, {@link #shooter()}), then the shot macros ({@code Scheduler.execute()}), then
 * write the motors. The sections only set fields; the intake's and shooter's default commands apply
 * those fields every loop.
 */
@TeleOp(name = "Teleop", group = "Main")
public class Teleop extends OpMode {
    public static double SLOW_SCALE = 0.4;

    private Robot robot;

    /** The intake as the bumpers last left it. */
    private Intake.Mode intakeMode = Intake.Mode.OFF;
    /** Both bumpers were pressed: ignore a single bumper until both are let go. */
    private boolean waitForBumperRelease = false;

    private boolean flywheelOn = false;
    /** False = AUTO (RT / A run the shot macros). True = MANUAL (hold RT to feed). */
    private boolean manualShooting = false;
    /** MANUAL only: RT held with the flywheel on. Runs the intake in and the up-wheels. */
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

        // The intake and shooter follow these fields every loop. A shot macro takes both over while
        // it runs and hands them back when it ends.
        Scheduler.schedule(
                robot.intake.operatorControlCommand(() -> feeding ? Intake.Mode.FEED : intakeMode),
                robot.shooter.armedControlCommand(() -> flywheelOn, () -> feeding));
    }

    @Override
    public void init_loop() {
        telemetry.addLine("ONE GAMEPAD (gamepad 1). Drive is robot centric.");
        telemetry.addLine(String.format(Locale.US, "Shooter speeds: fly %.0f  roller %.0f t/s",
                Shooter.TARGET_TICKS_PER_SEC, Shooter.ROLLER_TARGET_TICKS_PER_SEC));
        telemetry.addLine();
        telemetry.addLine("DRIVE    sticks drive, LT hold = slow");
        telemetry.addLine("INTAKE   RB in, LB out, both = stop (+ cancel shot)");
        telemetry.addLine("SHOOTER  X flywheel on/off, options = AUTO/MANUAL");
        telemetry.addLine("  AUTO    RT shoot one, A shoot four");
        telemetry.addLine("  MANUAL  hold RT to feed (flywheel must be on)");
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
        double scale = gamepad1.left_trigger > 0.5 ? SLOW_SCALE : 1.0;
        // The SDK reports stick-up as negative; Pedro wants +forward, +strafe left, +turn CCW.
        double forward = -gamepad1.left_stick_y * scale;
        double strafe = -gamepad1.left_stick_x * scale;
        double turn = -gamepad1.right_stick_x * scale;
        robot.drivetrain.drive(forward, strafe, turn);
    }

    // ---- Intake ----

    private void intake() {
        // Nobody lets go of two bumpers in the same loop, so after a "both" press the single bumper
        // released last is ignored until both are up; otherwise it would switch the intake back on.
        boolean in = gamepad1.right_bumper;
        boolean out = gamepad1.left_bumper;
        if (in && out) {
            intakeMode = Intake.Mode.OFF;
            cancelShot();
            waitForBumperRelease = true;
        } else if (!in && !out) {
            waitForBumperRelease = false;
        } else if (!waitForBumperRelease) {
            intakeMode = in ? Intake.Mode.IN : Intake.Mode.OUT;
        }
    }

    // ---- Shooter ----

    private void shooter() {
        if (gamepad1.xWasPressed()) flywheelOn = !flywheelOn;

        // Feel it instead of looking at the card: one buzz per spin-up when both wheels reach speed,
        // three blips when the intake gives up un-jamming.
        if (!flywheelOn) readyBuzzed = false;
        else if (!readyBuzzed && robot.shooter.atTarget()) {
            gamepad1.rumble(200);
            readyBuzzed = true;
        }
        boolean jammed = robot.intake.hasGivenUpUnjamming();
        if (jammed && !jamBuzzed) gamepad1.rumbleBlips(3);
        jamBuzzed = jammed;

        if (gamepad1.optionsWasPressed()) {
            manualShooting = !manualShooting;
            RobotLog.ii("Teleop", "shooting mode -> %s", manualShooting ? "MANUAL" : "AUTO");
            cancelShot();
        }

        // Read both edges every loop, whichever mode is on, so a press in one mode does not fire
        // later in the other.
        boolean rtPressed = gamepad1.rightTriggerWasPressed();
        boolean aPressed = gamepad1.aWasPressed();

        if (manualShooting) {
            feeding = gamepad1.right_trigger > 0.5 && flywheelOn;
        } else {
            feeding = false;
            if (rtPressed) startShot(robot.macros.shootOne());
            if (aPressed) startShot(robot.macros.shootAll());
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
                ? (feeding ? "MANUAL - FEEDING" : "MANUAL (hold RT to feed)")
                : "AUTO (RT one, A four)");
        telemetry.addData("Flywheel", robot.shooter.getStatusLine());
        telemetry.addData("Intake", feeding ? "IN (feeding)" : intakeMode.toString());
        if (!manualShooting) {
            telemetry.addData("Shot", "%s [%s]  (%d pulses)", robot.macros.getStatus(), robot.macros.getShotState(),
                    robot.macros.getShotsFired());
        }

        if (manualShooting && !flywheelOn && gamepad1.right_trigger > 0.5) {
            telemetry.addLine("!! flywheel is off: press X first");
        }
        if (robot.intake.hasGivenUpUnjamming()) telemetry.addLine("!! INTAKE JAMMED - run it out (LB)");
    }
}