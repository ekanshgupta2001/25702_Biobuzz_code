package org.firstinspires.ftc.teamcode.opmodes.teleop;

import com.pedropathing.ivy.Command;
import com.pedropathing.ivy.Scheduler;
import com.pedropathing.math.Pose;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.commands.Macros;
import org.firstinspires.ftc.teamcode.game.Field;
import org.firstinspires.ftc.teamcode.game.FieldPoses;
import org.firstinspires.ftc.teamcode.opmodes.MatchOpMode;
import org.firstinspires.ftc.teamcode.subsystems.Storage;
import org.firstinspires.ftc.teamcode.util.field.Alliance;
import org.firstinspires.ftc.teamcode.util.field.FieldConstants;
import org.firstinspires.ftc.teamcode.util.field.PoseStorage;
import org.firstinspires.ftc.teamcode.util.math.DriveScaling;
import org.firstinspires.ftc.teamcode.util.time.MatchClock;

import java.util.Locale;

/**
 * The match TeleOp.
 *
 * <p>Driving is itself an Ivy command ({@code Drivetrain.driverControlCommand}, which feeds Pedro's
 * {@code follower.manual(...)} with field-centric mixing and a heading hold) rather than code in the
 * loop. That is what makes macros safe: scheduling one that needs the drivetrain suspends driver
 * control through the scheduler, and ending or cancelling it restores driver control automatically.
 * There is no "am I in a macro?" flag for the loop to get wrong.
 *
 * <p>Pedro in teleop: sticks through {@code manual}, heading hold (and the aim lock riding on it) through a Pedro controller, snap
 * turns through {@code hold}, and two one-button paths ({@link Controls#DRIVE_TO_SHOOT},
 * {@link Controls#DRIVE_TO_PARK}) built from the current pose with {@code Paths.line}.
 *
 * <p>The lifecycle lives in {@link MatchOpMode}. What remains here is only what makes this OpMode
 * teleop: the bindings (one {@link Controls.Snapshot} per loop), macro launch and abort, the aim
 * lock the driver holds on the right trigger (the shooter is fixed, so aiming is the drivetrain's
 * heading hold pointed at the HIVE by {@code Macros.aimHeading}), the flywheel arm that comes back
 * by itself after a macro preempts it, haptics on transitions, and the two telemetry modes.
 *
 * <p>Season facts enter only through {@code game/}: which CELL to aim at
 * ({@code Field.upCellSide} after the TIPs the operator has counted), its tag range, and the
 * shooting and park poses, all authored for BLUE and rotated by {@code FieldConstants.forAlliance}.
 */
@TeleOp(name = "Teleop", group = "Main")
public class Teleop extends MatchOpMode {
    /** Stick deflection that counts as "the driver wants control back" and aborts a drive macro. */
    public static double MACRO_ABORT_STICK = 0.25;
    /**
     * Whether to show the full engineering readout. Off during a match: nobody reads twenty lines of
     * subsystem state while driving, and the few things that matter get lost among them.
     */
    public static boolean DEBUG_TELEMETRY = false;
    /** Below this, the pack is sagging enough to change how the robot drives. Warn the drivers. */
    public static double LOW_BATTERY_VOLTS = 11.5;
    /** Right-trigger pull past this holds the aim lock. */
    public static double AIM_LOCK_TRIGGER = 0.5;
    /** How often init retries an AprilTag fix; the pipeline switch and read are camera round-trips. */
    public static long LOCALIZE_RETRY_MS = 500;

    private static final int RUMBLE_SUCCESS_BLIPS = 1;
    /** Distinguishable from success without looking. */
    private static final int RUMBLE_FAILURE_BLIPS = 3;
    private static final int RUMBLE_FULL_BLIPS = 2;
    private static final int RUMBLE_FINAL_BLIPS = 2;

    private Alliance alliance = Alliance.BLUE;
    /** Where the current alliance came from, for the init card: default, auto, or the dpad. */
    private String allianceSource = "default";
    private boolean inheritedPose = false;
    private boolean localized = false;
    private boolean localizeTried = false;
    private long lastLocalizeTryMs = 0;
    /** TIPs of our HIVE the operator has counted; the up-CELL flips on each. */
    private int tipsCounted = 0;

    private Command activeMacro = null;
    private boolean flywheelArmed = false;
    private Command spinHold = null;

    // Previous values, for firing haptics on the transition rather than continuously.
    private Macros.Outcome lastOutcome = Macros.Outcome.IDLE;
    private int lastCount = 0;
    private boolean announcedFull = false;
    private boolean announcedFinal = false;

    @Override
    protected String logTag() {
        return "teleop";
    }

    @Override
    protected MatchClock.Period matchPeriod() {
        return MatchClock.Period.TELEOP;
    }

    // ---- Lifecycle hooks ----

    @Override
    protected void onInit() {
        // Default commands: priority -1, SUSPEND, QUEUE. Any command that needs the resource
        // preempts them and they resume by themselves when it ends. The sticks are read live by the
        // suppliers, not from the per-loop snapshot: analog values are never consumed.
        // Exactly one drive default. Before Pedro is tuned there is no follower, so the sticks go
        // straight to the motors through OpenLoopDrive: robot-centric, no heading hold, and the
        // drive macros stay off (see handleDriver). Robot builds only one of the two motor layers.
        if (robot.drivetrain.isAvailable()) {
            robot.drivetrain.driverControlCommand(
                    () -> shaped(Controls.DRIVE_FORWARD),
                    () -> shaped(Controls.DRIVE_STRAFE),
                    () -> shaped(Controls.DRIVE_TURN)).schedule();
        } else {
            robot.openLoopDrive.driverControlCommand(
                    () -> shaped(Controls.DRIVE_FORWARD),
                    () -> shaped(Controls.DRIVE_STRAFE),
                    () -> shaped(Controls.DRIVE_TURN)).schedule();
        }
        robot.intake.defaultIdleCommand().schedule();
        robot.storage.defaultIdleCommand().schedule();
        robot.transfer.defaultIdleCommand().schedule();
        robot.shooter.defaultIdleCommand().schedule();

        // Inherit where autonomous left off. Without this, teleop starts with an unknown heading
        // while defaulting to field-centric drive, the mode that depends on heading most, so the
        // driver's first stick input sends the robot in an arbitrary direction.
        if (PoseStorage.hasPose()) {
            robot.drivetrain.setPose(PoseStorage.getPose());
            robot.poseFusion.seed(PoseStorage.getPose());
            inheritedPose = true;
        }
        if (PoseStorage.hasAlliance()) {
            alliance = PoseStorage.getAlliance();
            allianceSource = "from auto";
        }
        applyAlliance();
        lastCount = robot.storage.count();
    }

    /** Everything that depends on the alliance and must follow it when the dpad changes it. */
    private void applyAlliance() {
        robot.drivetrain.setDriverHeadingOffset(Field.driverForwardHeading(alliance));
    }

    @Override
    protected void onInitLoop() {
        // Auto normally chooses the alliance, but PoseStorage outlives the match: a practice run or
        // the previous match can leave the wrong one behind, so the dpad always wins (fixthese B4).
        if (gamepad1.dpadLeftWasPressed() || gamepad1.dpadRightWasPressed()) {
            alliance = alliance.opposite();
            allianceSource = allianceSource.equals("from auto") || allianceSource.startsWith("dpad, overrode")
                    ? "dpad, overrode auto" : "dpad";
            applyAlliance();
        }
        // Expected to stay false all season (BIOBUZZ tags move). Only with a camera, and no more
        // often than LOCALIZE_RETRY_MS: the attempt is a pipeline switch plus a read.
        long now = robot.getClock().nowMs();
        if (robot.limelight.isAvailable() && (!localizeTried || now - lastLocalizeTryMs >= LOCALIZE_RETRY_MS)) {
            localizeTried = true;
            lastLocalizeTryMs = now;
            localized = robot.tryLocalizeFromAprilTag();
        }

        reportMissingHardware();
        if (robot.drivetrain.isAvailable() && !robot.drivetrain.isLocalizerSettled()) {
            telemetry.addLine("!! Localizer calibrating: wait a second before START");
        }
        telemetry.addData("Alliance", alliance + " (" + allianceSource + ")  dpad left/right to change");
        telemetry.addData("Pose from auto?", inheritedPose ? "yes" : "no - press Y once facing away from the driver wall");
        telemetry.addData("Localized?", localized ? "yes" : "no (odometry only this season)");
        int[] tags = tagRange();
        telemetry.addData("Aim target", alliance + " " + targetSide() + " CELL, tags " + tags[0] + "-" + tags[1]);
        telemetry.addData("Pose", robot.drivetrain.getPose());
        telemetry.addData("Drive", driveStatus());
        if (loggerError != null) telemetry.addData("!! Logger FAILED", loggerError);
        telemetry.addLine();
        for (String line : Controls.helpLines()) telemetry.addLine(line);
    }

    @Override
    protected void onDecide() {
        Controls.Snapshot in = Controls.read(gamepad1, gamepad2);
        handleDriver(in);
        handleOperator(in);
        updateAimLock(in);
        restoreFlywheelHold();
    }

    @Override
    protected void onAfterAct() {
        updateHaptics();
    }

    @Override
    protected void onTelemetry() {
        matchTelemetry();
        if (DEBUG_TELEMETRY) debugTelemetry();
    }

    // ---- Input ----

    private double shaped(Controls control) {
        return DriveScaling.shape(control.axis(gamepad1, gamepad2)) * slowScale();
    }

    private double slowScale() {
        return DriveScaling.slowScale(Controls.SLOW_MODE.axis(gamepad1, gamepad2));
    }

    private void handleDriver(Controls.Snapshot in) {
        if (in.pressed(Controls.TOGGLE_DRIVE_FRAME)) robot.drivetrain.toggleFieldCentric();
        if (in.pressed(Controls.RESET_HEADING)) {
            // Escape hatch when field-centric drive has drifted: the driver faces the robot away
            // from their wall and presses it. Without this a bad localisation makes the robot undrivable.
            robot.drivetrain.resetHeading();
        }

        // Two ways out of a macro: the abort button, or simply grabbing the sticks. The sticks only
        // abort a macro that owns the drivetrain; a shot in progress is not the driver's to cancel.
        if (macroRunning() && (in.pressed(Controls.ABORT)
                || (macroOwns(robot.drivetrain) && driverWantsControl(in)))) {
            abortMacro();
        }
        if (macroRunning()) return;

        if (!robot.drivetrain.isAvailable()) {
            // No follower: every drive macro would finish at once with TIMED_OUT and the aim lock has
            // nothing to steer. Answer the press with the failure rumble instead.
            if (in.pressed(Controls.COLLECT) || in.pressed(Controls.ALIGN)
                    || in.pressed(Controls.DRIVE_TO_SHOOT) || in.pressed(Controls.DRIVE_TO_PARK)
                    || in.pressed(Controls.SNAP_90) || in.pressed(Controls.SNAP_0)
                    || in.pressed(Controls.SNAP_270) || in.pressed(Controls.SNAP_180)) {
                gamepad1.rumbleBlips(RUMBLE_FAILURE_BLIPS);
            }
            return;
        }

        if (in.pressed(Controls.COLLECT)) {
            startMacro(robot.macros.collectPiece());
        } else if (in.pressed(Controls.ALIGN)) {
            startMacro(robot.macros.alignToPiece());
        } else if (in.pressed(Controls.DRIVE_TO_SHOOT)) {
            startMacro(robot.macros.driveTo(alliancePose(FieldPoses.BLUE_SHOOTING_SPOT)));
        } else if (in.pressed(Controls.DRIVE_TO_PARK)) {
            startMacro(robot.macros.driveTo(alliancePose(FieldPoses.BLUE_PARK)));
        } else if (in.pressed(Controls.SNAP_90)) {
            snapTo(90);
        } else if (in.pressed(Controls.SNAP_0)) {
            snapTo(0);
        } else if (in.pressed(Controls.SNAP_270)) {
            snapTo(270);
        } else if (in.pressed(Controls.SNAP_180)) {
            snapTo(180);
        }
    }

    private void handleOperator(Controls.Snapshot in) {
        // A direct mechanism command while a macro owns that mechanism would interrupt the macro
        // mid-group and leave its outcome stuck on RUNNING: abort it properly first.
        if (in.pressed(Controls.INTAKE)) {
            abortIfMacroOwns(robot.intake);
            robot.intake.intakeCommand().schedule();
        }
        if (in.pressed(Controls.OUTTAKE)) {
            abortIfMacroOwns(robot.intake);
            robot.intake.outtakeCommand().schedule();
        }
        if (in.pressed(Controls.EJECT)) {
            abortIfMacroOwns(robot.intake);
            robot.intake.ejectCommand().schedule();
        }
        if (in.pressed(Controls.STOP_INTAKE)) {
            if (macroRunning()) abortMacro();
            robot.intake.stopCommand().schedule();
        }

        if (!macroRunning()) {
            if (in.pressed(Controls.INTAKE_UNTIL_FULL)) {
                startMacro(robot.macros.intakeUntilFull());
            } else if (in.pressed(Controls.SHOOT_ONE)) {
                startMacro(robot.macros.shootOne());
            } else if (in.pressed(Controls.SHOOT_ALL)) {
                startMacro(robot.macros.shootAll());
            }
        }

        if (in.pressed(Controls.ARM_FLYWHEEL)) {
            flywheelArmed = !flywheelArmed;
            if (!flywheelArmed) stopFlywheelHold();
        }
        if (in.pressed(Controls.HIVE_TIPPED)) {
            tipsCounted++;       // the aim lock reads the target live, so it re-targets at once
        }
        if (in.pressed(Controls.TOGGLE_DEBUG)) DEBUG_TELEMETRY = !DEBUG_TELEMETRY;
    }

    /**
     * The aim lock is held, not toggled: while the driver pulls the trigger the drivetrain's
     * heading hold takes its setpoint from {@code Macros.aimHeading} on the current up-CELL (read
     * live, so a counted TIP re-targets at once); releasing the trigger returns the hold to normal.
     * It rides inside the driver-control default command, so it survives shots and resumes after
     * any macro without being rescheduled.
     */
    private void updateAimLock(Controls.Snapshot in) {
        boolean wanted = robot.drivetrain.isAvailable() && in.axis(Controls.AIM_LOCK) > AIM_LOCK_TRIGGER;
        if (wanted && !robot.drivetrain.isAimLocked()) {
            robot.drivetrain.setAimLock(() -> {
                int[] tags = tagRange();
                return robot.macros.aimHeading(targetCell(), tags[0], tags[1]);
            });
        } else if (!wanted && robot.drivetrain.isAimLocked()) {
            robot.drivetrain.clearAimLock();
        }
    }

    /**
     * The flywheel hold is the operator's standing wish, not a one-shot. A shot macro preempts it
     * (Ivy ends, it does not suspend, a priority-0 command), so once no macro is running it is
     * scheduled again.
     */
    private void restoreFlywheelHold() {
        if (macroRunning()) return;
        if (flywheelArmed && !isScheduled(spinHold)) {
            spinHold = robot.shooter.holdSpeedCommand();
            spinHold.schedule();
        }
    }

    private void stopFlywheelHold() {
        if (spinHold != null) Scheduler.cancel(spinHold);
        spinHold = null;
    }

    private static boolean isScheduled(Command command) {
        return command != null && Scheduler.isScheduled(command);
    }

    private void snapTo(double degrees) {
        startMacro(robot.macros.snapToHeading(Math.toRadians(degrees)));
    }

    private void startMacro(Command macro) {
        activeMacro = macro;
        macro.schedule();
    }

    private boolean macroRunning() {
        return isScheduled(activeMacro);
    }

    private boolean macroOwns(Object subsystem) {
        return macroRunning() && activeMacro.requirements().contains(subsystem);
    }

    private void abortIfMacroOwns(Object subsystem) {
        if (macroOwns(subsystem)) abortMacro();
    }

    private boolean driverWantsControl(Controls.Snapshot in) {
        return Math.abs(in.axis(Controls.DRIVE_FORWARD)) > MACRO_ABORT_STICK
                || Math.abs(in.axis(Controls.DRIVE_STRAFE)) > MACRO_ABORT_STICK
                || Math.abs(in.axis(Controls.DRIVE_TURN)) > MACRO_ABORT_STICK;
    }

    private void abortMacro() {
        Scheduler.cancel(activeMacro);
        // Cancelling the command releases the Ivy resources, but the Pedro follower drives itself
        // once handed a path: this is the call that actually stops the robot.
        robot.abortMacro();
        activeMacro = null;
    }

    /** Which motor layer the sticks reach, and what that costs the driver. */
    private String driveStatus() {
        if (robot.drivetrain.isAvailable()) {
            return robot.drivetrain.isFieldCentric() ? "Pedro, field-centric" : "Pedro, robot-centric";
        }
        if (robot.openLoopDrive.isAvailable()) {
            return "OPEN LOOP (Pedro not tuned): robot-centric, no heading hold, drive macros off";
        }
        return "!! NO DRIVE MOTORS";
    }

    // ---- Season targets, from game/ ----

    private Field.CellSide targetSide() {
        return Field.upCellSide(alliance, tipsCounted);
    }

    private Pose targetCell() {
        return Field.cell(alliance, targetSide());
    }

    private int[] tagRange() {
        return Field.tagRange(alliance, targetSide());
    }

    private Pose alliancePose(Pose bluePose) {
        return FieldConstants.forAlliance(bluePose, alliance);
    }

    // ---- Feedback ----

    /**
     * Haptic feedback for things a driver cannot see. Telemetry reports outcomes accurately and no
     * driver reads it mid-match. Each of these fires on a transition, so a held state never buzzes
     * continuously. CANCELLED is deliberately silent: the driver just cancelled it and knows.
     */
    private void updateHaptics() {
        Macros.Outcome outcome = robot.macros.getOutcome();
        if (outcome != lastOutcome) {
            if (outcome == Macros.Outcome.SUCCESS) {
                gamepad1.rumbleBlips(RUMBLE_SUCCESS_BLIPS);
            } else if (outcome == Macros.Outcome.TIMED_OUT || outcome == Macros.Outcome.NO_TARGET) {
                gamepad1.rumbleBlips(RUMBLE_FAILURE_BLIPS);
            }
            lastOutcome = outcome;
        }

        // Possession is the one piece of state both drivers act on, so both get told.
        boolean full = robot.storage.isFull();
        int count = robot.storage.count();
        if (full && !announcedFull) {
            gamepad1.rumbleBlips(RUMBLE_FULL_BLIPS);
            gamepad2.rumbleBlips(RUMBLE_FULL_BLIPS);
        } else if (count > lastCount) {
            gamepad1.rumbleBlips(RUMBLE_SUCCESS_BLIPS);
            gamepad2.rumbleBlips(RUMBLE_SUCCESS_BLIPS);
        }
        announcedFull = full;
        lastCount = count;

        MatchClock clock = robot.getMatchClock();
        if (!announcedFinal && clock != null && clock.isFinalSeconds()) {
            announcedFinal = true;
            gamepad1.rumbleBlips(RUMBLE_FINAL_BLIPS);
            gamepad2.rumbleBlips(RUMBLE_FINAL_BLIPS);
        }
    }

    // ---- Telemetry ----

    /**
     * What a driver can actually use mid-match: time, pieces, what the last macro did, how the
     * robot is aiming, and anything broken. Faults render only when present, so their presence is
     * itself the signal.
     */
    private void matchTelemetry() {
        MatchClock clock = robot.getMatchClock();
        telemetry.addData("Time", clock == null ? "-" : clock.getStatus());
        telemetry.addData("Pieces", robot.macros.piecesOnBoard() + "/" + Storage.CAPACITY
                + (robot.storage.isFull() ? "  FULL" : ""));
        telemetry.addData("Macro", robot.macros.getStatus());
        telemetry.addData("Drive", driveStatus());
        telemetry.addData("Aim", (robot.drivetrain.isAimLocked() ? "LOCKED on " : Controls.AIM_LOCK.button() + " aims at ")
                + alliance + " " + targetSide() + " CELL");
        telemetry.addData("Flywheel", flywheelArmed
                ? String.format(Locale.US, "ARMED  %.0f rpm", robot.shooter.getRpm()) : "off");

        for (String missing : robot.getMissingHardware()) telemetry.addData("!! MISSING", missing);
        if (loggerError != null) telemetry.addData("!! Logger FAILED", loggerError);
        if (robot.intake.hasGivenUpUnjamming()) {
            telemetry.addLine("!! INTAKE JAMMED - anti-jam gave up. Use "
                    + Controls.OUTTAKE.button() + " on gamepad 2 to outtake.");
        }
        double volts = robot.getBatteryVolts();
        if (volts > 0 && volts < LOW_BATTERY_VOLTS) {
            telemetry.addData("!! BATTERY LOW", "%.2f V", volts);
        }
        if (!DEBUG_TELEMETRY) {
            telemetry.addLine("(" + Controls.TOGGLE_DEBUG.button() + " on gamepad 2 for debug)");
        }
    }

    /** Everything else. Useful in the pit and at practice; noise during a match. */
    private void debugTelemetry() {
        telemetry.addLine();
        // A spike lasts one cycle and is gone before anyone can read it, so p95, max and a spike
        // count are what actually diagnose a stuttering loop.
        telemetry.addData("Loop", loopStats.getStatus());
        telemetry.addData("Battery V", "%.2f", robot.getBatteryVolts());
        telemetry.addData("Slow scale", "%.2f", slowScale());
        telemetry.addData("Pose", robot.drivetrain.getPose());
        telemetry.addData("Heading hold", robot.drivetrain.isHeadingHoldActive()
                ? String.format(Locale.US, "holding %.0f deg", Math.toDegrees(robot.drivetrain.getHeldHeading()))
                : "driver steering");
        telemetry.addData("Path", robot.drivetrain.isFollowingPath() ? "FOLLOWING"
                : robot.drivetrain.isHoldingPose() ? "HOLDING" : "manual");
        telemetry.addLine();
        telemetry.addData("Intake", "%s  %.0f / %.0f t/s  %.2f A%s", robot.intake.getMode(),
                robot.intake.getTargetVelocity(), robot.intake.getVelocityTicksPerSec(),
                robot.intake.getCurrentAmps(), robot.intake.isUnjamming() ? "  UNJAMMING" : "");
        telemetry.addData("Storage", "%d  %s  exits %d%s", robot.storage.count(), robot.storage.getMode(),
                robot.storage.getExitEvents(), robot.storage.hasExitSensor() ? "" : "  (no exit sensor)");
        telemetry.addData("Transfer", "%s  lift %s  feed %s", robot.transfer.getMode(),
                robot.transfer.hasPieceInLift(), robot.transfer.hasFeedSensor() ? robot.transfer.pieceAtFeed() : "n/a");
        int[] aimTags = tagRange();
        double aim = robot.macros.aimHeading(targetCell(), aimTags[0], aimTags[1]);
        telemetry.addData("Aim heading", Double.isNaN(aim) ? "n/a"
                : String.format(Locale.US, "%.1f deg (%s)", Math.toDegrees(aim), robot.drivetrain.isAimLocked() ? "locked" : "off"));
        telemetry.addData("Shooter", "%.0f / %.0f rpm  %s", robot.shooter.getRpm(),
                robot.shooter.getTargetRpm(), robot.shooter.getMode());
        telemetry.addLine();
        int[] tags = tagRange();
        telemetry.addData("Limelight", "%s  pipeline %s  tag tx %.1f", robot.limelight.isAvailable() ? "ok" : "MISSING",
                robot.limelight.getPipelineName(), robot.limelight.getTagTx(tags[0], tags[1]));
        telemetry.addData("Localization", robot.poseFusion.getStatus());
        telemetry.addData("Sensors hue", "entrance %.0f  full %.0f  transfer %.0f  feed %.0f",
                robot.storageEntranceSensor.getHue(), robot.storageFullSensor.getHue(),
                robot.transferSensor.getHue(), robot.shooterFeedSensor.getHue());
        telemetry.addData("Shots fired", robot.macros.getShotsFired());
    }
}
