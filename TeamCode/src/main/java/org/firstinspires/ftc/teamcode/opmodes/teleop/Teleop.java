package org.firstinspires.ftc.teamcode.opmodes.teleop;

import com.pedropathing.ivy.Scheduler;
import com.pedropathing.math.Pose;

import org.firstinspires.ftc.teamcode.commands.Macros;
import org.firstinspires.ftc.teamcode.game.Field;
import org.firstinspires.ftc.teamcode.game.FieldPoses;
import org.firstinspires.ftc.teamcode.opmodes.MatchOpMode;
import org.firstinspires.ftc.teamcode.subsystems.Intake;
import org.firstinspires.ftc.teamcode.subsystems.Shooter;
import org.firstinspires.ftc.teamcode.util.field.Alliance;
import org.firstinspires.ftc.teamcode.util.field.FieldConstants;
import org.firstinspires.ftc.teamcode.util.field.PoseStorage;
import org.firstinspires.ftc.teamcode.util.math.DriveScaling;
import org.firstinspires.ftc.teamcode.util.time.MatchClock;


/**
 * The match TeleOp. Concrete per alliance ({@code BlueTeleop}, {@code RedTeleop}) so the side is
 * chosen by picking the OpMode and can never be stale, unconfirmed, or inherited wrong from a
 * previous run.
 *
 * <h2>Two ways to shoot, and the second one always works</h2>
 * Normally the flywheel speed comes from the pose — distance to the target CELL through
 * {@code Shooter.setTargetForDistance} — and the aim lock points the rear-firing shooter at that CELL
 * through {@code Macros.aimHeading}. Both depend on odometry being right.
 *
 * <p><b>Operator dpad-up is the escape hatch.</b> It drops into MANUAL: the speed becomes
 * {@code Shooter.MANUAL_TICKS_PER_SEC}, trimmed live on dpad left/right, and the aim lock is switched
 * off so the driver points the robot by hand. Odometry, the distance table and the aim law are all
 * out of the loop in one press. That matters because every automatic path here is pose-derived, so
 * one hard collision can make all of them wrong at once, and there is no sensor anywhere that would
 * notice.
 *
 * <h2>Nothing counts pieces</h2>
 * There is no sensor in the intake or the tunnel. Shoot One fires one tunnel pulse, Shoot All fires
 * {@code Macros.PIECES_PER_LOAD}, and the operator decides when the robot is empty. The card says
 * "pulses", never "pieces".
 */
public abstract class Teleop extends MatchOpMode {
    /** How much dpad left/right moves the manual flywheel speed, in ticks/sec per press. */
    public static double SPEED_TRIM_TICKS_PER_SEC = 100;

    private final Alliance side;

    private boolean intakeForward = false;
    private boolean intakeReverse = false;
    private boolean flywheelArmed = false;
    private boolean manualMode = false;
    /** How many times our HIVE has tipped, which flips the CELL the shooter must hit. */
    private int tipsCounted = 0;

    private Pose targetCell = null;
    private int minTag = 0;
    private int maxTag = 0;

    private Macros.Outcome lastOutcome = Macros.Outcome.IDLE;
    private boolean warnedFinalSeconds = false;
    private boolean warnedJam = false;

    protected Teleop(Alliance side) {
        this.side = side;
    }

    @Override
    protected final Alliance alliance() {
        return side;
    }

    @Override
    protected final MatchClock.Period matchPeriod() {
        return MatchClock.Period.TELEOP;
    }

    @Override
    protected void onInit() {
        // Exactly one drive default, and one idle per mechanism. The drive default reads the sticks
        // through DriveScaling and applies the slow-mode scale; the heading hold lives inside
        // Drivetrain and only engages once there is a follower.
        Scheduler.schedule(
                robot.drivetrain.driverControlCommand(
                        () -> shaped(Controls.DRIVE_FORWARD),
                        () -> shaped(Controls.DRIVE_STRAFE),
                        () -> shaped(Controls.DRIVE_TURN)),
                robot.intake.operatorControlCommand(this::intakeMode),
                robot.shooter.armedControlCommand(() -> flywheelArmed));

        robot.drivetrain.setDriverHeadingOffset(Field.driverForwardHeading(side));
        // Autonomous leaves its final pose behind, so field-centric drive and the aim law work from
        // the first second of teleop with no re-init ritual.
        if (PoseStorage.hasPose()) robot.drivetrain.setPose(PoseStorage.getPose());
        retarget();
    }

    @Override
    protected void onInitLoop() {
        telemetry.addLine(side + " teleop");
        telemetry.addLine(robot.drivetrain.hasFollower()
                ? "Pedro tuned: paths, snaps and aim lock are live."
                : "NOT TUNED: robot-centric sticks only; paths, snaps and aim lock are off.");
        telemetry.addLine();
        for (String line : Controls.helpLines()) telemetry.addLine(line);
    }

    @Override
    protected void onDecide() {
        Controls.Snapshot in = Controls.read(gamepad1, gamepad2);
        boolean tuned = robot.drivetrain.hasFollower();

        // One generic gate: anything that declared Needs.DRIVETRAIN is refused, with a buzz, when
        // there is no follower to drive it. A new macro cannot slip past by being left off a list.
        if (!tuned && in.anyPressed(Controls::requiresDrivetrain)) {
            gamepad1.rumbleBlips(3);
        }

        handleDriver(in, tuned);
        handleOperator(in);
        updateShooterTarget();
        updateAimLock(in, tuned);
    }

    private void handleDriver(Controls.Snapshot in, boolean tuned) {
        if (in.pressed(Controls.TOGGLE_DRIVE_FRAME)) robot.drivetrain.toggleFieldCentric();
        if (in.pressed(Controls.RESET_HEADING)) robot.drivetrain.resetHeading();

        // A stick abort only cancels a macro that owns the drivetrain: a driver grabbing the sticks
        // must not cancel a shot in progress.
        boolean stickMoved = Math.abs(shaped(Controls.DRIVE_FORWARD)) > 0
                || Math.abs(shaped(Controls.DRIVE_STRAFE)) > 0
                || Math.abs(shaped(Controls.DRIVE_TURN)) > 0;
        if (in.pressed(Controls.ABORT) || (stickMoved && robot.drivetrain.isFollowingPath())) {
            robot.abortMacro();
        }

        if (!tuned) return;

        if (in.pressed(Controls.RESEED_POSE)) {
            // The one-button recovery after a hard collision. Both the aim law and the flywheel's
            // distance lookup are pose-derived, so drift breaks the whole scoring path at once.
            // The robot has to actually be on its start line for this to mean anything.
            robot.drivetrain.setPose(FieldConstants.forAlliance(FieldPoses.BLUE_START_FACING_HIVE, side));
        }
        if (in.pressed(Controls.DRIVE_TO_SHOOT)) {
            Scheduler.schedule(robot.macros.driveTo(
                    FieldConstants.forAlliance(FieldPoses.BLUE_SHOOTING_SPOT, side)));
        }
        if (in.pressed(Controls.DRIVE_TO_PARK)) {
            Scheduler.schedule(robot.macros.driveTo(
                    FieldConstants.forAlliance(FieldPoses.BLUE_PARK, side)));
        }
        if (in.pressed(Controls.SNAP_0)) snap(0);
        if (in.pressed(Controls.SNAP_90)) snap(90);
        if (in.pressed(Controls.SNAP_180)) snap(180);
        if (in.pressed(Controls.SNAP_270)) snap(270);
    }

    private void snap(double degrees) {
        Scheduler.schedule(robot.macros.snapToHeading(Math.toRadians(degrees)));
    }

    private void handleOperator(Controls.Snapshot in) {
        if (in.pressed(Controls.INTAKE)) {
            intakeForward = !intakeForward;
            intakeReverse = false;
        }
        if (in.pressed(Controls.OUTTAKE)) {
            intakeReverse = !intakeReverse;
            intakeForward = false;
        }
        if (in.pressed(Controls.STOP_INTAKE)) {
            intakeForward = false;
            intakeReverse = false;
            robot.abortMacro();
        }

        if (in.pressed(Controls.ARM_FLYWHEEL)) flywheelArmed = !flywheelArmed;
        if (in.pressed(Controls.TOGGLE_MANUAL)) manualMode = !manualMode;
        if (in.pressed(Controls.HIVE_TIPPED)) {
            tipsCounted++;
            retarget();
        }
        if (in.pressed(Controls.SPEED_UP)) Shooter.MANUAL_TICKS_PER_SEC += SPEED_TRIM_TICKS_PER_SEC;
        if (in.pressed(Controls.SPEED_DOWN)) Shooter.MANUAL_TICKS_PER_SEC -= SPEED_TRIM_TICKS_PER_SEC;

        if (in.pressed(Controls.SHOOT_ONE)) Scheduler.schedule(robot.macros.shootOne());
        if (in.pressed(Controls.SHOOT_ALL)) Scheduler.schedule(robot.macros.shootAll());

    }

    /**
     * What the operator currently wants the intake to do. Read every loop by the default command
     * rather than pushed on a press: the rest state is then enforced continuously, so a missed edge
     * or a preempting shot can never leave a mechanism running.
     */
    private Intake.Mode intakeMode() {
        if (intakeForward) return Intake.Mode.IN;
        if (intakeReverse) return Intake.Mode.OUT;
        return Intake.Mode.OFF;
    }

    /**
     * The flywheel speed, chosen every loop. Manual wins outright; otherwise the distance table is
     * consulted, and with no pose at all the manual value is the only thing left.
     */
    private void updateShooterTarget() {
        Pose pose = robot.drivetrain.getPose();
        if (manualMode || pose == null || targetCell == null) {
            robot.shooter.setManualTarget();
        } else {
            robot.shooter.setTargetForDistance(pose.distance(targetCell));
        }
    }

    /**
     * The aim lock, held on the right trigger: the heading hold points the rear-firing shooter at the
     * target CELL while the sticks still translate. Off in manual mode, by design — that is what
     * makes manual an override and not a partial one.
     */
    private void updateAimLock(Controls.Snapshot in, boolean tuned) {
        boolean wanted = tuned && !manualMode
                && Controls.AIM_LOCK.axis(gamepad1, gamepad2) > 0.5
                && targetCell != null;
        if (wanted && !robot.drivetrain.isAimLocked()) {
            robot.drivetrain.setAimLock(() -> robot.macros.aimHeading(targetCell, minTag, maxTag));
        } else if (!wanted && robot.drivetrain.isAimLocked()) {
            robot.drivetrain.clearAimLock();
        }
    }

    /** Recomputed only when the alliance's up-CELL changes, not every loop. */
    private void retarget() {
        Field.CellSide up = Field.upCellSide(side, tipsCounted);
        targetCell = Field.cell(side, up);
        int[] range = Field.tagRange(side, up);
        minTag = range[0];
        maxTag = range[1];
    }

    @Override
    protected void onAfterAct() {
        // Haptics fire on transitions, because a driver cannot read telemetry mid-match.
        Macros.Outcome now = robot.macros.getOutcome();
        if (now != lastOutcome) {
            if (now == Macros.Outcome.SUCCESS) gamepad1.rumbleBlips(1);
            else if (now == Macros.Outcome.TIMED_OUT) gamepad1.rumbleBlips(3);
            lastOutcome = now;
        }

        // A jam the anti-jam logic has given up on needs a human to reverse it.
        if (robot.intake.hasGivenUpUnjamming() && !warnedJam) {
            warnedJam = true;
            gamepad2.rumbleBlips(3);
        } else if (!robot.intake.hasGivenUpUnjamming()) {
            warnedJam = false;
        }

        MatchClock clock = robot.getMatchClock();
        if (clock != null && clock.isFinalSeconds() && !warnedFinalSeconds) {
            warnedFinalSeconds = true;
            // One long buzz, deliberately unlike any other pattern: 20 seconds left.
            gamepad1.rumble(600);
            gamepad2.rumble(600);
        }
    }

    @Override
    protected void onTelemetry() {
        MatchClock clock = robot.getMatchClock();
        // Loop rate first: it is the number that explains everything else going wrong.
        telemetry.addData("Loop", "%.0f Hz (%.1f ms)", robot.getLoopHz(), robot.getLoopMs());
        telemetry.addData("Time", clock == null ? "-" : clock.getStatus());
        telemetry.addData("Battery", "%.1f V", robot.getBatteryVolts());
        telemetry.addLine();

        telemetry.addData("Mode", manualMode ? "MANUAL (odometry ignored)" : "auto aim + table");
        telemetry.addData("Flywheel", "%s  %s", flywheelArmed ? "ARMED" : "off",
                robot.shooter.getStatusLine());
        telemetry.addData("Intake", intakeForward ? "IN" : intakeReverse ? "REVERSE" : "off");
        telemetry.addData("Macro", "%s  (%d pulses)", robot.macros.getStatus(),
                robot.macros.getShotsFired());

        String aim = !robot.drivetrain.hasFollower() ? "no follower"
                : robot.drivetrain.isAimLocked()
                        ? (robot.macros.hasAimBias()
                                ? String.format("LOCKED, tag-corrected %+.1f deg",
                                        robot.macros.getAimBiasDegrees())
                                : "LOCKED, odometry only")
                        : "free";
        telemetry.addData("Aim", aim);
        telemetry.addData("Target", "CELL %s, tags %d-%d, %d tip(s)",
                Field.upCellSide(side, tipsCounted), minTag, maxTag, tipsCounted);
        telemetry.addData("Drive", robot.drivetrain.isFieldCentric() ? "field centric" : "robot centric");

        if (robot.intake.hasGivenUpUnjamming()) telemetry.addLine("!! INTAKE JAMMED - reverse it");
        if (!robot.limelight.isConnected()) telemetry.addLine("!! no Limelight: odometry aim only");
    }

    private double shaped(Controls control) {
        double raw = control.axis(gamepad1, gamepad2);
        return DriveScaling.shape(raw) * DriveScaling.slowScale(Controls.SLOW_MODE.axis(gamepad1, gamepad2));
    }
}
