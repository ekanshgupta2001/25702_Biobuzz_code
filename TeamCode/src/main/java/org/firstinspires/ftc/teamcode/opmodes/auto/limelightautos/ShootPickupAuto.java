package org.firstinspires.ftc.teamcode.opmodes.auto.limelightautos;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.commands.Shoot;
import org.firstinspires.ftc.teamcode.game.Field;
import org.firstinspires.ftc.teamcode.opmodes.auto.HardcodedAuto;
import org.firstinspires.ftc.teamcode.util.field.Alliance;
import org.firstinspires.ftc.teamcode.util.field.AllianceStorage;

import java.util.Locale;

/**
 * Limelight auto 1: shoot four, pick up the four pollen in the corner, shoot again if the HIVE has
 * not tipped. The Driver Station runs it as {@link RedShootPickupAuto} or {@link BlueShootPickupAuto};
 * those two only say which alliance. All the logic and every number to tune is in this file.
 *
 * <p><b>PLACE:</b> where Auto HARDCODED starts: back on the wall, FRONT (shooter + camera) facing the
 * CELL that starts UP.
 *
 * <pre>
 *   SPIN_UP         wheels on, wait for them
 *   FEED            intake + feeder: the 4 pre-loaded balls go in
 *   DWELL           a moment for the last ball, then wheels off
 *   TURN_TO_POLLEN  turn 90 degrees toward the pollen corner       (timer)
 *   SQUARE          strafe into the wall so the robot is square    (timer)
 *   PICKUP          drive forward with the intake running          (timer)
 *   TURN_TO_HIVE    turn most of the way back toward the HIVE      (timer)
 *   FIND_TAG        keep turning slowly until the camera has the tag centred
 *                     tag found      = the CELL is still UP = not tipped -> shoot again
 *                     never found    = the CELL is DOWN = tipped         -> DONE, keep the balls
 *   SPIN_UP_2, FEED_2, DWELL_2       the second volley
 *   DONE            everything off
 * </pre>
 *
 * <p><b>How it tells a tip:</b> a CELL's tags can only be seen while that CELL is up. The robot looks
 * for the tags of the CELL it shot at first. Seen = still up. Not seen = it tipped.
 *
 * <p><b>Left and right are the same for RED and BLUE</b>, because the field is the same field turned
 * half way round. Only the tags differ.
 *
 * <p>The wheel speeds are automatic, the same as Teleop: {@code robot.readSensors()} sets them from the
 * tag distance every loop, and uses the fixed speeds when there is no tag.
 *
 * <p>Every step is a timer or has a timeout, so nothing can get stuck. Every time, power and direction
 * below is a first guess: tune them on the practice field, one step at a time, top to bottom.
 */
public abstract class ShootPickupAuto extends OpMode {
    // ---- Numbers to tune. Drive convention: + forward, + strafe LEFT, + turn LEFT (counter-clockwise). ----

    /** Which way the pollen corner is from the start spot: +1 = turn LEFT, -1 = turn RIGHT. */
    public static double TURN_TO_POLLEN_DIRECTION = 1;
    public static double TURN_POWER = 0.5;
    /** How long a 90 degree turn takes at {@link #TURN_POWER}. It does not need to be exact: SQUARE fixes it. */
    public static long TURN_90_MS = 450;

    /** Which way the wall is after that turn: +1 = strafe LEFT into it, -1 = strafe RIGHT. */
    public static double SQUARE_DIRECTION = 1;
    public static double SQUARE_POWER = 0.4;
    public static long SQUARE_MS = 800;

    public static double PICKUP_POWER = 0.35;
    /** How long it drives forward over the pollen with the intake on. */
    public static long PICKUP_MS = 1800;

    /**
     * The timed part of the turn back to the HIVE (the opposite way to the first turn). Set it so the
     * robot stops a little SHORT of facing the HIVE: FIND_TAG carries on turning the same way, slowly,
     * until the camera finds the tag. If this turn goes too far, FIND_TAG turns away from the tag.
     */
    public static long TURN_BACK_MS = 500;

    /** Turn power while searching for the tag, and while centring on it. Slow, so it does not swing past. */
    public static double FIND_POWER = 0.2;
    /** The tag is "centred" when it is within this many degrees of straight ahead. */
    public static double AIM_TOLERANCE_DEG = 3;
    /** Give up searching after this long. No tag by then = the HIVE tipped. */
    public static long FIND_TIMEOUT_MS = 3000;

    /** Wheel spin-up before the second volley. */
    public static long SECOND_SPIN_UP_MS = 2000;

    /** The autonomous period. Nothing may move after it (BIOBUZZ G403). */
    private static final long AUTO_PERIOD_MS = 30000;

    private enum State {
        SPIN_UP, FEED, DWELL,
        TURN_TO_POLLEN, SQUARE, PICKUP, TURN_TO_HIVE, FIND_TAG,
        SPIN_UP_2, FEED_2, DWELL_2,
        DONE
    }

    private final Alliance alliance;
    /** The CELL that starts UP: the one this auto shoots at, and looks for again after the pickup. */
    private final Field.CellSide firstCell;

    private Robot robot;
    private State state = State.DONE;
    private long startMs = 0;
    private long stateSinceMs = 0;

    /** What the drivetrain is told every loop. Set by {@link #enter} (and by FIND_TAG while aiming). */
    private double forward = 0, strafe = 0, turn = 0;

    /** True once FIND_TAG has seen the first CELL's tag, even for one loop. */
    private boolean tagFound = false;
    /** For the card: what the tip check decided. */
    private String tipResult = "not checked yet";

    protected ShootPickupAuto(Alliance alliance) {
        this.alliance = alliance;
        this.firstCell = Field.startingUpCellSide(alliance);
    }

    @Override
    public void init() {
        robot = new Robot(hardwareMap);
        robot.setAlliance(alliance);        // the camera reads this alliance's tags
        AllianceStorage.save(alliance);     // Teleop starts on this side
        telemetry.setMsTransmissionInterval(100);
    }

    @Override
    public void init_loop() {
        robot.readSensors();   // so the card shows what the camera sees before START

        telemetry.addLine("=========================");
        telemetry.addLine("   ALLIANCE:  " + alliance);
        telemetry.addLine("=========================");
        telemetry.addLine("LIMELIGHT AUTO 1: shoot 4, pick up 4, shoot again if not tipped");
        telemetry.addLine("PLACE: back on the wall, FRONT facing the UP cell");
        telemetry.addLine();
        // The pre-match check: these should show a tag, a distance, and "UP cell in view: YES".
        telemetry.addData("Limelight", robot.limelight.getStatusLine());
        telemetry.addData("Distance", distanceText());
        telemetry.addData("UP cell in view", robot.seesCell(firstCell) ? "YES" : "NO  <-- check the placement");
        telemetry.addData("Battery", "%.1f V", robot.getBatteryVolts());
    }

    @Override
    public void start() {
        startMs = nowMs();
        robot.drivetrain.onStart();
        enter(State.SPIN_UP, startMs);
    }

    @Override
    public void loop() {
        robot.readSensors();   // camera, and the wheel speeds from the tag distance
        long now = nowMs();

        // Safety net: nothing may move once the 30 s period is over.
        if (now - startMs >= AUTO_PERIOD_MS && state != State.DONE) {
            enter(State.DONE, now);
        }

        long inState = now - stateSinceMs;
        switch (state) {
            case SPIN_UP:
                if (inState >= HardcodedAuto.FIRST_SHOT_AT_MS) enter(State.FEED, now);
                break;
            case FEED:
                if (inState >= HardcodedAuto.FEED_MS) enter(State.DWELL, now);
                break;
            case DWELL:
                if (inState >= Shoot.FINAL_DWELL_MS) enter(State.TURN_TO_POLLEN, now);
                break;
            case TURN_TO_POLLEN:
                if (inState >= TURN_90_MS) enter(State.SQUARE, now);
                break;
            case SQUARE:
                if (inState >= SQUARE_MS) enter(State.PICKUP, now);
                break;
            case PICKUP:
                if (inState >= PICKUP_MS) enter(State.TURN_TO_HIVE, now);
                break;
            case TURN_TO_HIVE:
                if (inState >= TURN_BACK_MS) enter(State.FIND_TAG, now);
                break;
            case FIND_TAG:
                findTag(now, inState);
                break;
            case SPIN_UP_2:
                if (inState >= SECOND_SPIN_UP_MS) enter(State.FEED_2, now);
                break;
            case FEED_2:
                if (inState >= HardcodedAuto.FEED_MS) enter(State.DWELL_2, now);
                break;
            case DWELL_2:
                if (inState >= Shoot.FINAL_DWELL_MS) enter(State.DONE, now);
                break;
            default:
                break;
        }

        robot.drivetrain.drive(forward, strafe, turn);
        robot.writeActuators();

        telemetry.addData("Step", state);
        telemetry.addData("Time", "%.1f s", (now - startMs) / 1000.0);
        telemetry.addData("Tip check", tipResult);
        telemetry.addData("Drive", "forward %.2f  strafe %.2f  turn %.2f", forward, strafe, turn);
        telemetry.addData("Shooter", robot.shooter.getStatusLine());
        telemetry.addData("Limelight", robot.limelight.getStatusLine());
        telemetry.addData("Distance", distanceText());
    }

    @Override
    public void stop() {
        if (robot != null) robot.limelight.stop();   // or the camera keeps being polled after the OpMode
        // No motor writes here: the SDK rejects them from stop() and stops the motors itself.
    }

    /**
     * FIND_TAG, every loop. No tag of the first CELL: keep turning the way TURN_TO_HIVE was turning.
     * Tag in view: turn toward it until it is centred, then shoot. Out of time: shoot if the tag was
     * ever seen (it is close enough), otherwise the HIVE has tipped and the robot stops here.
     */
    private void findTag(long now, long inState) {
        double tx = robot.limelight.getShotTagTxDegrees();   // + = the tag is to the right
        boolean seen = robot.seesCell(firstCell) && !Double.isNaN(tx);
        if (seen) tagFound = true;

        if (seen && Math.abs(tx) <= AIM_TOLERANCE_DEG) {
            tipResult = "tag centred: NOT tipped, shooting again";
            enter(State.SPIN_UP_2, now);
        } else if (inState >= FIND_TIMEOUT_MS) {
            if (tagFound) {
                tipResult = "tag seen but not centred: NOT tipped, shooting anyway";
                enter(State.SPIN_UP_2, now);
            } else {
                tipResult = "no tag: TIPPED (or the turn missed), keeping the balls";
                enter(State.DONE, now);
            }
        } else if (seen) {
            turn = tx > 0 ? -FIND_POWER : FIND_POWER;   // tag on the right = turn right (negative)
        } else {
            turn = -TURN_TO_POLLEN_DIRECTION * FIND_POWER;   // keep searching the same way
        }
    }

    /** Changes state and sets that state's outputs once, on entry. */
    private void enter(State next, long now) {
        state = next;
        stateSinceMs = now;
        forward = 0;
        strafe = 0;
        turn = 0;
        switch (next) {
            case SPIN_UP:
            case SPIN_UP_2:
                robot.shooter.arm();
                robot.shooter.feederOff();
                robot.intake.stop();
                break;
            case FEED:
            case FEED_2:
                robot.intake.feed();   // the intake and the feeder carry the balls into the wheels
                robot.shooter.feederOn();
                break;
            case DWELL:
            case DWELL_2:
                robot.intake.stop();
                robot.shooter.feederOff();
                break;
            case TURN_TO_POLLEN:
                robot.shooter.disarm();   // the wheels and the feeder
                turn = TURN_TO_POLLEN_DIRECTION * TURN_POWER;
                break;
            case SQUARE:
                strafe = SQUARE_DIRECTION * SQUARE_POWER;
                break;
            case PICKUP:
                robot.intake.in();
                forward = PICKUP_POWER;
                break;
            case TURN_TO_HIVE:
                robot.intake.stop();
                turn = -TURN_TO_POLLEN_DIRECTION * TURN_POWER;   // the opposite way to the first turn
                break;
            case FIND_TAG:
                tagFound = false;
                tipResult = "searching for the tag...";
                break;   // findTag() sets the turn every loop
            case DONE:
            default:
                robot.intake.stop();
                robot.shooter.disarm();
                break;
        }
    }

    private String distanceText() {
        double inches = robot.getShotInches();
        return Double.isNaN(inches) ? "no tag of ours in view (fixed speeds)"
                : String.format(Locale.US, "%.0f in", inches);
    }

    /** Milliseconds from a clock that only counts up (the Driver Station can change the wall clock). */
    private static long nowMs() {
        return System.nanoTime() / 1_000_000;
    }
}
