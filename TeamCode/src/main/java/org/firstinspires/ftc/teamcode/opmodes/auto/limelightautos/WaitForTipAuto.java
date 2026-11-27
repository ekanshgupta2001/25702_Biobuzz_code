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
 * Limelight auto 2: wait for our HIVE to tip, then shoot four and park. No tip in 15 seconds: just
 * park. The Driver Station runs it as {@link RedWaitForTipAuto} or {@link BlueWaitForTipAuto}; those
 * two only say which alliance. All the logic and every number to tune is in this file.
 *
 * <p><b>PLACE:</b> nearer the corner, FRONT (shooter + camera) facing the CELL that starts DOWN.
 *
 * <pre>
 *   WATCH     nothing moves. The camera watches for the tags of the CELL the robot faces.
 *               tags in view (for TIP_CONFIRM_MS) = that CELL came UP = the HIVE tipped -> SPIN_UP
 *               WAIT_MS with no tip                                                     -> PARK
 *   SPIN_UP   wheels on, wait for them
 *   FEED      intake + feeder: the 4 pre-loaded balls go in
 *   DWELL     a moment for the last ball, then wheels off
 *   PARK      one timed move toward the parking spot
 *   DONE      everything off
 * </pre>
 *
 * <p><b>How it tells a tip:</b> a CELL's tags can only be seen while that CELL is up. The CELL this
 * robot faces starts down, so its tags appear when the HIVE tips.
 *
 * <p>The wheel speeds are automatic, the same as Teleop: {@code robot.readSensors()} sets them from the
 * tag distance every loop.
 *
 * <p>If the init screen already says the tag is in view, the robot is facing the wrong CELL (or the
 * HIVE is set up wrong) and it would shoot the moment the match starts.
 */
public abstract class WaitForTipAuto extends OpMode {
    // ---- Numbers to tune. Drive convention: + forward, + strafe LEFT, + turn LEFT (counter-clockwise). ----

    /** How long to wait for a tip before giving up and parking. */
    public static long WAIT_MS = 15000;
    /** The tags must stay in view this long to count as a tip, so one odd camera frame does not. */
    public static long TIP_CONFIRM_MS = 300;

    /** The park move: these three powers for {@link #PARK_MS}. A first guess: strafe right for a second. */
    public static double PARK_FORWARD = 0;
    public static double PARK_STRAFE = -0.5;
    public static double PARK_TURN = 0;
    public static long PARK_MS = 1000;

    /** The autonomous period. Nothing may move after it (BIOBUZZ G403). */
    private static final long AUTO_PERIOD_MS = 30000;

    private enum State { WATCH, SPIN_UP, FEED, DWELL, PARK, DONE }

    private final Alliance alliance;
    /** The CELL that starts DOWN: the one this robot faces. Its tags appear when the HIVE tips. */
    private final Field.CellSide watchedCell;

    private Robot robot;
    private State state = State.DONE;
    private long startMs = 0;
    private long stateSinceMs = 0;

    /** What the drivetrain is told every loop. Set by {@link #enter}. */
    private double forward = 0, strafe = 0, turn = 0;

    /** When the watched CELL's tags came into view; -1 while they are not in view. */
    private long seenSinceMs = -1;
    /** For the card: what happened. */
    private String tipResult = "watching for the tip";

    protected WaitForTipAuto(Alliance alliance) {
        this.alliance = alliance;
        this.watchedCell = Field.startingUpCellSide(alliance).opposite();
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
        telemetry.addLine("LIMELIGHT AUTO 2: wait for the tip, shoot 4, park");
        telemetry.addLine("PLACE: near the corner, FRONT facing the DOWN cell");
        telemetry.addLine();
        // Before the match the DOWN cell's tags must NOT be in view.
        telemetry.addData("Limelight", robot.limelight.getStatusLine());
        telemetry.addData("DOWN cell tags in view", robot.seesCell(watchedCell)
                ? "YES  <-- WRONG: it would shoot at START" : "no (good)");
        telemetry.addData("Battery", "%.1f V", robot.getBatteryVolts());
    }

    @Override
    public void start() {
        startMs = nowMs();
        robot.drivetrain.onStart();
        enter(State.WATCH, startMs);
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
            case WATCH:
                if (!robot.seesCell(watchedCell)) {
                    seenSinceMs = -1;
                } else if (seenSinceMs < 0) {
                    seenSinceMs = now;
                }

                if (seenSinceMs >= 0 && now - seenSinceMs >= TIP_CONFIRM_MS) {
                    tipResult = String.format(Locale.US, "TIPPED at %.1f s: shooting", (now - startMs) / 1000.0);
                    enter(State.SPIN_UP, now);
                } else if (inState >= WAIT_MS) {
                    tipResult = "no tip in time: parking with the balls";
                    enter(State.PARK, now);
                }
                break;
            case SPIN_UP:
                if (inState >= HardcodedAuto.FIRST_SHOT_AT_MS) enter(State.FEED, now);
                break;
            case FEED:
                if (inState >= HardcodedAuto.FEED_MS) enter(State.DWELL, now);
                break;
            case DWELL:
                if (inState >= Shoot.FINAL_DWELL_MS) enter(State.PARK, now);
                break;
            case PARK:
                if (inState >= PARK_MS) enter(State.DONE, now);
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
    }

    @Override
    public void stop() {
        if (robot != null) robot.limelight.stop();   // or the camera keeps being polled after the OpMode
        // No motor writes here: the SDK rejects them from stop() and stops the motors itself.
    }

    /** Changes state and sets that state's outputs once, on entry. */
    private void enter(State next, long now) {
        state = next;
        stateSinceMs = now;
        forward = 0;
        strafe = 0;
        turn = 0;
        switch (next) {
            case WATCH:
                seenSinceMs = -1;
                break;
            case SPIN_UP:
                robot.shooter.arm();
                break;
            case FEED:
                robot.intake.feed();   // the intake and the feeder carry the balls into the wheels
                robot.shooter.feederOn();
                break;
            case DWELL:
                robot.intake.stop();
                robot.shooter.feederOff();
                break;
            case PARK:
                robot.intake.stop();
                robot.shooter.disarm();   // the wheels and the feeder
                forward = PARK_FORWARD;
                strafe = PARK_STRAFE;
                turn = PARK_TURN;
                break;
            case DONE:
            default:
                robot.intake.stop();
                robot.shooter.disarm();
                break;
        }
    }

    /** Milliseconds from a clock that only counts up (the Driver Station can change the wall clock). */
    private static long nowMs() {
        return System.nanoTime() / 1_000_000;
    }
}
