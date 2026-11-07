package org.firstinspires.ftc.teamcode.commands;

import com.pedropathing.ivy.Command;
import com.qualcomm.robotcore.util.RobotLog;

import org.firstinspires.ftc.teamcode.subsystems.Intake;
import org.firstinspires.ftc.teamcode.subsystems.Shooter;

/**
 * The shooting cycle: wait once for both wheels to reach speed, then run the intake and the up-wheels
 * nonstop, so the balls go through one after another.
 *
 * <pre>
 *   SPIN_UP   wait until both wheels are at speed (or SPIN_UP_TIMEOUT_MS, then feed anyway)
 *   FEED      intake + up-wheels on, for feedMs (Teleop: until X cancels it)
 *   DWELL     feed off, wheels still on a moment so the last ball leaves
 *   DONE
 * </pre>
 *
 * <h2>The wheels still recover between balls</h2>
 * This does not wait between balls, and it does not need to for the wheels to recover: the shooter's
 * own control law ({@code Shooter.update()}, every loop) pushes harder whenever a ball slows a wheel.
 * The flywheel hold rides alongside this command ({@code Shooter.armedCommand()} in {@code Macros}),
 * so the wheels stay armed the whole time.
 *
 * <h2>What it requires</h2>
 * The intake only. The up-wheels are switched through the shooter, which nothing else touches while
 * the shot holds it, and the drivetrain is left alone so the driver keeps driving through a shot.
 * Feeding uses {@code Intake.Mode.FEED}: anti-jam is off, so a jammed ball stalls the intake until the
 * feed ends. That is why Teleop's feed is still bounded ({@code Macros.SHOOT_TIMEOUT_MS}).
 */
public class Shoot {
    /** Pass as {@code feedMs} to feed until the command is cancelled (Teleop's X). */
    public static final long UNTIL_CANCELLED = Long.MAX_VALUE;

    /** How long autonomous feeds for: long enough for all four pre-loads to go through. */
    public static long AUTO_FEED_MS = 4000;
    /** Spin-up deadline. Past this it feeds anyway: a ball kept on the robot scores nothing. */
    public static long SPIN_UP_TIMEOUT_MS = 3000;
    /** After the feed ends, long enough for the last piece to leave before the wheels are released. */
    public static long FINAL_DWELL_MS = 150;

    /**
     * Not used by the macro any more (it feeds nonstop). Kept because both shooter tuners use it: their
     * test shot is one pulse of this length.
     */
    public static long FEED_PULSE_MS = 400;
    /**
     * Not used by the macro any more (it does not wait between balls). Kept as the shooter tuners'
     * yardstick for a recovery that is too slow.
     */
    public static long RECOVER_TIMEOUT_MS = 900;

    private enum State { SPIN_UP, FEED, DWELL, DONE }

    private final Intake intake;
    private final Shooter shooter;
    private final long feedMs;

    private State state = State.DONE;
    private long stateSinceMs = 0;
    private boolean fedFull = false;

    public Shoot(Intake intake, Shooter shooter, long feedMs) {
        this.intake = intake;
        this.shooter = shooter;
        this.feedMs = feedMs;
    }

    /** The cycle's step, for the telemetry card: SPIN_UP, FEED, DWELL or DONE. */
    public String getStateName() {
        return state.name();
    }

    /** True once the feed ran its whole time (as opposed to being cut short). */
    public boolean completed() {
        return fedFull;
    }

    /**
     * The command. Requires the intake for the whole cycle so the default idle command cannot
     * re-assert "stopped" while feeding.
     */
    public Command build() {
        return Command.build()
                .setStart(this::start)
                .setExecute(this::step)
                .setDone(() -> state == State.DONE)
                .setEnd(ec -> {
                    intake.stop();
                    shooter.feederOff();
                    state = State.DONE;
                })
                .requiring(intake);
    }

    private void start() {
        fedFull = false;
        // A zero target would make atTarget() true on the first tick (|0 - 0| < tolerance) and the
        // tunnel would push pieces into a dead flywheel. If there is no speed, do nothing rather than
        // dump the load on the floor.
        if (feedMs <= 0 || shooter.getTarget() <= 0) {
            state = State.DONE;
            return;
        }
        enter(State.SPIN_UP);
    }

    private void enter(State next) {
        state = next;
        stateSinceMs = System.currentTimeMillis();
        RobotLog.ii("Shoot", "-> %s  fly %.0f/%.0f  roller %.0f/%.0f", next,
                shooter.getVelocity(), shooter.getTarget(), shooter.getRollerVelocity(), shooter.getRollerTarget());
        // The action for a state is applied on entry, so a transition costs no loop.
        if (next == State.FEED) {
            intake.feed();
            shooter.feederOn();
        } else {
            intake.stop();
            shooter.feederOff();
        }
    }

    private long elapsed() {
        return System.currentTimeMillis() - stateSinceMs;
    }

    /**
     * One pass of the machine. Every transition is chained in this same call, so a state whose exit
     * condition is already true does not cost a loop.
     */
    private void step() {
        // Bounded so a single call cannot spin forever if two states ever transition into each other.
        for (int guard = 0; guard < State.values().length + 1; guard++) {
            switch (state) {
                case SPIN_UP:
                    if (shooter.atTarget() || elapsed() >= SPIN_UP_TIMEOUT_MS) {
                        enter(State.FEED);
                        continue;
                    }
                    return;
                case FEED:
                    if (elapsed() >= feedMs) {
                        fedFull = true;
                        enter(State.DWELL);
                        continue;
                    }
                    return;
                case DWELL:
                    if (elapsed() >= FINAL_DWELL_MS) {
                        enter(State.DONE);
                        continue;
                    }
                    return;
                default:
                    return;
            }
        }
    }
}
