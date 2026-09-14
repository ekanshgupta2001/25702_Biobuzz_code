package org.firstinspires.ftc.teamcode.subsystems;

import com.pedropathing.follower.Follower;
import com.pedropathing.math.Pose;
import com.pedropathing.paths.Path;

/**
 * A {@link PathFollower} with Pedro 3.0.0's state machine and none of its motion.
 *
 * <p>Mirrors the real follower's surprises exactly: the mode is whatever the last command was,
 * {@link #atParametricEnd()} is true whenever not following, {@link #isBusy()} is set by
 * {@link #follow} and only cleared by {@link #settle()} (Pedro clears it inside a hold), and
 * {@link #follow} restarts the path. The test decides when a path's geometry "finishes" by calling
 * {@link #finishPath()}; the mode stays FOLLOW until the code under test acts, which is exactly
 * the window the drivetrain's {@code setEnd} has to handle.
 */
public final class FakePathFollower implements PathFollower {
    public Pose pose = Pose.zero();
    public Follower.Mode mode = Follower.Mode.IDLE;
    public boolean busy = false;
    public boolean atEnd = false;

    public Path lastPath = null;
    public Pose lastHoldPose = null;
    public boolean lastScaled = false;
    public double lastForward = 0, lastStrafe = 0, lastTurn = 0;

    public int updateCalls = 0;
    public int followCalls = 0;
    public int manualCalls = 0;
    public int holdCalls = 0;
    public int stopCalls = 0;
    public int setPoseCalls = 0;

    /** The path geometry is done; Pedro's own tick would hold or idle next. */
    public void finishPath() {
        atEnd = true;
    }

    /** A hold has settled within tolerance, clearing Pedro's busy flag. */
    public void settle() {
        busy = false;
    }

    /** A hold-based turn has arrived: heading snaps to the held pose's heading. */
    public void finishTurn() {
        if (lastHoldPose != null) pose = pose.withHeading(lastHoldPose.heading());
        busy = false;
    }

    @Override
    public void update() {
        updateCalls++;
    }

    @Override
    public Pose pose() {
        return pose;
    }

    @Override
    public void setPose(Pose pose) {
        this.pose = pose;
        setPoseCalls++;
    }

    @Override
    public void manual(double forward, double strafe, double turn) {
        manualCalls++;
        lastForward = forward;
        lastStrafe = strafe;
        lastTurn = turn;
        mode = Follower.Mode.MANUAL;
        busy = false;
        atEnd = false;
    }

    @Override
    public void follow(Path path) {
        followCalls++;
        lastPath = path;
        mode = Follower.Mode.FOLLOW;
        busy = true;
        atEnd = false;
    }

    @Override
    public boolean atParametricEnd() {
        return mode != Follower.Mode.FOLLOW || atEnd;
    }

    @Override
    public boolean isBusy() {
        return busy;
    }

    @Override
    public void hold(Pose pose, boolean scaled) {
        holdCalls++;
        lastHoldPose = pose;
        lastScaled = scaled;
        mode = Follower.Mode.HOLD;
        busy = true;
    }

    @Override
    public void stop() {
        stopCalls++;
        mode = Follower.Mode.IDLE;
        busy = false;
    }

    @Override
    public Follower.Mode mode() {
        return mode;
    }
}
