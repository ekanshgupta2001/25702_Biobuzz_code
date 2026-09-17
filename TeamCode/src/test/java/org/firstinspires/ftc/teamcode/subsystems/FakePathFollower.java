package org.firstinspires.ftc.teamcode.subsystems;

import com.pedropathing.follower.Follower;
import com.pedropathing.math.Pose;
import com.pedropathing.paths.Path;

/**
 * A {@link PathFollower} with Pedro 3.0.0's state machine and none of its motion.
 *
 * <p>Mirrors the real follower's surprises exactly: the mode is whatever the last command was,
 * {@link #atParametricEnd()} is true whenever not following, and {@link #follow} restarts the
 * path. The test decides when a path's geometry "finishes" by calling {@link #finishPath()} and
 * when a turn has arrived by calling {@link #finishTurn()}; the mode stays FOLLOW or HOLD until the
 * code under test acts, which is exactly the window the drivetrain's {@code setEnd} has to handle.
 */
public final class FakePathFollower implements PathFollower {
    public Pose pose = Pose.zero();
    public Follower.Mode mode = Follower.Mode.IDLE;
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

    /** A hold-based turn has arrived: heading snaps to the held pose's heading. */
    public void finishTurn() {
        if (lastHoldPose != null) pose = pose.withHeading(lastHoldPose.heading());
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
        atEnd = false;
    }

    @Override
    public void follow(Path path) {
        followCalls++;
        lastPath = path;
        mode = Follower.Mode.FOLLOW;
        atEnd = false;
    }

    @Override
    public boolean atParametricEnd() {
        return mode != Follower.Mode.FOLLOW || atEnd;
    }

    @Override
    public void hold(Pose pose, boolean scaled) {
        holdCalls++;
        lastHoldPose = pose;
        lastScaled = scaled;
        mode = Follower.Mode.HOLD;
    }

    @Override
    public Follower.Mode mode() {
        return mode;
    }
}
