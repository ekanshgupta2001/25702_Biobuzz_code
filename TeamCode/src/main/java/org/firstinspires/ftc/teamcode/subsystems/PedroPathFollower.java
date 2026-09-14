package org.firstinspires.ftc.teamcode.subsystems;

import com.pedropathing.follower.Follower;
import com.pedropathing.math.Pose;
import com.pedropathing.paths.Path;

/** {@link PathFollower} backed by a real Pedro {@link Follower}. Pure delegation. */
public final class PedroPathFollower implements PathFollower {
    private final Follower follower;

    public PedroPathFollower(Follower follower) {
        this.follower = follower;
    }

    /** The wrapped follower, for callers that need Pedro's full API. */
    public Follower raw() {
        return follower;
    }

    @Override
    public void update() {
        follower.update();
    }

    @Override
    public Pose pose() {
        return follower.pose();
    }

    @Override
    public void setPose(Pose pose) {
        follower.setPose(pose);
    }

    @Override
    public void manual(double forward, double strafe, double turn) {
        follower.manual(forward, strafe, turn);
    }

    @Override
    public void follow(Path path) {
        follower.follow(path);
    }

    @Override
    public boolean atParametricEnd() {
        return follower.atParametricEnd();
    }

    @Override
    public boolean isBusy() {
        return follower.isBusy();
    }

    @Override
    public void hold(Pose pose, boolean scaled) {
        follower.hold(pose, scaled);
    }

    @Override
    public void stop() {
        follower.stop();
    }

    @Override
    public Follower.Mode mode() {
        return follower.mode();
    }
}
