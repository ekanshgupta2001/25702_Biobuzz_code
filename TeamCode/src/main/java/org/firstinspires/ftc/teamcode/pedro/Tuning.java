package org.firstinspires.ftc.teamcode.pedro;

import com.pedropathing.algorithm.Foresight;
import com.pedropathing.revhub.drivetrains.Mecanum;
import com.pedropathing.revhub.localizers.PinpointLocalizer;
import com.pedropathing.tuning.autotune.Procedure;
import com.pedropathing.tuning.autotune.Tuner;

import org.firstinspires.ftc.teamcode.pedro.procedures.ForesightTuner;
import org.firstinspires.ftc.teamcode.pedro.procedures.MecanumTuner;
import org.firstinspires.ftc.teamcode.pedro.procedures.PinpointTuner;
import org.firstinspires.ftc.teamcode.pedro.procedures.ShooterTuner;
import org.firstinspires.ftc.teamcode.pedro.procedures.Tests;

/**
 * AutoTune registration. {@code TunerScanner} finds every {@code @Tuner} method here when the Robot
 * Controller app starts and <em>invokes it then</em>, so each factory must be {@code static}, take no
 * arguments, be declared to return exactly {@link Procedure}, and never throw. A config that is
 * still {@code null} is therefore only ever dereferenced inside a lambda that runs when the tuner is
 * used, or answered with a {@link NotReady} procedure that says what to paste first.
 *
 * <p>Tuning order (docs/01 section A.7): Mecanum Tuner → Tests/Driving → Pinpoint Tuner →
 * Tests/Odometry → Foresight Tuner → Tests/Hold, Line, Curve. Each tuner ends with a code block to
 * paste into {@link Constants}. The web UI is {@code http://192.168.43.1:10158} on the robot's Wi-Fi.
 *
 * <p>The Shooter Tuner is ours, not the Quickstart's: it times flywheel spin-ups for kS / kV / kP and
 * ends with a block to paste into {@code Shooter.java}. This site is the only tuning tool in the repo.
 *
 * <p>BIOBUZZ R704: AutoTune's web servers are always bound while the library is on the classpath, so
 * before an event this file, {@code pedro/procedures/*} and the {@code tuning} dependency are removed
 * by hand. See {@code build.dependencies.gradle} and HANDOFF section 4.
 */
public class Tuning {
    @Tuner
    public static Procedure mecanumTuner() {
        return new MecanumTuner();
    }

    @Tuner
    public static Procedure pinpointTuner() {
        return new PinpointTuner();
    }

    /** Needs {@code Constants.localizerConfig}: the Foresight tuner drives on the tuned localizer. */
    @Tuner
    public static Procedure foresightTuner() {
        if (Constants.localizerConfig == null) {
            return new NotReady("Foresight Tuner",
                    "Run the Pinpoint Tuner first, paste its PinpointConfig into Constants.localizerConfig, redeploy.");
        }
        // Argument order here is (localizer, drivetrain); Tests below is (drivetrain, localizer, algorithm).
        return new ForesightTuner(
                h -> new PinpointLocalizer(h, Constants.localizerConfig),
                h -> new Mecanum(h, Constants.drivetrainConfig));
    }

    /** Offers only the tests whose dependencies exist: Driving always, the rest as configs are pasted in. */
    @Tuner
    public static Procedure tests() {
        return new Tests(
                h -> new Mecanum(h, Constants.drivetrainConfig),
                Constants.localizerConfig == null ? null : h -> new PinpointLocalizer(h, Constants.localizerConfig),
                Constants.foresightConfig == null ? null : () -> new Foresight(Constants.foresightConfig));
    }

    /** Flywheel spin-up timing, for {@code Shooter.kS / kV / kP}. Needs no Pedro config. */
    @Tuner
    public static Procedure shooterTuner() {
        return new ShooterTuner();
    }

    /** A listed tuner that, when run, says what to do instead of failing with a NullPointerException. */
    private static final class NotReady extends Procedure {
        private final String message;

        NotReady(String name, String message) {
            super(name, "Not available yet: " + message);
            this.message = message;
        }

        @Override
        public void run() throws InterruptedException {
            abort(message);
        }
    }
}
