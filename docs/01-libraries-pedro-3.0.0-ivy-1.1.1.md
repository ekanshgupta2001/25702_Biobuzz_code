# Libraries: Pedro Pathing 3.0.0 and Ivy 1.1.1

Reference for the two libraries this codebase is built on. Everything here was verified against the
artifacts in the Gradle cache: the `com.pedropathing:core:3.0.0` bytecode (javap + disassembly, no
sources are published for core), the `revhub:3.0.0` and `tuning:1.0.0` sources, the
`com.pedropathing.ivy:core:1.1.1` bytecode, the `ivy:pedro:1.1.1` source, and the Ivy 1.0.0 sources
(for scheduler semantics). Where something is inferred rather than read, it is marked **[inferred]**.

Re-checked on 2026-09-16 against the pedropathing.com documentation source (`Docs-master/`, the
site's `content/docs` tree). The site is wrong in three places, each noted where it matters
(A.4 stick signs, A.6 the custom-drivetrain page, B.1 the `follow` done condition); where the
site and the bytecode disagree, the bytecode wins and the code follows the bytecode.

Companion docs: `02-robot-physical-architecture.md` (what the robot is),
`03-software-architecture.md` (how the code is organised and how it uses these libraries).

---

## 0. Dependencies

`build.dependencies.gradle`:

```gradle
repositories {
    mavenCentral()
    maven { url 'https://repo.dairy.foundation/releases/' }
    google()
}
dependencies {
    // FTC SDK 11.2.1 ...
    implementation 'com.pedropathing:revhub:3.0.0'      // FTC hardware layer; pulls core:3.0.0
    implementation 'com.pedropathing:tuning:1.0.0'      // AutoTune web tuner
    implementation 'com.pedropathing.ivy:pedro:1.1.1'   // Ivy Pedro glue; pulls ivy core:1.1.1
}
```

| Artifact | Packaging | Depends on | Notes |
|---|---|---|---|
| `com.pedropathing:core:3.0.0` | jar, pure JVM | nothing | Follower, Foresight, paths, math, config. Unit-testable off-robot. |
| `com.pedropathing:revhub:3.0.0` | aar | core 3.0.0, kotlin-stdlib 2.3.21 | `Mecanum`, `PinpointLocalizer`, other localizers. Package root `com.pedropathing.revhub`. |
| `com.pedropathing:tuning:1.0.0` | aar | kotlin-stdlib; runtime: revhub 3.0.0, Sloth 0.3.0, jnanoid, nanohttpd-websocket | AutoTune. Sloth/Sinister does classpath scanning and app-lifecycle hooks. |
| `com.pedropathing.ivy:core:1.1.1` | **jar**, zero deps | nothing | Command scheduler. Java packages are still `com.pedropathing.ivy.*`. |
| `com.pedropathing.ivy:pedro:1.1.1` | aar | ivy core 1.1.1 | One class: `PedroCommands`. Needs Pedro core on the classpath. |

Java level: `build.common.gradle` compiles at Java 8 (`sourceCompatibility 1.8`), `minSdkVersion 24`.
Core is class-file version 52 (Java 8) and uses only lambdas and default methods. The Quickstart's
own `pedro/procedures/*Tuner.java` use `java.util.List.of(...)` (Java 9 / API 30) in nine places;
D8 normally backports these, but if a tuner dies with `NoSuchMethodError: java.util.List.of`, replace
with `Arrays.asList(...)` or `com.pedropathing.utils.Utils.listOf(...)`.

---

# Part A — Pedro Pathing 3.0.0

## A.1 Building a Follower

There is exactly one constructor:

```java
public Follower(com.pedropathing.localization.Localizer localizer,
                com.pedropathing.drivetrain.Drivetrain drivetrain,
                com.pedropathing.algorithm.Algorithm algorithm)
```

The order is **Localizer, Drivetrain, Algorithm**. The comment in the Quickstart's `pedro/Constants.java`
(`new Follower(Drivetrain, Localizer, Foresight)`) is wrong; the compiler will reject a swap because the
three types are unrelated, but do not copy the comment.

- `Localizer` — pose and velocity source. `Follower.update()` ticks it for you.
- `Drivetrain` — consumes `DrivePowers(forward, strafe, turn)` in the robot frame and writes motors.
- `Algorithm` — the controller. `Foresight` is the only implementation shipped in 3.0.0.

### The config idiom

Every config object is filled through a lambda that runs eagerly inside the constructor:

```java
public interface Configuration<T> { void configure(T config); }
MecanumConfig cfg = new MecanumConfig(c -> { c.frontLeftName.set("lf"); /* ... */ });
```

`ConfigVar<T>`: `required()` (throws `IllegalStateException` on `get()` until set), `of(default)`,
`set(v)` (runs validators, `IllegalArgumentException` on failure), `get()`, and `at(v)` which returns a
`Modifier` for a temporary per-path override (see A.5). Validators: `positive()`, `nonnegative()`,
`negative()`, `nonpositive()`, `nonnull()`.

### `MecanumConfig` (`com.pedropathing.revhub.drivetrains`)

| Field | Type | Default |
|---|---|---|
| `frontLeftName`, `backLeftName`, `frontRightName`, `backRightName` | `ConfigVar<String>` | required |
| `frontLeftDirection`, `backLeftDirection`, `frontRightDirection`, `backRightDirection` | `ConfigVar<DcMotorSimple.Direction>` | required |
| `manualBrakeMode` | `ConfigVar<Boolean>` | `true` — BRAKE while in manual mode |
| `powerThreshold` | `ConfigVar<Double>` | `0.01` — smallest power change that triggers a hardware write |

No ticks-per-rev, wheel size, max power or voltage compensation live here any more. **`Mecanum` owns its motors, and two instances fight.** Its constructor calls
`map.get(DcMotorEx.class, name)` for all four names itself and wraps each in its own `CachedMotor`,
which skips a `setPower` it believes is already set. So two `Mecanum` objects over the same four
ports hold two independent power caches: writes through one are invisible to the other, and a motor
written behind cache A cannot be stopped through cache B. Build **one** and pass that instance to
`new Follower(localizer, mecanum, algorithm)` — the `Follower` takes the `Drivetrain` interface, so
stick driving via `mecanum.drive(powers, true)` and path following share one motor layer. The same
trap catches a bench or diagnostic OpMode that resolves its own `DcMotorEx` for a port a subsystem
already holds.

`Mecanum` mixes
`fl = f - s - t`, `fr = f + s + t`, `bl = f + s - t`, `br = f - s + t`, so **+strafe is robot-left and
+turn is counter-clockwise**. It normalises down only, never up. Path following and holding run the
motors in **FLOAT**; only manual mode uses BRAKE (when `manualBrakeMode` is true).

### `PinpointConfig` (`com.pedropathing.revhub.localizers`)

| Field | Type | Default |
|---|---|---|
| `name` | `ConfigVar<String>` | required |
| `xPodDirection`, `yPodDirection` | `ConfigVar<GoBildaPinpointDriver.EncoderDirection>` | required |
| `xPodOffset`, `yPodOffset` | `ConfigVar<Double>` | required |
| `offsetUnits`, `globalDistanceUnit`, `encoderResolutionUnit` | `ConfigVar<DistanceUnit>` | `INCH` |
| `podType` | `ConfigVar<GoBildaOdometryPods>` | `goBILDA_4_BAR_POD` |
| `ticksPerUnit` | `ConfigVar<OptionalDouble>` | empty (wins over `podType` if present) |
| `resetMode` | `ConfigVar<PinpointLocalizer.ResetMode>` | `RECALIBRATE_IMU` (`RESET_AND_RECALIBRATE_IMU`, `NONE`) |

**The `PinpointLocalizer` constructor calls `reset()` and starts an IMU calibration.** Every Quickstart
tuner sleeps 1000 ms after constructing it, before `waitForStart()`. In an iterative `OpMode`, build the
follower in `init()` and re-zero the pose in `start()`; never construct and immediately read.

### `ForesightConfig` (`com.pedropathing.algorithm`)

Twelve fields are **required** and all of them are produced by the ForesightTuner:

| Required | Type |
|---|---|
| `headingFeedback` | `Controller` |
| `forwardTranslational`, `strafeTranslational` | `Controller` (typically `Controller.piecewise(small).put(2.5, large)`) |
| `brake`, `coast` | `Controller` (typically `Controller.proportionalFeedforward(kV)`) |
| `linearBrakeCoefficients`, `quadraticBrakeCoefficients` | `Matrix` (`Matrix.diag(forward, strafe)`) |
| `headingBrakeCoefficients` | `Vector2D` (`Vector2D.cartesian(linear, quadratic)`) |
| `maxAchievableForwardVelocity`, `maxAchievableStrafeVelocity` | `Double` (in/s) |
| `naturalForwardDeceleration`, `naturalStrafeDeceleration` | `Double` (in/s²) |

Defaults worth knowing (all `ConfigVar`, all overridable per path):

| Field | Default | Meaning |
|---|---|---|
| `headingStaticFF` | `staticFeedforward(0)` | |
| `holdPointTranslationalScaling` / `holdPointHeadingScaling` | `0.45` / `0.35` | applied only by `hold(pose, true)` |
| `maxBrakingPower` | `0.2` | |
| `maxPathSpeed`, `maxVelocityConstraint`, `maxAccelerationConstraint`, `maxDecelerationConstraint`, `maxDecelerationScale` | `+∞` (`Constraint.NONE`) | the per-path speed limits |
| `brakeAggression` | `1.0` | |
| `coastDownToVelocity` | `0.0` | |
| `headingDeviationTolerance` | `toRadians(11.25)` | |
| `translationalDeviationTolerance` | `2.5` in | |
| `brakeAtEnd`, `pathSkip` | `true`, `true` | |
| `headingDriveRatio` | `0.5` | |
| `cosineScale` | `false` | |
| `minCorrectionDistance` | `0.001` | |
| `parametricTConstraint` | `0.025` | `atParametricEnd()` fires at `t >= 1 - 0.025` |
| `headingConstraint` / `translationalConstraint` / `velocityConstraint` | `0.007` rad / `0.1` in / `0.1` in/s | hold "settled" tolerances |
| `timeoutConstraint` | `100.0` **ms** | hold settle timeout |

`new Foresight(cfg)` eagerly reads the two `natural*Deceleration` vars (throws at construction if unset);
the other required vars are read on the first `update()` after `follow()`. Construct one `Foresight` in
`init()` to smoke-test a config.

### `Controller` (`com.pedropathing.controllers`)

`double calculate(double target, double error)`. Factories: `Controller.zero`, `proportional(kP)`,
`proportionalFeedforward(kV)` (`kV * target`), `staticFeedforward(kS)`, `staticTargetFeedforward(kS)`,
`integral(kI[, iZone, decay, maxI])`, `derivative(kD)`, `pid(kP, kI, kD)` (returns a mutable
`PIDController`), `sum(...)`, `piecewise(base)` (a `PiecewiseController`; `put(threshold, c)` keys on
`|error|`, `floorEntry`). Defaults: `.plus()`, `.minus()`, `.times(k)`, `.reset()`.
`PIDController.calculate(target, error, derivative)` **subtracts** `kD * derivative` (expects a
measurement derivative, e.g. `follower.twist().omega`).
`com.pedropathing.controllers.filters.KalmanFilter(modelCov, dataCov)`: `update(d)`, `update(d, meas)`,
`state()`, `reset(...)`.

### Worked example: mecanum + goBILDA Pinpoint

This is the shape the Quickstart itself used before `Constants.java` was blanked for the 3.0.0
template. Recover it with `git show 20768b3:TeamCode/src/main/java/org/firstinspires/ftc/teamcode/pedro/Constants.java`.
The gains below are that robot's, not ours.

```java
public class Constants {
    public static MecanumConfig drivetrainConfig = new MecanumConfig(c -> {
        c.frontLeftName.set("lf");  c.backLeftName.set("lr");
        c.frontRightName.set("rf"); c.backRightName.set("rr");
        c.frontLeftDirection.set(DcMotorSimple.Direction.REVERSE);
        c.backLeftDirection.set(DcMotorSimple.Direction.REVERSE);
        c.frontRightDirection.set(DcMotorSimple.Direction.FORWARD);
        c.backRightDirection.set(DcMotorSimple.Direction.FORWARD);
        c.manualBrakeMode.set(true);
    });

    public static PinpointConfig localizerConfig = new PinpointConfig(c -> {
        c.name.set("pinpoint");
        c.xPodOffset.set(2.187);  c.yPodOffset.set(-4.572);
        c.xPodDirection.set(GoBildaPinpointDriver.EncoderDirection.FORWARD);
        c.yPodDirection.set(GoBildaPinpointDriver.EncoderDirection.FORWARD);
    });

    public static ForesightConfig foresightConfig = new ForesightConfig(c -> {
        c.forwardTranslational.set(Controller.piecewise(Controller.proportional(0.1)).put(2.5, Controller.proportional(0.3)));
        c.strafeTranslational.set(Controller.piecewise(Controller.proportional(0.1)).put(2.5, Controller.proportional(0.3)));
        c.coast.set(Controller.proportionalFeedforward(0.01098));
        c.brake.set(Controller.proportionalFeedforward(0.00873));
        c.headingFeedback.set(Controller.proportional(5.2587));
        c.headingBrakeCoefficients.set(Vector2D.cartesian(0.05642, 0.00638));
        c.linearBrakeCoefficients.set(Matrix.diag(0.10606, 0.08719));
        c.quadraticBrakeCoefficients.set(Matrix.diag(0.001466, 0.001384));
        c.maxAchievableForwardVelocity.set(72.73);
        c.maxAchievableStrafeVelocity.set(52.34);
        c.naturalForwardDeceleration.set(85.01);
        c.naturalStrafeDeceleration.set(104.50);
    });

    public static Follower create(HardwareMap h) {
        return new Follower(new PinpointLocalizer(h, localizerConfig),
                            new Mecanum(h, drivetrainConfig),
                            new Foresight(foresightConfig));
    }
}
```

Static config objects are shared mutable state: `ConfigVar.set()` and per-path `Modifier`s mutate the
same instance for every `Foresight` in the process. One robot per OpMode makes that fine.

`revhub`'s `Loggers.telemetryLog(telemetry)` returns a `Consumer<String>`, but the only sink on
`Follower` is `withLogger(Consumer<FollowerLog>)`. Adapt: `follower.withLogger(log -> sink.accept(log.toString()))`.

## A.2 Loop contract

```java
Follower follower = Constants.create(hardwareMap);   // init(): builds hardware, starts IMU calibration
follower.setPose(startPose);
// ... at least 1 s later (start()):
follower.setPose(startPose);
follower.update();                                   // priming tick: first dt is 0
follower.follow(path);

// every loop:
follower.update();                                   // exactly once, first
// then read follower.pose(), atParametricEnd(), etc.
```

`update()` clears the debug snapshot, calls `localizer.update()` (**do not call it yourself too**),
then dispatches on mode:

| Mode | What `update()` does |
|---|---|
| `FOLLOW` | if the tracker is done: `holdEnd ? hold(endPose) : stop()`; else drive with `algorithm.calculatePath(...)`, `manual=false` (FLOAT) |
| `HOLD` | drive with `algorithm.calculateHold(...)`, `manual=false` |
| `MANUAL` | `drivetrain.drive(manualPowers, true)` (BRAKE if configured) |
| `IDLE` | `drivetrain.stop()` |

`update(double dt)` exists for a fixed timestep (JVM tests). `dt` is computed from `System.nanoTime()`
otherwise.

## A.3 `Follower` API

**Commanding** — each of these clears the previous state (releasing any per-path modifiers):

| Method | Effect |
|---|---|
| `follow(Path)` | mode `FOLLOW`, new `PathTracker`, `algorithm.reset()`. Restarts from t = 0; there is no resume. |
| `hold(Pose)` | mode `HOLD`, **unscaled** (full authority). |
| `hold(Pose, boolean useHoldScaling)` | `true` applies the 0.45 / 0.35 scalings — the gentle hold. |
| `manual(DrivePowers)` / `manual(f, s, t)` | mode `MANUAL`; powers are latched, so call every loop. |
| `stop()` | mode `IDLE`. |

**Queries**

| Method | Meaning |
|---|---|
| `mode()` | `Follower.Mode` — `FOLLOW`, `HOLD`, `MANUAL`, `IDLE`; plus `following()`, `holding()`, `manual()`, `idle()` |
| `atParametricEnd()` | `true` if not in `FOLLOW`; `false` while more than one segment remains; else `t >= 1 - parametricTConstraint`. **This is "the path is done".** |
| `isBusy()` | `Foresight.busy`, set by `follow()` and cleared **only inside HOLD** once settled within the three constraints or after `timeoutConstraint` ms. Always `true` during `FOLLOW`; never clears if `holdEnd` is false. Use it for "settled at the endpoint", never for "still following". |
| `pose()` | inches, radians, heading normalised to `[0, 2π)` |
| `velocity()` | **field frame** `Velocity(vx, vy, omega)` |
| `twist()` | **robot frame** `Twist(vx=forward, vy=left, omega)` |
| `setPose(Pose)`, `setX/setY/setHeading` | write the localizer. There is no `setStartingPose`. |
| `completion()`, `parametricCompletion()`, `remainingDistance()`, `distanceToEndpoint()` | progress (`completion` is arc-length fraction; `parametricCompletion` is raw t) |
| `pathIndex()` | current segment index (`-1` when idle) |
| `currentPath()`, `currentSegment()`, `currentCurve()`, `closestPose()`, `closestTangent()`, `closestNormal()`, `curvature()`, `tangentialVelocity()` | path introspection; `null`/0 when idle |
| `poseAt(distance)`, `tangentAt(d)`, `normalAt(d)`, `curvatureAt(d)` | lookahead by **distance along the curve** |
| `holdEnd` | `public final ConfigVar<Boolean>`, default `true`: hold at the endpoint when a path finishes |
| `withLogger(Consumer<FollowerLog>)`, `debug()`, `createDebug()` | `FollowerLog` has `followState`, `localizer`, `drivetrain`, `algorithm` maps |
| `algorithm()`, `setAlgorithm(Algorithm)`, `localizer`, `drivetrain` | collaborators |

**Removed from 2.x with no direct equivalent:** `setMaxPower` (use `path.with(cfg.maxPathSpeed.at(p))`),
`turnTo`/`turn` (use `hold(pose().withHeading(target))`), `atPose` (compute `pose().distance(target)`),
`startTeleopDrive`/`setTeleOpDrive` (use `manual(...)`), `breakFollowing` (`stop()`),
`pausePathFollowing`/`resumePathFollowing`, `isRobotStuck`, path callbacks, `getTranslationalError`/
`getHeadingError` (cast `algorithm()` to `Foresight` for `translationalError()`/`headingError()`).

## A.4 Teleop with `ManualDrive`

`com.pedropathing.follower.ManualDrive` is a static utility; it never touches the follower's mode.

```java
// Robot-centric
follower.manual(-gamepad1.left_stick_y, -gamepad1.left_stick_x, -gamepad1.right_stick_x);

// Field-centric: pass field-relative sticks, get robot-relative powers back
follower.manual(ManualDrive.fieldCentric(fwd, strafe, turn, follower.pose().heading()));
// overload with a heading offset: fieldCentric(fwd, strafe, turn, heading, offset)

// Heading lock: replaces `turn` with a controller output
DrivePowers p = ManualDrive.headingLock(follower, Controller.pid(kP, 0, kD), powers, targetHeadingRad);
// 6/7-arg overloads add predictive braking (brakeLinear, brakeQuadratic[, scalar=0.5]) and clamp the
// turn output to [-0.3, 1.0] — asymmetric and hard-coded.
```

Switching: `follower.follow(path)` hands control to the path; `follower.manual(DrivePowers.zero())`
takes it back. Always keep the single `follower.update()` per loop.

**Stick signs (the docs page is wrong).** `guide/teleop-usage.mdx` passes `gamepad1.left_stick_x`
and `gamepad1.right_stick_x` to `manual` un-negated. With the `Mecanum` mixing above (+strafe is
robot-left, +turn is counter-clockwise) that inverts strafe and turn: stick-right would strafe left
and turn left. The Quickstart's own `procedures/Tests.java` negates all three
(`-left_stick_y, -left_stick_x, -right_stick_x`), and so does `opmodes/teleop/Controls`. Do not
"fix" `Controls` toward the docs page; the check that settles it on the robot is the SDK's
TestHardware or the Tests/Driving procedure.

## A.5 Paths

Units: inches and radians. Axes: +x forward at the zero pose, +y left, heading CCW. `Pose` normalises
heading to `[0, 2π)` in its constructor; display with `Angle.normalizeSigned(h)` and compare with
`Angle.error(a, b)`, never with subtraction.

```java
new Pose(x, y, heading); new Pose(x, y); Pose.zero(); Pose.interpolate(a, b, t)
pose.x(); pose.y(); pose.heading();            // not getX()/getY()/getHeading()
pose.withX(..); withY(..); withHeading(..); distance(other); toVector2D(); exp(Twist, dt); log()
Vector2D.cartesian(x, y); .polar(r, theta); .unit(theta); .zero()   // no public constructor
```

### `com.pedropathing.api.Paths`

```java
Paths.line(Pose a, Pose b)            // AtomicPath over a Line
Paths.line(Vector2D a, Vector2D b)
Paths.curve(Pose... controlPoints)    // Bezier CONTROL points (passes through first and last only)
Paths.through(Pose... points)         // interpolating spline THROUGH every point (2 points -> line)
Paths.path(Path... paths)             // CompoundPath (the 2.x PathChain)
Paths.path(Curve curve)
```

### Heading — mandatory

A `Path` from `Paths.*` has **no heading**; `follow()` throws `IllegalStateException("Cannot resolve
segments: path has no heading.")`. All of these return a new immutable `Path`:

| Method | Heading |
|---|---|
| `.constant(h)` / `.constant(Pose)` | fixed |
| `.linear(h0, h1)` / `.linear(Pose, Pose)` | shortest-way, over **arc-length fraction** |
| `.linear(h0, h1, endT)` | linear until `endT`, then constant |
| `.tangent()` / `.reverseTangent()` | follow the curve tangent (or drive backwards) |
| `.facingPoint(Vector2D or Pose)` | always face a point |
| `.heading(Interpolator)` | anything; `Interpolator` is `(curve, t) -> heading`, so a lambda works |

`Interpolator.piecewise().until(0.5, Interpolator.tangent).until(1.0, Interpolator.constant(0))` —
keys must be strictly increasing and the chain **must end at `1.0`**, or it throws at follow time.
`Interpolator.longLinear(h0, h1)` goes the long way round. On a `CompoundPath`, a heading set on the
compound applies to the whole thing; otherwise every child must carry its own.

### Per-path constraints (what replaced `PathConstraints`)

```java
ForesightConfig cfg = Constants.foresightConfig;
Path slow = Paths.line(a, b).constant(0)
        .with(cfg.maxPathSpeed.at(0.35),
              cfg.translationalConstraint.at(0.5),
              cfg.timeoutConstraint.at(750.0));
```

The `PathTracker` applies each `Modifier` when it enters the segment and reverts it when it leaves or
when the follower changes mode. `at()` on an unset required var throws when the **segment starts**.

### Callbacks

**There are none in 3.0.0.** `ParametricCallback`, `TemporalCallback`, `PoseCallback` and
`PathBuilder.add*Callback` are gone. Poll `follower.pathIndex()` / `parametricCompletion()` in the loop,
or compose Ivy commands (Part B): `race(follow, waitUntil(() -> follower.parametricCompletion() > 0.6))`.

### `PoseFactory` (alliance mirroring)

```java
PoseFactory blue = PoseFactory.degrees();        // of(x, y, headingDegrees)
PoseFactory red  = blue.mirrorX(72).mapHeading(h -> Math.PI - h);
```

**[inferred]** `mirrorX(a)` maps `x -> 2a - x, heading -> -heading` and `mirrorY(a)` leaves heading
untouched, which is not the correct reflection of a heading. Prefer explicit `.mapX(...)` +
`.mapHeading(...)` and verify on the field. `FieldConstants.forAlliance` in our code does the same
job with the correct heading math.

### Three examples

```java
import static com.pedropathing.api.Paths.*;

// 1. Straight line, constant heading
Path forward = line(Pose.zero(), new Pose(48, 0, 0)).constant(0);

// 2. Bezier with linear heading
Path arc = curve(Pose.zero(), new Pose(48, 0), new Pose(48, 48)).linear(0, Math.toRadians(90));

// 3. Compound path with a global speed cap
Path full = path(line(start, mid).constant(0),
                 curve(mid, new Pose(60, 110), basket).linear(0, Math.toRadians(135)))
            .with(Constants.foresightConfig.maxPathSpeed.at(0.7));

follower.follow(full);
while (opModeIsActive() && !follower.atParametricEnd()) follower.update();
while (opModeIsActive() && follower.isBusy()) follower.update();   // settle in the automatic hold
```

## A.6 `Localizer`, `Drivetrain`, value types

```java
public interface Localizer {
    void setPose(Pose pose); MotionState state(); void update(); void reset();
    default Pose pose(); default Twist twist(); default Velocity velocity();
    default void setX/setY/setHeading(double); default Map<String,Object> debug();
}
public interface Drivetrain {
    void drive(DrivePowers powers, boolean manual);
    double maxScaling(DrivePowers current, DrivePowers delta);
    void stop(); void stop(boolean brake);
    Map<String,Object> debug();
    double interpolateVelocity(double xRadius, double yRadius, double theta);
    default double interpolateAcceleration(double xAccel, double yAccel, double theta);
}
```

`MotionState.ofVelocity(pose, velocity)` / `ofTwist(pose, twist)` / `withPose(pose)` (immutable).
`DrivePowers(forward, strafe, turn)`, `DrivePowers.zero()`. `Velocity` is world frame; `Twist` is body
frame; `velocity.toTwist(heading)` / `twist.toVelocity(heading)` convert.
Shipped localizers (all `(HardwareMap, XConfig)`): `PinpointLocalizer`, `OTOSLocalizer`,
`OctoQuadLocalizer`, `ThreeWheelLocalizer`, `ThreeWheelIMULocalizer`, `TwoWheelLocalizer`, and in core
`FusionLocalizer(deadReckoning, P, Q, R, historySize)` with `addMeasurement(pose, timestampNanos)`.
There is no drive-encoder localizer in 3.0.0.

The docs' "Custom Drivetrain" page (`custom/drivetrain.mdx`) still describes the 2.x abstract class
(`calculateDrive`, `runDrive`); the 3.0.0 contract is the `Drivetrain` interface above, which is
what the test fake `FakePedroDrivetrain` implements.

Because core has no dependencies, a JVM test can build a real `Follower` on a fake `Localizer` that
integrates `DrivePowers` via `Pose.exp(Twist, dt)` and a recording `Drivetrain`, driven by
`follower.update(dt)`.

## A.7 AutoTune (`tuning:1.0.0`)

**You do not write a tuning OpMode.** Sloth hooks start an HTTP server on port **10158** and a
WebSocket on **12649** when the Robot Controller app starts. Browse to `http://192.168.43.1:10158`
(Control Hub) from a device on the robot's Wi-Fi.

Registration: a `@Tuner` annotation on a **`public static` zero-arg method returning `Procedure`**.
`TunerScanner` finds them in TeamCode at startup. This is what `pedro/Tuning.java` is for:

```java
public class Tuning {
    @Tuner public static Procedure mecanumTuner()  { return new MecanumTuner(); }
    @Tuner public static Procedure pinpointTuner() { return new PinpointTuner(); }
    @Tuner public static Procedure foresightTuner() {
        return new ForesightTuner(h -> new PinpointLocalizer(h, Constants.localizerConfig),   // localizer FIRST
                                  h -> new Mecanum(h, Constants.drivetrainConfig));
    }
    @Tuner public static Procedure tests() {
        return new Tests(h -> new Mecanum(h, Constants.drivetrainConfig),                     // drivetrain FIRST
                         h -> new PinpointLocalizer(h, Constants.localizerConfig),
                         () -> new Foresight(Constants.foresightConfig));
    }
}
```

The two factories take their `Function<HardwareMap, ?>` arguments in **different orders** and the
compiler cannot catch a swap. `Tests` accepts `null` for any argument and only offers the tests whose
dependencies exist.

`Procedure` (extend it, implement `run()`; runs on a `Pedro-Tuning` thread): `inputs(title, desc)` →
`Inputs` form (`.s/.d/.i/.b/.e(name[, Enum.class])`, `.withDefault`, `.min/.max`), `awaitInputs(form)`,
`confirmation(title, msg)`, `withDisplay(Display, block)`, `runOpMode(TuningOpMode<R>)` (registers and
runs a real `LinearOpMode` from the browser, returns its result), `result(name, value)`,
`code(Language.JAVA, text)` (the paste-ready config block), `abort(msg)`.
`TuningOpMode<R>`: implement `protected R runTuningOpMode()`; `canStop` controls the web Stop button.
One session and one socket at a time.

**Tuning order for mecanum + Pinpoint:**

1. **Mecanum Tuner** — spins each wheel; you report forward/reversed; emits `MecanumConfig`.
2. **Tests → Driving** — sanity-drive with the gamepad, no localization.
3. **Pinpoint Tuner** — forward direction, strafe direction (push **left**), offsets (spin 180° CCW);
   emits `PinpointConfig`.
4. **Tests → Odometry** — automated forward/left/turn check; aborts on a flipped or mis-scaled axis.
5. **Tests → Localization / Pose** — watch the pose while pushing the robot.
6. **Foresight Tuner** — ten sub-OpModes (forward/strafe max velocity, forward/strafe natural
   deceleration, heading braking, heading kP, forward/strafe braking, forward/strafe translational);
   emits the full `ForesightConfig` block (the 12 required fields, nothing else).
7. **Tests → Hold / Line / Curve / Interpolation** — validate the tuned follower. (The Line/Curve/
   Interpolation tests ignore the `Distance` input and use 48 in; a local variable shadows the field.)

Every tuner OpMode: construct → `setPose(zero)` → `Thread.sleep(1000)` → `waitForStart()` →
`setPose(zero)` → `update()` → `follow(...)` → loop `update()`; chain on `atParametricEnd()`.

To remove AutoTune for competition, drop the `tuning` dependency (there is no runtime off switch).
In this repo the dependency is always in the build (a team choice, 2026-09-30), so before an event it
is removed by hand from `build.dependencies.gradle` together with `pedro/Tuning.java` and
`pedro/procedures/**`, the sources that import it. See HANDOFF §4.

**`@Tuner` factories run at Robot Controller start-up** (`TunerScanner` invokes every one to list
it), so they must be `public static`, zero-arg, declared to return exactly `Procedure`, and must
never throw: a factory whose config is not filled in yet returns a `Procedure` that aborts with a
message when run (`Tuning.NotReady`), not `null` and not an NPE. `Tests(drivetrain, null, null)` is
legal and exposes only the Driving test; `ForesightTuner` NPEs on a null localizer function.

## A.8 2.x → 3.0.0 migration table

| 2.x | 3.0.0 |
|---|---|
| `com.pedropathing.ftc.*` | `com.pedropathing.revhub.*` |
| `com.pedropathing.geometry.Pose` (`getX/getY/getHeading`) | `com.pedropathing.math.Pose` (`x()/y()/heading()`) |
| `geometry.Vector` (2-D polar) | `math.Vector2D` (static factories); `math.Vector` is now n-D |
| `FollowerBuilder(...).build()` / `Constants.createFollower` | `new Follower(localizer, drivetrain, algorithm)`; your own factory |
| `FollowerConstants` | removed; gains in `ForesightConfig`, motors in `MecanumConfig` |
| `MecanumConstants`, `PinpointConstants`, `*Constants` | `MecanumConfig`, `PinpointConfig`, `*Config` |
| `PIDFController` / `PIDFCoefficients` / `FilteredPIDF*` | `controllers.Controller`, `Controller.pid(...)`, `PIDController` |
| `KalmanFilterParameters` + `control.KalmanFilter` | `controllers.filters.KalmanFilter(modelCov, dataCov)`; `getState()` → `state()` |
| `PathConstraints` | `ForesightConfig` vars + `path.with(var.at(v))` |
| `PathBuilder` / `PathChain` | `Paths.*` statics / `CompoundPath` |
| `BezierLine(a, b)` / `BezierCurve(...)` / `BezierPoint` | `Paths.line` / `Paths.curve` / `follower.hold(pose)` |
| `setConstantHeadingInterpolation` / `setLinearHeadingInterpolation` / `setTangentHeadingInterpolation` / `setReversed` | `.constant` / `.linear` / `.tangent` / `.reverseTangent` |
| `HeadingInterpolator` | `paths.interpolator.Interpolator` |
| callbacks (`addParametricCallback` etc.) | removed — poll, or Ivy |
| `follower.update()` + `updatePose()` etc. | `follower.update()` only (also ticks the localizer) |
| `followPath(chain[, maxPower][, holdEnd])` | `follow(path)`; `holdEnd.set(...)`; `path.with(maxPathSpeed.at(p))` |
| `holdPoint(pose[, scaled])` | `hold(pose[, scaled])` (default is now **unscaled**) |
| `setStartingPose` | `setPose` |
| `getPose()` / `getVelocity()` | `pose()` / `velocity()` + `twist()` |
| `isBusy()` (false while holding) | `atParametricEnd()` for path done; `isBusy()` semantics changed |
| `startTeleopDrive` / `setTeleOpDrive(f, s, t, robotCentric)` | `manual(f, s, t)` / `manual(ManualDrive.fieldCentric(...))` |
| `breakFollowing()` | `stop()` |
| `turnTo` / `turn` / `isTurning` | removed — `hold(pose().withHeading(h))` |
| `atPose(pose, xTol, yTol)` | removed — compute locally |
| `setMaxPower` | removed — `maxPathSpeed` modifier |
| `getCurrentTValue` / `getCurrentPathNumber` / `getPathCompletion` | `parametricCompletion()` / `pathIndex()` / `completion()` |
| `CoordinateSystem` / `FTCCoordinates` / `PoseConverter` | removed — `api.PoseFactory` |
| `util.NanoTimer` / `util.Timer` | `utils.Timer` |
| `math.MathFunctions` | `utils.Utils`, `utils.Angle` |
| `SelectableOpMode`-based `Tuning.java` | AutoTune `@Tuner` factories |

Nothing in 3.0.0 is `@Deprecated`; anything missing is simply gone.

## A.9 Gotchas

1. Constructor order is `(Localizer, Drivetrain, Algorithm)`.
2. `ForesightTuner(localizer, drivetrain)` vs `Tests(drivetrain, localizer, algorithm)`.
3. `update()` ticks the localizer. Calling `localizer.update()` as well double-integrates on
   dead-reckoning localizers.
4. `isBusy()` is not "is following".
5. `hold(pose)` is the aggressive, unscaled hold. The automatic end-of-path hold is also unscaled.
6. `Thread.sleep(1000)` after constructing a `PinpointLocalizer` (IMU calibration). In an iterative
   OpMode: construct in `init()`, re-zero in `start()`.
7. `List.of` in the Quickstart procedures under Java 8 source level.
8. Path following runs the motors in FLOAT; braking is commanded reverse power.
9. `Foresight` reads the two decelerations at construction and everything else on the first
   `update()` after `follow()`.
10. `Pose` normalises heading to `[0, 2π)`.
11. `Velocity` is world frame, `Twist` is robot frame.
12. A path without a heading, or a piecewise interpolator not ending at 1.0, fails at follow time.
13. `follow()` never resumes; re-plan from the current pose after an interruption.
14. `Paths.curve` = control points; `Paths.through` = waypoints.
15. AutoTune's servers are always bound while `tuning` is on the classpath.
16. `Mecanum` resolves and caches its own four motors: build one instance and share it with the
    `Follower` (A.1). Two instances, or a second `DcMotorEx` on the same port, silently disagree.

---

# Part B — Ivy 1.1.1

## B.1 What changed from 1.0.0

Core is semantically identical to 1.0.0 (every class, signature and lambda structure matches on
disassembly). Java package names are unchanged; only the Maven coordinates split. `waitMs` no longer
depends on Pedro's `Timer` (it uses `System.currentTimeMillis()` directly), which is what makes
`ivy:core` a zero-dependency JAR. `Commands.lazy/match/branch/onInterrupt`, `Groups.loop` and
`Command.proxy()` all already existed in 1.0.0; they are new **to us**, not new to the library.

The `pedro` module shrank: 1.0.0's `Follow`, `Hold`, `Turn` classes and the `follow(chain, maxPower,
holdEnd)` overloads are gone. 1.1.1 has only:

```java
PedroCommands.follow(Follower f, Path p)   // setStart(f.follow(p)), setDone(f::atParametricEnd) — no setEnd, no requirements
PedroCommands.hold(Follower f)             // hold(f, f.pose()) — pose captured at BUILD time
PedroCommands.hold(Follower f, Pose p)     // Commands.instant(f.hold(p)) — finishes on tick one
```

The docs (`ivy/pedro-commands.mdx`, `pathing/guide/path-following.mdx`) say `follow()` finishes
when the follower "is no longer busy" or `following()` turns false. The 1.1.1 source is
`setDone(follower::atParametricEnd)`: done at 97.5 % of the last segment, while the follower is
still in FOLLOW and before its own end-of-path hold. `Drivetrain.followLazyCommand` uses the same
condition and adds the requirement and the hand-back that `follow()` lacks (B.6).

## B.2 Model

```
schedule()  →  start()                 runs INLINE inside Scheduler.schedule()
Scheduler.execute() each loop:
               execute()               always called before done() is checked
               done()? → end(NATURALLY), requirements released
preempted   →  end(INTERRUPTED)  or  end(SUSPENDED)     per interruptedBehavior()
```

- `requirements()` is a `Set<Object>` of resource keys — by convention the subsystem instance
  (`.requiring(this)`). A command with no requirements never conflicts and always starts.
- Enums: `InterruptedBehavior { END (default), SUSPEND }`, `ConflictBehavior { CANCEL, OVERRIDE
  (default), QUEUE }`, `BlockedBehavior { CANCEL (default), QUEUE }`, `EndCondition { NATURALLY,
  INTERRUPTED, SUSPENDED }`. `CommandBuilder` defaults: no requirements, priority 0, `END`, `CANCEL`,
  `OVERRIDE`, `done = () -> false`.

### Scheduler algorithm (exact)

`Scheduler.schedule(cmd)`:
1. No conflicting requirements → start.
2. Any holder has **higher** priority → blocked: `QUEUE` parks it, `CANCEL` drops it silently.
   All-or-nothing: one high-priority holder blocks the whole command.
3. Any holder has **equal** priority → the incoming `conflictBehavior` decides: `OVERRIDE` interrupts
   holders and starts, `QUEUE` parks, `CANCEL` drops.
4. All holders strictly lower → interrupt them and start.

`interrupt(holder)`: `END` → `end(INTERRUPTED)` and gone; `SUSPEND` → `end(SUSPENDED)` and parked in
`suspendedCommands`.

`Scheduler.execute()` runs three phases in order: **running** (snapshot; `execute()` then `done()` →
`end(NATURALLY)`), **queued** (start any whose requirements are free — calls `start()`), **suspended**
(re-add any whose requirements are free — **does not call `start()`**). A queued priority-0 command beats
a suspended priority −1 default to a freed resource. `Scheduler.cancel(cmd)` always calls
`end(INTERRUPTED)` (ignores `SUSPEND`). `Scheduler.reset()` clears everything **without calling `end()`**.
There is no duplicate guard: scheduling the same object twice runs `start()` twice.

## B.3 API

**`Command`** (interface): `NOOP`, `build()`, the five metadata accessors, `start/done/execute/end`,
defaults `schedule()`, `cancel()`, `isScheduled()`, `then(...)` (sequential), `with(...)` (parallel),
`raceWith(...)`, `until(cond)` (race with `waitUntil`), `unless(cond)` (**broken, see B.5**),
`proxy()` (schedules the inner command independently; its `done()` reads inverted — avoid).

**`CommandBuilder`**: `setStart(Runnable)`, `setExecute(Runnable)`, `setDone(BooleanSupplier)`,
`setEnd(Consumer<EndCondition>)`, `requiring(Object...)` (fresh `HashSet`) / `requiring(Set)` (by
reference), `setPriority(int)`, `setInterruptedBehavior`, `setBlockedBehavior`, `setConflictBehavior`.
`start/done/execute/end` are `final` — configure, do not subclass.

**`Scheduler`** (static): `schedule(cmd)`, `schedule(cmds...)`, `execute()`, `reset()`,
`isRunning(cmd)`, `isScheduled(cmd)` (running | queued | suspended), `cancel(cmd)`. State is static
and survives OpMode restarts: **`Scheduler.reset()` first thing in `init()`**, and in every test's
`@Before`/`@After`.

**`Commands`**

| Factory | Done when |
|---|---|
| `waitMs(double)` | wall-clock elapsed (`System.currentTimeMillis()`; timer starts in `start()`, keeps counting while suspended) |
| `waitUntil(BooleanSupplier)` | condition true |
| `instant(Runnable)` | immediately; body runs at `schedule()` time |
| `infinite(Runnable)` | never |
| `conditional(cond, ifTrue, ifFalse)` | chosen child done; claims **both** children's requirements, max priority |
| `branch(LinkedHashMap<BooleanSupplier, Command>)` | first true predicate's command done; immediately if none |
| `match(Supplier<E>, EnumMap<E, Command>)` | matched case done; immediately if none |
| `lazy(Supplier<Command>)` | supplied command done; immediately if `null`. **Contributes no requirements and priority 0** — always add `.requiring(...)` |
| `onInterrupt(Runnable)` | never; runs the callback on **any** end condition; use as a race/deadline child |

**`Groups`**

| Group | Done | Requirements / priority | On interrupt |
|---|---|---|---|
| `sequential(a, b, ...)` | last child done; one tick per hand-off | union / max | `SUSPENDED` ends only the current child; otherwise ends the rest. (Unguarded index: do not give a `sequential` `SUSPEND`.) |
| `parallel(...)` | all done (children stored in a `HashMap`, duplicates collapse) | union / max | ends unfinished children |
| `race(...)` | first done (iteration order is a `HashMap`'s) | union / max | ends the others with `INTERRUPTED` |
| `deadline(dl, ...)` | `dl` done | union incl. `dl` / max | ends unfinished children |
| `repeat(cmd, n or IntSupplier)` | n iterations of the **same instance** | cmd's | as sequential |
| `loop(cmd)` | never | cmd's | delegates |

Groups do **not** inherit `interruptedBehavior`, `conflictBehavior` or `blockedBehavior` from children.

## B.4 The default-command pattern

```java
Command.build()
        .setExecute(() -> { /* re-assert the idle state every tick */ })
        .setDone(() -> false)
        .setEnd(ec -> stop())
        .setPriority(-1)
        .setInterruptedBehavior(InterruptedBehavior.SUSPEND)
        .setBlockedBehavior(BlockedBehavior.QUEUE)
        .requiring(this);
```

| Setting | Why |
|---|---|
| priority −1 | any priority-0 command falls through to step 4 and preempts it unconditionally |
| `SUSPEND` | it is parked, and phase 3 of `execute()` resumes it the instant the resource frees |
| `BlockedBehavior.QUEUE` | if something already holds the resource at init, the default `CANCEL` would drop it silently for the whole match |
| logic in `setExecute` | resume never re-calls `start()` |
| `setEnd` neutralises the subsystem | runs on `SUSPENDED` too, so a macro inherits a stopped mechanism |

Schedule it exactly once, after `Scheduler.reset()`, in `init()`. Unchanged in 1.1.1.

## B.5 Traps (all verified live in 1.1.1)

1. **`unless()` never finishes.** It is `conditional(cond, NOOP, this)` and `NOOP = build()` with the
   default `done = () -> false`. Skip with `conditional(cond, real, instant(() -> {}))` instead.
2. **A builder without `setDone` runs forever** (same root cause; `infinite()` and `onInterrupt()`
   exploit it deliberately).
3. **`waitMs` is wall-clock.** It reads `System.currentTimeMillis()`, so a test cannot fake it
   without real sleeps. This repo has no test suite and uses wall-clock waits directly
   (`commands/Waits`); the injected-`Clock` indirection that existed for the deleted tests is gone.
   For reference, a fakeable wait is:
   ```java
   static CommandBuilder waitMs(Clock clock, long ms) {
       long[] t0 = new long[1];
       return Command.build().setStart(() -> t0[0] = clock.nowMs())
                             .setDone(() -> clock.nowMs() - t0[0] >= ms);
   }
   ```
4. **A direct hardware call under a default command goes nowhere**: `instant(() -> intake.intake())`
   declares no requirements, so the idle command's `execute()` re-asserts stop before the actuator
   write. Go through a factory that does `.requiring(subsystem)`, or put the requirement on the group.
5. **`Commands.lazy` adds no requirements** — it cannot suspend the default command. Always
   `lazy(...).requiring(subsystem)`.
6. **`proxy()`'s `done()` looks inverted** (`Scheduler.isScheduled(this)` without a `!`). Avoid.
7. **`sequential` with `SUSPEND`** can throw `IndexOutOfBoundsException` if suspended after its last
   child finished. Keep the default `END` on groups.
8. **`Repeat.done()` / `Repeat.end()` NPE before `start()`** (the list is built in `start`). This bites
   an ordinary `sequential(a, repeat(b, n))` that is interrupted, for example by a timeout `race`,
   before it reaches the repeat: the sequential ends its unstarted children and `Repeat.end()`
   dereferences the null list (reproduced in `AutoRoutineTest`, 2026-09-13). Unroll into guarded
   `conditional` steps, or better, write the loop as one command: the shooting cycle is
   `commands/ShootCycle`, a state machine, both for this and for the hand-off cost (trap 11).
9. **`PedroCommands.hold(follower)`** captures the pose when the command is **built**, not started. A
   hold built in `init()` drives back to the init pose. Use `lazy(() -> hold(f, f.pose())).requiring(...)`.
10. **`PedroCommands.hold(...)` is an `instant`**: it finishes on tick one and leaves the follower
    station-keeping with no owner. Our `Drivetrain.holdCommand()` keeps the resource.
11. **`PedroCommands.follow` has no `setEnd` and no requirements.** An interrupted follow leaves the
    follower in `FOLLOW`, still driving. Never schedule it bare.
12. **`setEnd` on a group replaces the group's own end.** `sequential`, `parallel`, `race` and
    `deadline` are `CommandBuilder`s whose `setEnd` is the code that ends their children; chaining
    your own `setEnd` onto one throws that away and the children are never ended. To run something
    when a group is interrupted, make it a child: `deadline(body, onInterrupt(callback))`
    (`Macros.reporting`). A natural finish fires the callback too, so the callback must be a no-op
    once the work is done.

## B.6 Using Ivy with Pedro 3.0.0 in this codebase

Keep the FTC_Guide's `Drivetrain.followLazyCommand(supplier, holdEnd)` shape and re-base it:

```java
public Command followLazyCommand(Supplier<Path> pathSupplier, boolean holdEnd) {
    long[] startedAt = new long[1]; boolean[] started = new boolean[1];
    return Command.build()
        .setStart(() -> {
            startedAt[0] = clock.nowMs();
            Path p = pathSupplier.get();                      // built from the CURRENT pose
            started[0] = p != null && follower != null;
            if (started[0]) follower.follow(p);
        })
        .setDone(() -> !started[0]                            // null supplier: finish at once
                 || (clock.nowMs() - startedAt[0] >= MIN_PATH_MS && follower.atParametricEnd()))
        .setEnd(ec -> {
            if (!started[0]) return;
            if (ec == EndCondition.NATURALLY && holdEnd) follower.hold(follower.pose());   // station-keep
            else follower.manual(0, 0, 0);                                                // hand back
        })
        .requiring(this);
}
```

It provides what `lazy(PedroCommands.follow(...))` does not: the requirement, cleanup on interrupt,
the `MIN_PATH_MS` guard (a zero-length path is at its parametric end on tick one), `holdEnd` semantics,
and a null-path exemption. (It also used to provide a `PathFollower` seam for JVM tests; that
interface is deleted and `Drivetrain` holds the `Follower` directly.)

Every Ivy import the Guide uses (`Command`, `CommandBuilder`, `Scheduler`, `behaviors.*`,
`Commands.{instant, waitMs, waitUntil, conditional}`, `Groups.{sequential, parallel, race, deadline}`)
compiles unchanged. JVM tests get the real scheduler for free: `ivy:core` is on the test classpath
transitively and has no Android dependency.
