# Software Architecture

How the BIOBUZZ code is organised, and why. **Scope: the V1 robot** (`02-robot-physical-architecture.md`):
collect, store four, transfer, aim, shoot. Flower scoring, extensions and a diverter are V2 and have
no code here. The structure is the FTC_Guide reference architecture
(Robot / subsystems / commands / util / opmodes / game, Ivy scheduler, JVM unit tests) re-based on
Pedro Pathing 3.0.0 and Ivy 1.1.1 with no Panels dependency, and re-shaped around the mechanisms in
`02-robot-physical-architecture.md`. Library facts are in `01-libraries-pedro-3.0.0-ivy-1.1.1.md`.

Everything in the tree below is implemented and JVM-tested (HANDOFF section 2 has the numbers); the
Javadoc header of each class is its contract and this document is the map.

---

## 1. The loop contract

Every match OpMode extends `opmodes/MatchOpMode`, whose `loop()` is **final**:

```java
public final void loop() {
    loopMs = loopTimer.milliseconds(); loopTimer.reset(); loopStats.record(loopMs);
    nowMs = robot.getClock().nowMs();

    robot.readSensors();          // 1. observe   (clear bulk caches, refresh sensors, tick clock)
    robot.updateLocalization();   //    blend an AprilTag fix into the pose
    onDecide();                   // 2. decide    (schedule commands from gamepad / auto tree)
    Scheduler.execute();          //    driver control and macros both run here
    robot.writeActuators();       // 3. act       (every mechanism's update() writes hardware)

    onAfterAct(); if (logger != null) logger.logRow(robot.logCells(loopMs)); onTelemetry();
}
```

`Robot` has `readSensors()` and `writeActuators()` and **deliberately no `update()`** that does both:
a single method would let a caller put observe-and-act on the same side of the scheduler, so every
command would decide on last loop's data. The order is structural, not a convention.

Lifecycle (all five SDK methods are final in `MatchOpMode`):

| SDK | `MatchOpMode` does | Hook |
|---|---|---|
| `init()` | `Scheduler.reset()` → `buildRobot()` → telemetry interval → `openLogger()` (try/catch) → hook → report missing hardware | `onInit()` |
| `init_loop()` | `robot.readSensors()` → hook | `onInitLoop()` |
| `start()` | `gamepad1/2.resetEdgeDetection()` → `robot.startMatch(period)` → hook | `onStart()` |
| `loop()` | above | `onDecide()`, `onAfterAct()`, `onTelemetry()` |
| `stop()` | hook → `Scheduler.reset()` → `robot.stop()` → `logger.close()` | `onStop()` |

Subclasses implement `logTag()` and `matchPeriod()`. `Robot.stop()` releases non-actuator hardware
only: the SDK rejects motor writes from an iterative OpMode's `stop()`.

## 2. Package tree

`TeamCode/src/main/java/org/firstinspires/ftc/teamcode/`

```
Robot.java                     composition root; readSensors()/writeActuators(); supplier wiring
opmodes/
  MatchOpMode.java             abstract base (extends OpMode); final lifecycle; read → decide → execute → write
  test/
    BenchOpMode.java           abstract base for the pit benches: same Robot, no Ivy; readSensors → onBench → writeActuators
    IntakeBench.java           "Bench: Intake": free speed, stall amps, anti-jam, reject
    StorageBench.java          "Bench: Storage": count, sensors, the SENSORLESS_FEED_PULSE_MS pulse, edge sampling
    TransferBench.java         "Bench: Transfer": lift/feed, the same pulse
    ShooterBench.java          "Bench: Shooter": ticks/rev, rpm, spin-up, shot dip and recovery, PIDF
    ColorSensorBench.java      "Bench: Color sensors": HSV, windows, alliance, presence distance
    LimelightBench.java        "Bench: Limelight": pipelines, tags, blob, mount check
    SelfTest.java              "SelfTest": PASS / FAIL / SKIP per subsystem; run first at every event
  teleop/
    Controls.java              enum of every gamepad binding + generated help card; one Snapshot per loop
    Teleop.java                match TeleOp; driving is an Ivy default command over Pedro's manual mode
  auto/
    MainAuto.java              @Autonomous host: AutoSelector in init, schedules AutoRoutine
    AutoRoutine.java           the auto command tree, JVM-testable
    AutoSelector.java          init-phase alliance / start-position menu
subsystems/
  Drivetrain.java              PathFollower + Clock; manual drive, heading hold (+ aim lock), path commands
  OpenLoopDrive.java           timed open-loop driving through Pedro's Mecanum, for before the follower is tuned
  PathFollower.java            interface: the slice of Pedro 3 Follower the Drivetrain calls
  PedroPathFollower.java       production adapter over com.pedropathing.follower.Follower
  Intake.java                  front 16 mm roller; velocity; JamDetector; captured supplier
  Storage.java                 4-piece channel; Gecko transport; count / full
  Transfer.java                rear 90° vertical Gecko lift into the shooter feed
  Shooter.java                 flywheel velocity + feed gate; atSpeed()
  Limelight.java               Limelight3A: AprilTag pose + piece detection
  ColorSensor.java             NormalizedColorSensor wrapper, one instance per sensor point
  templates/
    Mechanism.java             interface Mechanism<S extends Enum<S>>
    PositionalMotor.java       RUN_TO_POSITION presets, limits, timeouts
    PositionalServo.java       open-loop servo with travel-time wait
    VelocityMotor.java         shared velocity-motor wrapper (new vs the Guide)
commands/
  Macros.java                  multi-subsystem one-button actions with Outcome + timeouts
game/
  PieceType.java               enum POLLEN / NECTAR: colour thresholds, size, height, name
  FieldPoses.java              BLUE-side start / staging / score / park (placeholders)
pedro/
  Constants.java               MecanumConfig, PinpointConfig, ForesightConfig, create(HardwareMap)
  Tuning.java                  @Tuner factories for AutoTune
  procedures/*.java            Quickstart AutoTune procedures (Mecanum, Pinpoint, Foresight, Tests, …)
util/
  control/JamDetector.java
  diagnostics/LoopTimer.java, MatchLogger.java, RateLimiter.java
  field/Alliance.java, FieldConstants.java, PoseFusion.java, PoseStorage.java, StartPosition.java
  hardware/Hardware.java, HardwareNames.java
  math/Angles.java, ColorMath.java, DriveScaling.java, MedianFilter.java, VisionMath.java
  time/Clock.java, MatchClock.java, SystemClock.java
```

`TeamCode/src/test/java/org/firstinspires/ftc/teamcode/` mirrors it: fakes (`util/time/FakeClock`,
`subsystems/FakeDcMotorEx`, `subsystems/FakePathFollower`) and one test class per pure-logic util,
per command-aware subsystem, for `Macros`, `AutoRoutine` and `FieldPoses`.

## 3. Dependency rules

```
opmodes ─────► Robot, subsystems, commands, game, util, Pedro, Ivy, SDK
opmodes.auto / opmodes.teleop ─► opmodes.MatchOpMode (never each other)
opmodes.test ─► opmodes.test.BenchOpMode (never MatchOpMode, never Ivy commands)
Robot ───────► subsystems, commands.Macros, game.PieceType, util.*, Pedro math.Pose, SDK
commands ────► Robot (back-reference), subsystems, util.math, Pedro paths/math, Ivy
subsystems ──► util.*, pedro.Constants (Drivetrain only), Pedro, Ivy, SDK hardware
game ────────► util.field, subsystems.ColorSensor (PieceType.isAtSensor only), Pedro math.Pose
util.field ──► util.math.Angles, Pedro math.Pose, controllers.filters.KalmanFilter
util.math, util.time, util.control, util.hardware ──► java.* (and HardwareMap for Hardware) ONLY
```

Rules: nothing in `util/math`, `util/time`, `util/control` imports hardware or a library; `game/` is
the only place the season is named; subsystems speak of "piece" and "blob"; `pedro/` is the only
place a `Follower` is built; `HardwareNames` is the only place a config string appears.

## 4. Robot composition

- Every subsystem is a `public final` field on `Robot`, plus `macros`, `poseFusion`, and a
  `MatchClock` that is **null before `startMatch()`**.
- Constructors: `Robot(HardwareMap)`, `Robot(HardwareMap, Clock)`, and a composition constructor
  taking pre-built subsystems so tests assemble a robot from fakes.
- Construction: `Hardware.reset()` → every `LynxModule` to `BulkCachingMode.MANUAL` → collect
  voltage sensors once → build subsystems with the injected `Clock` → wire suppliers → `new Macros(this)`.
- `readSensors()`: clear every hub's bulk cache, push `PieceType` target heights into the Limelight,
  `limelight.update()`, each `ColorSensor.update()`, tick `MatchClock`, sample battery every
  `VOLTAGE_SAMPLE_MS` (voltage is not bulk-cached; report the **lowest** sensor).
- `writeActuators()`: `intake`, `storage`, `transfer`, `shooter`, `openLoopDrive`, then
  `drivetrain.update()`.
- Cross-subsystem wiring is **supplier injection in `Robot`'s constructor**, e.g.
  `intake.setCapturedSupplier(() -> storage.entranceSeesPiece())`, `intake.setFullSupplier(storage::isFull)`,
  `shooter.setFeedSupplier(transfer::pieceAtFeed)`. A subsystem never imports another subsystem.
- `updateLocalization()` blends any absolute fix into `poseFusion` every loop and writes the
  returned pose back to the drivetrain **only when `PoseFusion` reports `ACCEPTED`**: an unconditional
  write-back released the heading hold and re-wrote the Pinpoint over I2C every loop (fixthese B1).
  **BIOBUZZ: AprilTags move with the HIVE and cannot localise
  the robot (docs/04 §3), so this runs odometry-only until a static reference exists;**
  `tryLocalizeFromAprilTag()` and the relocalize macro stay wired but are expected to report no fix.
  `abortMacro()` is the shared cancel path (cancel drivetrain path, restore the AprilTag pipeline,
  mark macro cancelled).

Subsystem conventions:

- `isAvailable()` on every subsystem; every call no-ops when its device is missing, so a partially
  wired robot still runs (`util/hardware/Hardware` records the missing name for SelfTest/Teleop).
- `update()` is the only method that writes hardware. Commands set intent; `update()` applies it.
- Sensor subsystems cache one reading per loop inside `update()`; getters are cheap and consistent
  within a tick.
- Every subsystem takes its resolved device(s) and a `Clock` through a terminal constructor
  (`Intake(DcMotorEx, Clock)`) so tests inject fakes; `HardwareMap` constructors delegate to it.
- **Subsystems own their commands.** `*Command()` factories return `com.pedropathing.ivy.Command`
  with `.requiring(this)`; heavy commands are composed from primitives.
- **Arbitration is Ivy's job, not a flag's.** Default commands sit at priority −1 with
  `InterruptedBehavior.SUSPEND` and `BlockedBehavior.QUEUE`; anything requiring the subsystem preempts
  and they resume automatically. Default-command logic lives in `setExecute` (resume skips `start()`).
- Tunables are plain `public static` fields (no Panels `@Configurable`); edit and redeploy.

## 5. OpModes

**`Controls`** — one enum entry per binding: `(Pad, button, description, edge predicate | axis
function)`. `wasPressed(gp1, gp2)`, `axis(gp1, gp2)` (the SDK's stick-up-is-negative sign applied
here only), `helpLines()` renders the init card. The SDK's `*WasPressed()` consumes on read, so
`Controls.read(gp1, gp2)` takes one immutable `Snapshot` per loop and `Teleop` decides from it.
Adding a control is one entry. Driver pad: drive axes, slow mode, drive-frame toggle, reset
heading, abort, aim lock (hold R-trigger), collect / align, Pedro paths to the shooting spot and
park, snap headings. Operator pad: intake / outtake / eject / stop-or-cancel, intake-until-full,
shoot one / all, flywheel arm, TIP counted, debug toggle.

**`Teleop`** — `onInit()` schedules exactly one drive default (`drivetrain.driverControlCommand(fwd,
strafe, turn)` with `DriveScaling.shape(...) * slowScale()` when Pedro is tuned, else
`openLoopDrive.driverControlCommand(...)`: robot-centric, no heading hold, drive macros off) and every
mechanism's `defaultIdleCommand()`, and inherits the pose, alliance and **piece count** from
`PoseStorage`. `onInitLoop()` lets the driver flip the alliance on the dpad (it always wins over what
auto left), retries `tryLocalizeFromAprilTag()`, and shows the help card. The collect/align macros
are refused with the failure rumble until `Limelight.MOUNT_CALIBRATED` is set.
Without a storage-entrance sensor the count is unknowable, so the "Pieces" line shows `?` and the
shoot buttons fire blind (`Macros.piecesOnBoard()`). `onDecide()` reads one `Controls.Snapshot`,
handles driver then operator input, holds or releases the drivetrain aim lock from the right
trigger (`Drivetrain.setAimLock` fed by `Macros.aimHeading` on the current up-CELL from
`game/Field`), then re-schedules the flywheel hold if a macro preempted it. The
sticks abort only a macro that owns the drivetrain; operator X cancels any macro. Haptics rumble
on **transitions** (macro success/failure, a piece in, storage full, final 20 s).
`matchTelemetry()` shows time, pieces, macro state, drive frame, aim and flywheel state, and
faults only when present; `debugTelemetry()` adds the engineering readout.

**`MainAuto`** (`@Autonomous "Auto: shoot 4 + leave"`) — `onInitLoop()` polls `AutoSelector` and
shows the placement card; `onStart()` schedules `new AutoRoutine(robot).build()`; `onDecide()` is
the buzzer safety net (cancel, `abortMacro()`, `stopMechanisms()` once the match clock expires);
writes `PoseStorage` **every loop** (alliance and start; the pose too once a localizer exists).
Once Pedro is tuned it will also seed `FieldConstants.forAlliance(FieldPoses.startPose(start),
alliance)` and fusion.

**`AutoRoutine`** — one Ivy `sequential(...)` ending in `.requiring(robot.intake, robot.storage,
robot.transfer, robot.shooter, robot.openLoopDrive, robot.drivetrain)` so the routine owns its
subsystems for the whole run. **The first-competition version is hardcoded and open-loop** (no
localizer, no paths, no aiming; the robot is placed with the rear-firing shooter toward the CELL):
`storage.setCount(4)` → `Waits.bounded(macros.shootAll(), SHOOT_BUDGET_MS)` → settle →
`openLoopDrive.driveForMsCommand(LEAVE_DIRECTION, LEAVE_MS)` → stop; LEAVE runs whether or not the
shooting succeeded. The Pedro-path version that replaces it once tuned uses the leg pattern:

```java
sequential(instant(() -> currentLeg = name),
           race(drivetrain.followLazyCommand(() -> pathToward(target), holdEnd), waitMs(budgetMs)),
           instant(() -> finishLeg(name, target)));          // atPose check → "ok" | "MISSED"
```

`pathToward` builds `Paths.line(current, target).linear(current.heading(), target.heading())` from
the **current** pose at command start and returns `null` when it cannot. A missed leg skips the
scoring block via `skipIfAnyLegMissed(cmd)` = `conditional(() -> missed == 0, cmd, instant(() -> {}))`
(never `cmd.unless(...)`), then park is attempted unconditionally. Mid-path actions that 2.x did with
`addParametricCallback` are now `race(follow, waitUntil(() -> follower.parametricCompletion() > t))`
followed by the action, or a `parallel` with a `waitUntil`-gated command.

**`AutoSelector`** — dpad cycles alliance / start, A locks the menu against accidental dpad presses,
B unlocks; reads every edge each poll. START runs whatever is shown, locked or not.

**`opmodes/test/`: the benches and `SelfTest`** — iterative OpModes on `BenchOpMode`, group "Bench".
They build the real `Robot` (same config names, directions, velocity code and sensor predicates as the
match OpModes) and drive one subsystem through its intent setters with no Ivy: each loop is
`readSensors()` → the bench's buttons → `writeActuators()`. They exist because the code is otherwise
proven only against fakes, and because every "measure this" constant in HANDOFF section 9 needs a
tool: each bench's card names the constants it feeds and shows the value to paste. `SelfTest` is a
clock-stepped state machine (config names, battery, each velocity mechanism at a low speed with the
sign of the measured velocity checked, colour sensors, Limelight fps, drive layer, localizer settled)
that leaves a PASS / FAIL / SKIP table on the screen; SKIP means not fitted. `BenchOpModesTest` runs
every one of them through `init / init_loop / start / loop` on the fakes. Wheel directions are not a
bench: the SDK's **TestHardware** utility OpMode (11.2+) spins any motor by config name, and the
Mecanum Tuner on a `-Ptuning` build does the same with a web UI.

## 6. Subsystems

| Subsystem | Hardware | States / contract | Commands | Sensors consumed |
|---|---|---|---|---|
| `OpenLoopDrive` | Pedro `revhub.drivetrains.Mecanum` on `Constants.drivetrainConfig` (names + directions, no tuning) | `drive(f, s, t)` / `stop()` set intent; `update()` writes only on change so an idle instance never fights the follower | `driveForMsCommand(f, s, t, ms)` (clock-based), `defaultStopCommand` | none |
| `Drivetrain` | Pedro `Follower` via `PathFollower` | pose, `isFollowingPath()`, heading hold (with an aim-lock setpoint via `setAimLock`, for the fixed shooter), `holdHeading(rad)`, field/robot-centric | `driverControlCommand` (default), `followLazyCommand(supplier, holdEnd)`, `followPathCommand`, `turnToCommand(rad)`, `holdCommand()` | Pinpoint via Pedro |
| `Intake` | 1 `DcMotorEx` velocity (`VelocityMotor`) | `Mode {IDLE, INTAKING, OUTTAKING, EJECTING}`, `hasPiece()`, `isBlockedByFullStorage()` (roller held still at 4, G407), anti-jam only while actively intaking | `intakeCommand`, `outtakeCommand`, `ejectCommand`, `stopCommand`, `runForMs` (injected clock), `captureCommand([supplier])`, `defaultIdleCommand` (−1, SUSPEND, QUEUE) | `capturedSupplier` (entrance sensor), `fullSupplier` from Storage |
| `Storage` | 1–2 `DcMotorEx` velocity (Gecko side wheels, BRAKE) | `count()` 0..4 from rising edges (entrance +1, exit −1), `setCount`/`markEntered`/`markExited`, `isFull()` (count or sensor), `hasPiece()`, `Mode {IDLE, ADVANCING, REVERSING}` | `advanceOneCommand()` (until an exit event or `ADVANCE_TIMEOUT_MS`; skips when empty), `advanceUntilCommand(cond, timeout)`, `advanceCommand`, `reverseCommand`, `stopCommand`, `defaultIdleCommand` | `entranceSupplier`, `fullSupplier`, `exitSupplier` (transfer sensor) |
| `Transfer` | 1 `DcMotorEx` velocity (BRAKE) | `hasPieceInLift()`, `pieceAtFeed()`, `hasFeedSensor()`; `Mode {IDLE, LIFTING, FEEDING, REVERSING}` | `liftOneCommand()` (until staged at the feed; refuses to double-feed), `feedCommand()` (until the feed sensor clears + dwell), both timed pulses without a feed sensor; `liftCommand`, `reverseCommand`, `stopCommand`, `defaultIdleCommand` | `inLiftSupplier`, `atFeedSupplier` (null = no sensor) |
| `Shooter` | 1–2 `DcMotorEx` velocity (FLOAT), RPM via `TICKS_PER_REV` | `setTargetRpm`, `spinUp`, `idle`, `atSpeed()`, `Mode {IDLE, SPINNING_UP, READY}` derived | `spinUpCommand()` (done at speed or timeout, keeps spinning while owned), `holdSpeedCommand()` (never done; idles on interrupt), `idleCommand`, `defaultIdleCommand` | none; feeding is `Transfer.feedCommand()` composed by a macro that holds the shooter |
| `Limelight` | `Limelight3A` | see §16 | none (data source) | camera |
| `ColorSensor` | `NormalizedColorSensor` per point | see §17 | none (data source) | sensor |

Piece-flow interlocks are expressed as suppliers wired in `Robot`, and as `waitUntil` conditions in
`Macros`; no subsystem polls another.

## 7. Templates

- `Mechanism<S extends Enum<S>>`: `Command goTo(S)`, `S getState()` (last commanded), `boolean
  atState()` (actually arrived), `void update()`.
- `PositionalMotor<S>`: `preset(state, ticks)`, `limits(min, max)`, `RUN_TO_POSITION` with
  `setTargetPosition(0)` before entering the mode, `MIN_MOVE_MS` guard against instant false
  completion, `MOVE_TIMEOUT_MS` against jams, power cut in `setEnd`. No power at construction.
- `PositionalServo<S>`: `preset(state, position)`; `goTo` = `sequential(instant(set), waitMs(travel))`,
  travel scaled by distance and measured at build time; `atState()` is always true (no feedback).
  Prefer `NaN` initial position so a servo is not slammed during init.
- `VelocityMotor` (new): `RUN_USING_ENCODER`, `setVelocity(ticksPerSec)`, `atSpeed(tolerance)`,
  `getCurrentAmps()`, optional `JamDetector` hook, terminal constructor `(DcMotorEx, Clock)`. Used by
  Intake, Storage, Transfer, Shooter so the spec-vs-measured ticks/rev question is answered once.
  **Ticks per rev must be measured**, not read off a spec sheet: the Guide's Intake documents that
  pairing the 312 RPM part's 537.7 ticks/rev with a 435 RPM motor overstates the ceiling by ~40%.

## 8. Drivetrain on Pedro 3.0.0: the `PathFollower` re-spec

`PathFollower` is the seam that keeps everything else independent of Pedro. Its 3.0.0 shape:

```java
public interface PathFollower {
    void update();                       // once per loop
    Pose pose();  void setPose(Pose p);
    void manual(double forward, double strafe, double turn);   // robot-frame powers
    void follow(Path path);              // restarts from t = 0
    boolean atParametricEnd();           // path geometry finished
    boolean isBusy();                    // settled-in-hold flag, NOT "following"
    void hold(Pose pose, boolean scaled);
    void stop();
    Follower.Mode mode();                // FOLLOW / HOLD / MANUAL / IDLE
}
// PedroPathFollower adds raw(); Drivetrain.getFollower() returns it, null in tests.
```

Guide methods with no 3.0.0 equivalent, and what replaces them:

| Guide (2.x) | Here |
|---|---|
| `startTeleopDrive()` + `teleopEngaged` flag | none — `manual(...)` sets the mode; the double-tick bug cannot recur |
| `setTeleOpDrive(f, s, t, robotCentric)` | `manual(ManualDrive.fieldCentric(f, s, t, pose().heading()))` or `manual(f, s, t)` |
| `followPath(chain, holdEnd)` | `follow(path)`; hold-at-end is our `setEnd` decision, not Pedro's `holdEnd` |
| `isBusy()` for "still following" | `!atParametricEnd()` |
| `turnTo(rad)` | `hold(pose().withHeading(rad), false)` + done when within `TURN_TOLERANCE_RAD` (and `MIN_PATH_MS` elapsed) or after `TURN_TIMEOUT_MS` |
| `atPose(pose, xTol, yTol)` | local: `Math.abs(dx) <= xTol && Math.abs(dy) <= yTol` on `pose()` |
| `setMaxPower(p)` | `path.with(Constants.foresightConfig.maxPathSpeed.at(p))` inside the path supplier |
| `cancelPath()` = `startTeleop()` | `manual(0, 0, 0)` — hands the follower back to the default command |
| `setStartingPose` | `setPose` |
| heading hold via `PIDFController.updateError/run` | `Controller.pid(P, I, D)` fed `calculate(0, angleError)`; or `ManualDrive.headingLock(follower, controller, powers, held)` |
| `Pose.getX()/getY()/getHeading()` | `x()/y()/heading()` |

`Drivetrain.followLazyCommand(Supplier<Path>, boolean holdEnd)` keeps its Guide semantics: path
built at `start()` from the current pose, `MIN_PATH_MS` guard, null-supplier finishes at once, a
path without a heading is recorded via `Hardware.recordFailure` and skipped rather than thrown,
natural end with `holdEnd` explicitly holds at `path.endPose()`, any other end calls
`manual(0,0,0)`, `.requiring(this)`. Do not replace it with a bare `PedroCommands.follow` (no requirement, no cleanup).

`FakePathFollower` implements this interface and mirrors Pedro's real state machine (`atParametricEnd`
true when not following, `isBusy` only cleared in hold, `follow` restarts), so
`DrivetrainCommandTest` pins the behaviours that a Pedro upgrade could silently change.

## 9. `commands/Macros`

Multi-subsystem, one-button, bounded, reporting. Each macro: `begin(name)` → work → `finish(success,
failure, check)`; every wait is `race(work, waitMs(TIMEOUT))` with `commands/Waits.waitMs` on the injected `Clock` (Ivy's own `waitMs` is wall-clock); `Outcome {IDLE, RUNNING, SUCCESS,
TIMED_OUT, NO_TARGET, CANCELLED}`; vision paths are built inside `followLazyCommand` suppliers, never
stored; requirements come from the subsystem commands composed. Planned set:

| Macro | Composition (sketch) | Success |
|---|---|---|
| `intakeUntilFull()` | `intakeCommand` raced with `waitUntil(storage::isFull)` and timeout | count increased |
| `shootOne()` | `deadline(sequential(shooter.spinUpCommand(), storage.advanceOneCommand(), transfer.liftOneCommand(), transfer.feedCommand()), shooter.holdSpeedCommand())`; never requires the drivetrain, so driving and the aim lock continue through a shot | a piece left the storage and the feed cleared (sensors), or the pulse cycle completed (sensorless; count dead-reckoned) |
| `shootAll()` | `repeat(shootOne, storage::count)` with an overall timeout | storage empty |
| `alignToPiece()` / `collectPiece()` | blob pipeline → warm-up → `waitUntil(hasStableBlob)` → `followLazyCommand(approach)` with intake capture | new piece captured |
| `relocalize()` | AprilTag pipeline → `waitUntil(botpose != null)` → `tryLocalizeFromAprilTag` holding the drivetrain | fix applied |
| `snapToHeading(rad)` | `race(drivetrain.turnToCommand(rad), waitMs)` | within tolerance |
| `aimAt(Pose, minTag, maxTag)` / `aimHeading(...)` | the drivetrain turns (Pedro hold, re-issued as a visible tag refines the bearing) until the shooter's firing side faces the CELL: heading = bearing − `Shooter.HEADING_OFFSET_RAD`; the OpMode passes the CELL and tag range from `game/Field` | within `AIM_TOLERANCE_DEGREES` |

Success is measured after the fact against a snapshot taken at the start (a perfect drive that
collected nothing is a failure).

## 10. `game/`

Everything that changes with the season and nothing else. `PieceType` (POLLEN, NECTAR): hue,
tolerance, saturation/value floors, diameter, camera target height, driver-facing name;
`isAtSensor(ColorSensor)`. `FieldPoses`: BLUE poses using Pedro
`new Pose(x, y, Math.toRadians(h))`, mirrored by `FieldConstants.forAlliance`; placeholders until
the field is measured. `Robot` is the only production class that reads `PieceType`.

## 11. `pedro/`

`Constants.java`: `public static MecanumConfig drivetrainConfig`, `PinpointConfig localizerConfig`,
`ForesightConfig foresightConfig`, `Follower create(HardwareMap)` — see doc 01 §A.1 for the worked
version (`git show 20768b3:.../pedro/Constants.java`). Motor and Pinpoint names come from
`HardwareNames`. `Tuning.java`: `@Tuner` static factories for `MecanumTuner`, `PinpointTuner`,
`ForesightTuner`, `Tests` (doc 01 §A.7; note the two factories' differing argument orders).
`procedures/`: the Quickstart's AutoTune procedures, untouched. As of 2026-09-13 only
`drivetrainConfig` is filled (names from `HardwareNames`, left REVERSE / right FORWARD; verify with
the Mecanum Tuner); `OpenLoopDrive` drives through it before tuning. Until the localizer and
Foresight configs exist and `create()` returns a follower, Pedro cannot follow a path; `Drivetrain`
fails soft, `Robot` builds `OpenLoopDrive` on the real motors instead (never both), the sticks drive
robot-centric through it, and the drive macros, snap turns and aim lock are off. Intake, storage,
transfer, shooter and the hardcoded auto run as normal.

## 12. `util/`

| Class | Purpose | Notes |
|---|---|---|
| `control/JamDetector` | current-based stall detection with bounded un-jamming; time is an argument | explicit `stalling`/`unjamming` flags, never a 0 sentinel; recovery resets attempts |
| `diagnostics/LoopTimer` | p95 / max / spikes in a fixed histogram | `getStatus()` one-liner for telemetry |
| `diagnostics/MatchLogger` | CSV per loop under `/sdcard/FIRST/data/` | knows nothing about the robot: header in the constructor, cells in `logRow(Object...)`; `Robot` supplies `BIOBUZZ_COLUMNS` (27 columns incl. storage count, transfer state, heading hold / aim lock, shooter target vs actual); `NaN` for follower cells without a follower; flush every 50 rows; catches `Exception`; `MatchLogger(File, tag, header)` for JVM tests |
| `diagnostics/RateLimiter` | at most every N ms; `ready(now)` claims the slot | telemetry / expensive reads |
| `field/FieldConstants` | `FIELD_SIZE_INCHES = 144`, `Symmetry {MIRROR_X, MIRROR_Y, ROTATE_180}` + `SYMMETRY`, `forAlliance`, `mirrorAcrossX/Y`, `rotate180` (heading transformed too), `isInsideField` | the one source of field size, used by `PoseFusion` and `Limelight`; `SYMMETRY = ROTATE_180` per `docs/04` §2.5 |
| `field/PoseFusion` | X/Y Kalman blend of odometry + AprilTag; heading passes through | `controllers.filters.KalmanFilter(model, data)`; gates: field bounds, `MAX_JUMP_INCHES`; latency back-dating; **caller writes the pose back** |
| `field/PoseStorage`, `Alliance`, `StartPosition` | auto → teleop handoff; enums (`StartPosition` carries a driver-facing `label()`) | survives OpMode switch, not RC restart |
| `hardware/Hardware`, `HardwareNames` | fail-soft lookup; every config name | `Robot` calls `Hardware.reset()` first |
| `math/Angles` | `normalizeAngle`, `angleError`, `headingToward` | feed controllers an error, never a raw angle |
| `math/ColorMath` | `toHsv` on floats, `hueDistance`, `matches` | §17 |
| `math/DriveScaling` | deadband, expo, `shape`, `slowScale` | |
| `math/MedianFilter` | rolling median, `spread()`, `isReady()` | a median discards a bad frame |
| `math/VisionMath` | ray / ground-plane intersection, `Mount` model | pitch positive **downward** |
| `time/Clock`, `SystemClock`, `MatchClock` | injectable monotonic time; BIOBUZZ periods (30 s AUTO, 8 s transition, 120 s TELEOP), `isFlowerUnlocked()` for the 1:00 NECTAR-into-FLOWER window, `isFinalSeconds()` at 0:20 | `hasTimeFor()` true before the clock starts; `ENDGAME` phase = the FLOWER window |

Also planned in `util` once needed: a `Clock`-based `waitMs` command so command trees are
deterministic under test (doc 01 §B.5).

## 13. Testing

`./gradlew :TeamCode:test` runs on a laptop with no robot. Two layers:

- **Pure logic** (`util/math`, `util/time`, `util/control`, `util/diagnostics`, `util/field`): the
  reason those classes exist separately from the subsystems that use them. When you add math to a
  subsystem, extract it into `util/` so it can be covered.
- **Commands and subsystems** through Ivy's real `Scheduler` against fakes: `FakeClock.advance(ms)`,
  `FakeDcMotorEx` (records mode, velocity, writes), `FakePathFollower`. Every test does
  `Scheduler.reset()` in `@Before` and `@After` (the scheduler is static). The tick helper mirrors the
  loop order: `Scheduler.execute(); subsystem.update(); clock.advance(20);`.
- Ivy's `waitMs` reads the wall clock: tests that need it shorten the `public static` timeouts and
  sleep a few ms per tick under a wall-clock watchdog, or use the `Clock`-based wait.
- Test the failure case first: run a new macro with no target and confirm it times out and says so.

Build config: `TeamCode/build.gradle` has `testOptions { unitTests.returnDefaultValues = true }` and
`testImplementation 'junit:junit:4.13.2'`. `com.pedropathing.ivy:core` (zero-dependency JAR) and
Pedro core reach the test classpath transitively.

## 14. FTC_Guide → this repo

| Guide file | Here | Change |
|---|---|---|
| `util/math/*`, `util/time/*`, `util/control/JamDetector`, `util/hardware/*`, `diagnostics/LoopTimer`, `RateLimiter`, `opmodes/Controls`, `AutoSelector`, `subsystems/ColorSensor`, tests' `FakeClock`, `FakeDcMotorEx` | same | verbatim, minus `@Configurable` |
| `subsystems/Limelight`, `util/field/{FieldConstants, PoseStorage}`, `game/FieldPoses`, `Robot`, `MainAuto`, `SelfTest` | same | `Pose` import → `com.pedropathing.math.Pose`, accessors `x()/y()/heading()` |
| `util/field/PoseFusion` | same | `controllers.filters.KalmanFilter(model, data)`, `state()` |
| `util/diagnostics/MatchLogger` | same | follower columns from `FollowerLog` / `Foresight` |
| `subsystems/PathFollower`, `PedroPathFollower`, test `FakePathFollower` | same | re-specified (§8) |
| `subsystems/Drivetrain` | same | `manual`/`follow`/`hold`; heading hold on `Controller.pid`; `turnTo` via hold |
| `commands/Macros`, `opmodes/AutoRoutine` | same | `Paths.*` instead of `PathBuilder`/`BezierLine`; no callbacks; `Controller` instead of `PIDFController` |
| `subsystems/Intake` | same + `VelocityMotor` | unchanged logic |
| `subsystems/templates/ExampleLift` | removed | teaching example; V1 has no lift-type mechanism |
| `game/Pollen` | `game/PieceType` | two piece types |
| — | `Storage`, `Transfer`, `Shooter`, `VelocityMotor` | new for BIOBUZZ V1 (no turret: the shooter is fixed and the drivetrain aims) |
| `util/diagnostics/Drawing`, `onDraw()` hooks, `Drawing.init()` | removed | no Panels; `MatchLogger` is the diagnostics channel |
| `pedro/Tuning.java` (1644-line 2.x menu), `SelectableOpMode`, `com.pedropathing:telemetry` | AutoTune `@Tuner` factories | `com.pedropathing:tuning:1.0.0` |
| `pedro/Constants.java` (default `FollowerConstants`) | 3.0.0 configs | doc 01 §A.1 |
| `com.pedropathing:ivy:1.0.0` | `com.pedropathing.ivy:pedro:1.1.1` | imports unchanged |

## 15. Sensor data flow

```
Robot.readSensors()
  limelight.update()      → one LLResult cached; largest blob picked; tx/ty into MedianFilters
  colorSensor*.update()   → one NormalizedRGBA each → ColorMath.toHsv → hsv[3]
Robot suppliers
  intake.captured  ← storageEntrance.matchesHue(PieceType...)   (or a distance/beam-break read)
  intake.full      ← storage.isFull()  ← storage-full sensor
  transfer.atFeed  ← shooterFeed sensor
Scheduler.execute()        commands read the cached values
Robot.writeActuators()     mechanisms apply intent
```

Two distinct "have we got one?" questions: `limelight.hasStableBlob()` (the camera sees a piece on
the field) versus `storage.hasPiece()` / `intake.hasPiece()` (a piece is in the robot, sensor-confirmed).

## 16. Limelight 3A

**BIOBUZZ scope.** Every AprilTag is a cluster on the underside of a moving HIVE CELL (IDs 30–45,
3.25 in, cluster origin at the centre of the CELL opening). The SDK states they are unsuitable for
field localisation, so `getBotposeAsPedroPose()` is expected to return null all season; the AprilTag
pipeline is used for **aiming** (tx/ty/range to the CELL the shooter must hit: red starts on tags
34–37, blue on 42–45, flipping after each TIP), and the colour pipeline for POLLEN (yellow) and
NECTAR (red/blue). See docs/04 §3. Note also R704: no dashboard/streaming tools during matches.

Hardware class `com.qualcomm.hardware.limelightvision.Limelight3A` (SDK 11.2.1; identical to 11.1.0).
Configured in the Robot Controller as an Ethernet device of type Limelight3A, name
`HardwareNames.LIMELIGHT` (`"limelight"`). The Limelight runs its own pipelines; the SDK polls JSON
results over the USB-Ethernet link.

**`Limelight3A`**

| Method | Use |
|---|---|
| `start()` / `pause()` / `stop()` / `isRunning()` | start polling in `init()`, stop in `Robot.stop()` |
| `setPollRateHz(int)` | e.g. 100 before `start()` |
| `getLatestResult()` → `LLResult` | once per loop, cached by `Limelight.update()` |
| `getStatus()` → `LLStatus` | `getFps()`, `getName()`, `getPipelineIndex()`, `getPipelineType()`, `getCpu()`, `getRam()`, `getTemp()`, `getCameraQuat()` — SelfTest passes on `fps > 0` |
| `pipelineSwitch(int)` → boolean | only cache the new index on `true`; clear stale caches |
| `reloadPipeline()`, `uploadPipeline(json, idx)`, `uploadFieldmap(LLFieldMap, idx)`, `uploadPython(...)` | setup tooling |
| `updateRobotOrientation(double yawDegrees)` | required every loop for MegaTag2 |
| `updatePythonInputs(...)`, `captureSnapshot(name)`, `deleteSnapshot(s)` | optional |
| `getTimeSinceLastUpdate()`, `isConnected()` | health |

**`LLResult`**

| Method | Meaning |
|---|---|
| `isValid()` | a parsed result exists; **not** "a target is seen" |
| `getStaleness()` (ms) | age of the result; the wrapper rejects `> MAX_STALENESS_MS` (250) |
| `getPipelineIndex()`, `getPipelineType()` | what produced it |
| `getTx()`, `getTy()` (deg), `getTxNC()/getTyNC()` (no crosshair offset), `getTa()` (**0–100 %** of the image) | primary target |
| `getBotpose()` → `Pose3D` | MegaTag1 field pose in **metres**, origin at **field centre**; built from a 6-double array that is **all zeros, never null/NaN, when no tags are seen** |
| `getBotposeTagCount()` | **the gate**: require `>= 1` before trusting `getBotpose()` |
| `getBotpose_MT2()`, `getStddevMt1()`, `getStddevMt2()`, `getBotposeSpan()`, `getBotposeAvgDist()`, `getBotposeAvgArea()` | MegaTag2 and quality metrics |
| `getCaptureLatency()`, `getTargetingLatency()`, `getParseLatency()` (ms) | vision latency = capture + targeting (parse is Control-Hub side) |
| `getControlHubTimeStamp()`, `getTimestamp()` | timing |
| `getFiducialResults()` → `List<FiducialResult>` | `getFiducialId()`, `getFamily()`, `getTargetXDegrees()/YDegrees()`, `getTargetArea()`, `getRobotPoseFieldSpace()`, `getRobotPoseTargetSpace()`, `getCameraPoseTargetSpace()`, `getTargetPoseCameraSpace()`, `getTargetPoseRobotSpace()`, `getSkew()`, `getTargetCorners()` |
| `getColorResults()` → `List<ColorResult>` | `getTargetArea()`, `getTargetXDegrees()/YDegrees()`, `getTargetXPixels()/YPixels()`, `getTargetCorners()`, same pose getters |
| `getDetectorResults()`, `getClassifierResults()`, `getBarcodeResults()`, `getPythonOutput()` | other pipeline types |

`Pose3D`: `getPosition()` → `Position` (`x, y, z, unit`; `.toUnit(DistanceUnit.INCH)`),
`getOrientation()` → `YawPitchRollAngles` (`getYaw(AngleUnit.RADIANS)`).

**Wrapper contract (`subsystems/Limelight`)**, carried from the Guide:

```java
APRILTAG_PIPELINE_INDEX = 0;  BLOB_PIPELINE_INDEX = 1;
CAMERA_HEIGHT_INCHES, CAMERA_PITCH_DEGREES (positive = tilted toward the floor),
CAMERA_FORWARD_OFFSET_INCHES, CAMERA_LEFT_OFFSET_INCHES, CAMERA_YAW_OFFSET_DEGREES   // rigid mount → constants, measure after CAD
PICKUP_STANDOFF_INCHES = 8;  MAX_VALID_DISTANCE_INCHES = 120;  MAX_STALENESS_MS = 250;
DETECTION_WINDOW = 5;  MAX_DETECTION_SPREAD_DEGREES = 6;  BOTPOSE_HEADING_OFFSET_RAD = 0;
```

- `update()`: `getLatestResult()`; on invalid/empty clear the blob caches and reset both filters;
  else pick the **largest** colour blob in one pass and push its tx/ty into the `MedianFilter`s.
- `hasTarget()` = valid and not stale. `seesBlob()` = a blob exists. `hasStableBlob()` = full window
  **and** spread ≤ 6° on both axes. **Gate motion on `hasStableBlob()`.**
- `getBotposeAsPedroPose()` gates, in order: `hasTarget()`, pipeline is AprilTag, `getBotposeTagCount() >= 1`,
  then converts and checks the field:
  ```java
  Position p = bp.getPosition().toUnit(DistanceUnit.INCH);
  double x = p.x + FieldConstants.FIELD_CENTER_INCHES;      // centre origin → corner origin
  double y = p.y + FieldConstants.FIELD_CENTER_INCHES;
  if (!FieldConstants.isInsideField(x, y)) return null;
  double h = bp.getOrientation().getYaw(AngleUnit.RADIANS) + BOTPOSE_HEADING_OFFSET_RAD;
  return new Pose(x, y, h);
  ```
  The axis alignment between the Limelight field map and Pedro's frame must be verified on the real
  field; `BOTPOSE_HEADING_OFFSET_RAD` (and if needed an axis swap) is where the correction lives.
- `getVisionLatencyMs()` = capture + targeting latency, fed to `PoseFusion.update(...)`.
- `estimateBlobDistanceInches()` / `estimateBlobInRobotFrame()` / `estimateBlobApproachPose(pose)`:
  `VisionMath` ray-to-floor intersection using the `Mount` and the piece's target height pushed in by
  `Robot` from `PieceType`; the approach pose faces the piece but stops `PICKUP_STANDOFF_INCHES` short,
  and returns `null` (never the robot's own pose) when it cannot.
- Sign warning: our `CAMERA_PITCH_DEGREES` is positive **downward**; the Limelight docs' mount angle
  is positive upward.
- `switchPipeline()` caches only on success and resets the filters; `getPipelineName()` for telemetry.
- MegaTag2 (`updateRobotOrientation(yawDeg)` each loop + `getBotpose_MT2()`, with `getStddevMt2()`
  as a fusion weight) is available and unused; the yaw must be in the Limelight field frame in
  degrees, not Pedro radians.

## 17. Colour sensor

SDK interfaces (identical in 11.1.0 and 11.2.1):

```java
interface NormalizedColorSensor extends HardwareDevice { NormalizedRGBA getNormalizedColors(); float getGain(); void setGain(float); }
class NormalizedRGBA { public float red, green, blue, alpha; /* [0,1) */ @ColorInt int toColor(); }
interface SwitchableLight extends Light { void enableLight(boolean); boolean isLightOn(); }
interface DistanceSensor { double getDistance(DistanceUnit unit); }
```

| Sensor | RC config name | `SwitchableLight` | `DistanceSensor` | Gain |
|---|---|---|---|---|
| REV Color Sensor V3 (`RevColorSensorV3`) | "REV Color Sensor V3" | **no** (LED always on; `enableLed` is a no-op) | yes | yes |
| REV V1/V2 | | no (physical switch) | via range variants | yes |
| AndyMark Proximity & Color | "AndyMark Proximity & Color Sensor" | no | yes | **no** |
| Modern Robotics | | yes | no | |

Consequence: with a REV V3, `ColorSensor.hasLight()` is false and `setLight(...)` is a no-op;
SelfTest reports "(no switchable light)" rather than hiding it.

**Wrapper contract (`subsystems/ColorSensor`)**: constructor `(HardwareMap, name, gain, lightOn)`
with `DEFAULT_GAIN = 2f` (practical 1.0–4.0; raise in dim light, lower if channels pin near 1);
`update()` caches one `NormalizedRGBA` and converts with `ColorMath.toHsv(r, g, b, hsv)`;
`matchesHue(hueDeg, tolerance, minSat, minVal)`; `getHue/getSaturation/getValue`, `getHsv()`
(a copy), `getRed/Green/Blue/Alpha`, `setGain/getGain`, `hasLight/setLight/isLightOn`,
`hasDistance/getDistance(unit)` (`NaN` when not a `DistanceSensor`), `isAvailable()`. Instantiable
per `HardwareNames` sensor point.

**Why HSV is computed on the floats.** `NormalizedRGBA.toColor()` scales by 256 and clips each
channel to 0–255, so `Color.colorToHSV(rgba.toColor(), hsv)` quantises to 1/255 steps and pins
value at 1.0 for any channel above ~0.996 — exactly when a piece is closest and gain is above 1.
`ColorMath.toHsv` works on the unclipped floats: hue and saturation are channel ratios and stay
exact; value is clamped at 1 but never used for classification.

**Classification rule**: classify on hue, gate on saturation and value. Without the saturation floor
a white or grey surface passes any hue test; without the value floor, shadow noise produces random
hues. `hueDistance` is circular (350° and 10° are 20° apart), so red needs one window, not two.
`PieceType` carries the thresholds for POLLEN and NECTAR; measure them on the real pieces under
match lighting.

`ConceptVisionColorSensor` in the SDK samples is **not** a colour-sensor sample: it is an OpenCV
`PredominantColorProcessor` on a webcam with OpenCV HSV ranges (H 0–180, S/V 0–255) and discrete
swatches. `SensorColor.java` is the right reference for this wrapper.

## 18. Rules carried from the Guide's lessons

- Observe, decide, act, in that order; the order is structural (`MatchOpMode.loop()` is final).
- One lifecycle filled in through hooks; one place per idea (`Controls`, `HardwareNames`,
  `FieldConstants`, `game/`, `MatchOpMode`).
- A missing device degrades one subsystem, never the OpMode (`Hardware`, `isAvailable()`).
- Every wait has a timeout; every macro reports an outcome measured after the fact against a snapshot.
- Pure logic goes in `util/` so it can be tested; a sentinel must not overlap the valid range.
- One injected, monotonic `Clock`.
- Commands set intent; `update()` writes hardware; sample after the write.
- Any pose write releases the heading hold; relocalize holds the drivetrain.
- Go through a command that requires the subsystem, or make the group own it explicitly.
- State flags are set where the state changes; commands change no state at build time.
- Return `null`/`NaN` for "no reading", never 0 or the robot's own pose.
- A median, not an average, for vision; reject bad data before filtering.
- Sanity-check every gain by multiplying it by a typical error; clamp, tolerance, then a timeout.
- The one thing Ivy cannot do is stop the Pedro follower: every path command hands back in `setEnd`.
- A pattern nothing uses is a pattern nobody trusts: wire `Shooter` into `Robot` and
  `Teleop` before the hardware exists; `isAvailable()` keeps them inert.
- Code comments state the rule; a lessons-learned doc keeps the story. Documentation is tracked.

## 20. Practical notes from the 2026-09-14 review

Things that are correct in the code but will surprise someone on the robot:

- **Camera facing decides whether tag-aiming exists.** The shooter fires out the rear. A front camera
  only sees the HIVE tags while the robot is *not* aimed, so `Macros.aimHeading` silently falls back
  to odometry and the tag branch never runs; only piece detection benefits. A rear camera makes
  tag-refined aiming real: set `Limelight.CAMERA_YAW_OFFSET_DEGREES = 180` and the same maths works
  (blob approach paths then face the piece by turning around first). Undecided at the time of writing.
- **`followLazyCommand` reports "arrived" the moment anything else changes the follower's mode**
  (`atParametricEnd()` is true whenever the follower is not in FOLLOW). A `hold` or `manual` issued
  by another command while a path runs ends the path command after `MIN_PATH_MS`, holding at an end
  pose it never reached. Keep paths and turns sequential, never parallel, in the Pedro auto.
- **The sensorless pulse is `Macros.SENSORLESS_FEED_PULSE_MS`.** `Transfer.LIFT_PULSE_MS` and
  `FEED_PULSE_MS` only apply to a direct `liftOneCommand()` / `feedCommand()`, which the shooting
  macros no longer use without sensors. Tune the Macros one (the storage and transfer benches show
  it).
- **`Macros.INTAKE_RUNS_STORAGE = true` is an unprotected stall.** `Storage` has no jam detection;
  with the transfer stopped, the side wheels would push the queue against it at full velocity
  authority for up to `INTAKE_TIMEOUT_MS`. Leave it off unless the bench shows the channel will not
  accept a piece otherwise, and add a storage `JamDetector` first if so.
- **Shot counts and `SUCCESS` are pulse counts without sensors.** `pieceWasShot` degrades to `true`
  for every sensor that is absent. The auto log's `shots 4 : SUCCESS` means four pulses ran.
- **The storage count has three states.** Known from a sensor edge or `setCount`; unknown (no
  entrance sensor, nothing on record: shoot buttons fire blind, the interlock cannot fire, the card
  shows `?`); handed over from auto (`PoseStorage`), which is the dead-reckoned remainder.
- **Four fitted colour sensors are four to eight I2C transactions per loop** (colour, plus distance
  where the device has it), none of them bulk-cached. Watch `Loop` in the debug telemetry; if p95
  climbs, drop a sensor from the presence set before touching anything else.
- **Entrance edge counting samples at loop rate.** A piece that crosses the entrance sensor in under
  two loops is missed. `Bench: Storage` shows the longest in-view run; if it is one, slow the
  intake hand-off or move the sensor.
- **Wiring checks need no code**: the SDK Utility menu's TestHardware spins any motor and shows any
  sensor by config name. Use it before the benches at every event.

## 19. Build and run

```bash
JAVA_HOME="/Applications/Android Studio.app/Contents/jbr/Contents/Home" \
  ./gradlew :TeamCode:compileDebugJavaWithJavac --console=plain    # compile
./gradlew :TeamCode:test                                            # JVM unit tests, no robot
./gradlew :TeamCode:assembleDebug                                   # APK (or the green Run button)
adb pull /sdcard/FIRST/data ./logs                                  # match logs
```

Gradle 9.1.0 runs on JDK 17–25; if the system JDK is rejected, use the Android Studio JBR as above.
AutoTune web UI: `http://192.168.43.1:10158` while connected to the robot's Wi-Fi. **Remove the
`tuning` dependency from competition builds** (R704 prohibits third-party streaming/logging services
during matches, and AutoTune's server is always bound). The repo is on FTC SDK 11.2.1 from the Pedro
Quickstart; the BIOBUZZ SDK is **v12.0**, required for the AprilTag cluster API (docs/04 §6).

Driver-station OpModes once implemented: `Teleop` (Main), `Auto` (Main), `SelfTest` (Diagnostics —
run first at every event), `Concept: Commands` (Concept). Tuning has no OpMode; it is the web UI.
