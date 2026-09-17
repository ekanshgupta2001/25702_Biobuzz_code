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

    robot.readSensors();          // 1. observe   (clear bulk caches, refresh sensors, tick clock)
    onDecide();                   // 2. decide    (schedule commands from gamepad / auto tree)
    Scheduler.execute();          //    driver control and macros both run here
    robot.writeActuators();       // 3. act       (every mechanism's update() writes hardware)

    onAfterAct();
    if (logger != null && ++loopCount % LOG_EVERY_N_LOOPS == 0) logger.logRow(robot.logCells(loopMs));
    onTelemetry();
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
  RobotTunables.java           the classes whose public statics are tunables; snapshot() first in every init
  test/
    BenchOpMode.java           abstract base for the pit benches: same Robot, no Ivy; readSensors → onBench → writeActuators
    IntakeBench.java           "Bench: Intake": free speed, stall amps, anti-jam, reject
    StorageBench.java          "Bench: Storage": count, sensors, the SENSORLESS_FEED_PULSE_MS pulse, edge sampling
    TransferBench.java         "Bench: Transfer": lift/feed, the same pulse
    ShooterBench.java          "Bench: Shooter": ticks/rev, rpm, spin-up, shot dip and recovery, PIDF
    ColorSensorBench.java      "Bench: Color sensors": HSV, windows, alliance, presence distance
    LimelightBench.java        "Bench: Limelight": fps, frame freshness, every tag in view with its tx
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
  Intake.java                  front 16 mm roller; velocity; JamDetector; current sampled only while pulling
  Storage.java                 4-piece channel; Gecko transport; count / full
  Transfer.java                rear 90° vertical Gecko lift into the shooter feed
  Shooter.java                 flywheel velocity; atSpeed() held for AT_SPEED_HOLD_MS; a hold leaves its target
  Limelight.java               Limelight3A: the target CELL's tag tx, for aiming (nothing else)
  ColorSensor.java             NormalizedColorSensor wrapper, one instance per sensor point
  templates/
    VelocityMotor.java         shared velocity-motor wrapper: intent then write, writes on change only
commands/
  Macros.java                  multi-subsystem one-button actions with Outcome + timeouts; the aim law
  ShootCycle.java              the shooting cycle as one state-machine command (no Ivy hand-off loops)
  Waits.java                   Clock-based waitMs / bounded
game/
  Field.java                   the field frame (origin A1), tiles, HIVE CELLs, tag ranges, LOADING ZONE, pre-loads
  PieceType.java               enum POLLEN / NECTAR: hue windows, HUES_CALIBRATED, nectarAllianceAt
  FieldPoses.java              BLUE-side starts / shooting spot / park (placeholders)
pedro/
  Constants.java               MecanumConfig, PinpointConfig, ForesightConfig, create(HardwareMap)
  Tuning.java                  @Tuner factories for AutoTune
  procedures/*.java            the four Quickstart AutoTune procedures the robot uses (Mecanum, Pinpoint, Foresight, Tests)
util/
  control/JamDetector.java
  diagnostics/LoopTimer.java, MatchLogger.java, Tunables.java, BuildFlavor.java
  field/Alliance.java, FieldConstants.java, PoseStorage.java, StartPosition.java
  hardware/Hardware.java, HardwareNames.java
  math/Angles.java, ColorMath.java, DriveScaling.java
  time/Clock.java, MatchClock.java, SystemClock.java
```

`TeamCode/src/test/java/org/firstinspires/ftc/teamcode/` mirrors it: fakes (`util/time/FakeClock`,
`subsystems/FakeDcMotorEx`, `subsystems/FakePathFollower`) and one test class per pure-logic util,
per command-aware subsystem, for `Macros`, `AutoRoutine` and `FieldPoses`.

## 3. Dependency rules

```
opmodes ─────► Robot, subsystems, commands, game, util, Pedro, Ivy, SDK
opmodes.auto / opmodes.teleop ─► opmodes.MatchOpMode (never each other)
opmodes.test ─► opmodes.test.BenchOpMode, opmodes.RobotTunables (never MatchOpMode, never Ivy commands)
util.diagnostics.Tunables ─► java.lang.reflect only (the class list lives in opmodes.RobotTunables, so util never imports a subsystem)
Robot ───────► subsystems, commands.Macros, game.PieceType, util.*, Pedro math.Pose, SDK
commands ────► Robot (back-reference), subsystems, util.math, Pedro paths/math, Ivy
subsystems ──► util.*, pedro.Constants (Drivetrain only), Pedro, Ivy, SDK hardware
game ────────► util.field, subsystems.ColorSensor (PieceType.isAtSensor only), Pedro math.Pose
util.field ──► util.math.Angles, Pedro math.Pose
util.math, util.time, util.control, util.hardware ──► java.* (and HardwareMap for Hardware) ONLY
```

Rules: nothing in `util/math`, `util/time`, `util/control` imports hardware or a library; `game/` is
the only place the season is named; subsystems speak of "piece"; `pedro/` is the only
place a `Follower` is built; `HardwareNames` is the only place a config string appears.

## 4. Robot composition

- Every subsystem is a `public final` field on `Robot`, plus `macros` and a `MatchClock` that is
  **null before `startMatch()`**.
- Constructors: `Robot(HardwareMap)`, `Robot(HardwareMap, Clock)`, and a composition constructor
  taking pre-built subsystems so tests assemble a robot from fakes.
- Construction: `Hardware.reset()` → every `LynxModule` to `BulkCachingMode.MANUAL` → collect
  voltage sensors once → build subsystems with the injected `Clock` → wire suppliers → `new Macros(this)`.
- `readSensors()`: clear every hub's bulk cache, `limelight.update()` (the tag list), the
  storage-entrance `ColorSensor` every loop (an edge must not be missed) **but only when
  `wireSuppliers()` trusted it**, then **one** of the fitted presence sensors (full, transfer, feed)
  per loop in rotation (three fitted means each is up to ~60 ms old, fine for pieces that sit), tick
  `MatchClock`, sample battery every `VOLTAGE_SAMPLE_MS` (voltage is not bulk-cached; report the
  **lowest** sensor). Each colour-sensor half (colour, distance) is its own I2C transaction, so the
  loop asks only for what it consumes (`ColorSensor.update(boolean colour)`): distance wherever a
  sensor has it, the hue only where it is the presence signal (no distance) or the G408 reject is
  wired. Benches call `robot.setReadAllSensorData(true)` and read everything.
- `writeActuators()`: `intake`, `storage`, `transfer`, `shooter`, `openLoopDrive`, then
  `drivetrain.update()`.
- Cross-subsystem wiring is **supplier injection in `Robot.wireSuppliers()`**, e.g.
  `storage.setEntranceSupplier(this::pieceEnteringStorage)`, `intake.setFullSupplier(storage::isFull)`,
  `transfer.setAtFeedSupplier(this::pieceAtShooterFeed)`. A subsystem never imports another subsystem.
- **`wireSuppliers()` is also the one place that decides which sensor is trusted** (fixthese R2-A3,
  R2-B1). A `null` supplier means "no sensor" and every fallback follows from it (unknown count and
  blind shooting, dead-reckoned exits, timed pulses, no self-detected full). Every point judges
  presence with `pieceNear` (distance when the device has it, else hue). The entrance is wired only
  when it has distance or `PieceType.HUES_CALIBRATED`; the G408 reject only when the hues are
  measured. `Robot.sensingSummary()` prints the result on every init card and bench footer. A sensor
  is trusted because it was measured, not because it is in the configuration.
- There is no localisation step: every BIOBUZZ AprilTag rides on a moving HIVE CELL and cannot
  localise the robot (docs/04 §3), so the pose is the Pinpoint's alone and the tags are used for
  aiming only (§16). `abortMacro()` is the shared cancel path (cancel the drivetrain path, mark the
  macro cancelled).

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
- Tunables are plain `public static` fields (no Panels `@Configurable`); edit and redeploy, or nudge
  one on a bench. A static lives until the Robot Controller app restarts, so `util/diagnostics/Tunables`
  snapshots the compiled values at the first OpMode init and every match init card lists what differs.
- `VelocityMotor.write` reaches the bus only when the value changes (and every
  `REFRESH_EVERY_N_WRITES` loops regardless); `Intake` samples motor current once per `update()`,
  and only while the roller is pulling (`getCurrentAmps()` is NaN otherwise: an idle roller costs
  no ADC transaction).

## 5. OpModes

**`Controls`** — one enum entry per binding: `(Pad, button, description, edge predicate | axis
function)`. `wasPressed(gp1, gp2)`, `axis(gp1, gp2)` (the SDK's stick-up-is-negative sign applied
here only), `helpLines()` renders the init card. The SDK's `*WasPressed()` consumes on read, so
`Controls.read(gp1, gp2)` takes one immutable `Snapshot` per loop and `Teleop` decides from it.
Adding a control is one entry, and a control that steers the robot declares it
(`Needs.DRIVETRAIN`). `Teleop` refuses those generically
(`Snapshot.anyPressed(Controls::requiresDrivetrain)` without a follower), so a new macro cannot slip
past the gate. Driver pad: drive axes, slow mode, drive-frame toggle, reset
heading, abort, aim lock (hold R-trigger), Pedro paths to the shooting spot and park, snap headings
(A and X are unbound). Operator pad: intake / outtake / eject / stop-or-cancel, intake-until-full,
shoot one / all, flywheel arm, TIP counted, count = 4 / count = 0 (dpad up / left), debug toggle.

**`Teleop`** — `onInit()` schedules exactly one drive default (`drivetrain.driverControlCommand(fwd,
strafe, turn)` with `DriveScaling.shape(...) * slowScale()` when Pedro is tuned, else
`openLoopDrive.driverControlCommand(...)`: robot-centric, no heading hold, drive macros off) and every
mechanism's `defaultIdleCommand()`, and inherits the pose, alliance and **piece count** from
`PoseStorage`. `onInitLoop()` lets the driver flip the alliance on the dpad (it always wins over what
auto left) and shows the help card.
Without a trusted storage-entrance sensor the count is unknowable, so the "Pieces" line shows
`?  (assumes 4 ...)` and the shoot buttons fire blind (`Macros.piecesOnBoard()`, governed by
`Macros.ASSUME_FULL_WHEN_UNCOUNTED`); the operator's dpad up sets it to 4, which also holds the
intake roller. `onDecide()` reads one `Controls.Snapshot`,
handles driver then operator input, holds or releases the drivetrain aim lock from the right
trigger (`Drivetrain.setAimLock` fed by `Macros.aimHeading` on the current up-CELL from
`game/Field`; the CELL pose and tag range are recomputed only when the alliance or the TIP count
changes), then re-schedules the flywheel hold if a macro preempted it. The
sticks abort only a macro that owns the drivetrain; operator X cancels any macro. Haptics rumble
on **transitions** (macro success/failure, a piece in, storage full as two blips, final 20 s as one
long buzz so the two cannot be confused).
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
B unlocks; reads every edge each poll. START runs whatever is shown, locked or not: `MainAuto` blinks
`NOT LOCKED` on the init card and reports `started UNLOCKED` while running. That is safe only because
the hardcoded routine is alliance-safe; **the Pedro-path auto must schedule its drive legs only when
`selector.isConfirmed()`** and otherwise fall back to shooting in place.

**Init-card warnings (every match OpMode)** — `MatchOpMode.reportBuildWarnings()`: a first line
`!! TUNING BUILD` when `BuildFlavor.isTuningBuild()` finds the AutoTune library in the APK (R704), and
`!! TUNED THIS SESSION` followed by every tunable whose value differs from the compiled default.

**`opmodes/test/`: the benches and `SelfTest`** — iterative OpModes on `BenchOpMode`, group "Bench".
They build the real `Robot` (same config names, directions, velocity code and sensor predicates as the
match OpModes) and drive one subsystem through its intent setters with no Ivy: each loop is
`readSensors()` → the bench's buttons → `writeActuators()`. They exist because the code is otherwise
proven only against fakes, and because every "measure this" constant in HANDOFF section 9 needs a
tool: each bench's card names the constants it feeds and shows the value to paste. `SelfTest` is a
clock-stepped state machine (config names, battery, each velocity mechanism at a low speed with the
sign of the measured velocity checked, colour sensors, Limelight fps, drive layer, localizer settled)
that leaves a PASS / FAIL / SKIP table on the screen; SKIP means not fitted. Every bench footer shows
the loop stats, `Robot.sensingSummary()` and the tunables changed this session; BACK on gamepad 1
restores them all. Nothing is restored on stop (tune on the bench, then drive it, is the pit
workflow). `Bench: Shooter` and `Bench: Storage` flip `SECOND_MOTOR_DIRECTION` live on dpad right. `BenchOpModesTest` runs
every one of them through `init / init_loop / start / loop` on the fakes. Wheel directions are not a
bench: the SDK's **TestHardware** utility OpMode (11.2+) spins any motor by config name, and the
Mecanum Tuner on a `-Ptuning` build does the same with a web UI.

## 6. Subsystems

| Subsystem | Hardware | States / contract | Commands | Sensors consumed |
|---|---|---|---|---|
| `OpenLoopDrive` | Pedro `revhub.drivetrains.Mecanum` on `Constants.drivetrainConfig` (names + directions, no tuning) | `drive(f, s, t)` / `stop()` set intent; `update()` writes only on change so an idle instance never fights the follower | `driveForMsCommand(f, s, t, ms)` (clock-based), `defaultStopCommand` | none |
| `Drivetrain` | Pedro `Follower` via `PathFollower` | pose, `isFollowingPath()`, heading hold (with an aim-lock setpoint via `setAimLock`, for the fixed shooter), `holdHeading(rad)`, field/robot-centric | `driverControlCommand` (default), `followLazyCommand(supplier, holdEnd)`, `followPathCommand`, `turnToCommand(rad)`, `holdCommand()` | Pinpoint via Pedro |
| `Intake` | 1 `DcMotorEx` velocity (`VelocityMotor`) | `Mode {IDLE, INTAKING, OUTTAKING, EJECTING}`, `isBlockedByFullStorage()` (roller held still at 4, G407), anti-jam only while actively intaking, `getCurrentAmps()` sampled only then (NaN otherwise) | `intakeCommand`, `outtakeCommand`, `ejectCommand`, `stopCommand`, `defaultIdleCommand` (−1, SUSPEND, QUEUE) | `fullSupplier` from Storage, `rejectSupplier` (G408, off) |
| `Storage` | 1–2 `DcMotorEx` velocity (Gecko side wheels, BRAKE) | `count()` 0..4 from rising edges (entrance +1, exit −1), `setCount`/`markEntered`/`markExited`, `isFull()` (count or sensor), `hasPiece()`, `Mode {IDLE, ADVANCING, REVERSING}` | `advanceOneCommand()` (until an exit event or `ADVANCE_TIMEOUT_MS`; skips when empty), `advanceUntilCommand(cond, timeout)`, `advanceForMsCommand(ms)`, `stopCommand`, `defaultIdleCommand` | `entranceSupplier`, `fullSupplier`, `exitSupplier` (transfer sensor) |
| `Transfer` | 1 `DcMotorEx` velocity (BRAKE) | `hasPieceInLift()`, `pieceAtFeed()`, `hasFeedSensor()`; `Mode {IDLE, LIFTING, FEEDING, REVERSING}` | `liftOneCommand()` (until staged at the feed; refuses to double-feed), `feedCommand()` (until the feed sensor clears + dwell), both timed pulses without a feed sensor; `feedForMsCommand(ms)`, `stopCommand`, `defaultIdleCommand` | `inLiftSupplier`, `atFeedSupplier` (null = no sensor) |
| `Shooter` | 1–2 `DcMotorEx` velocity (FLOAT), RPM via `TICKS_PER_REV` | `setTargetRpm`, `spinUp`, `idle`, `atSpeed()` (in band for `AT_SPEED_HOLD_MS`, read live against the clock), `Mode {IDLE, SPINNING_UP, READY}` derived | `waitForSpeedCommand()` (requires nothing), `holdSpeedCommand()` (never done; its end leaves the target to the next owner, the default idles it a loop later), `defaultIdleCommand` | none; feeding is `ShootCycle`, which the macro runs while holding the shooter |
| `Limelight` | `Limelight3A` | see §16 | none (data source) | camera |
| `ColorSensor` | `NormalizedColorSensor` per point | see §17 | none (data source) | sensor |

Piece-flow interlocks are expressed as suppliers wired in `Robot`, and as `waitUntil` conditions in
`Macros`; no subsystem polls another.

## 7. Templates

- `VelocityMotor`: `RUN_USING_ENCODER`, `setTarget` / `update()` or `write(ticksPerSec)`,
  `atSpeed(tolerance)`, `getCurrentAmps()`, the SDK PIDF read at construction and restorable,
  terminal constructor `(DcMotorEx, direction, zeroPower)`. Used by Intake, Storage, Transfer,
  Shooter so the spec-vs-measured ticks/rev question is answered once. `write` reaches the bus only
  on change (refreshed every `REFRESH_EVERY_N_WRITES` loops). **Ticks per rev must be measured**,
  not read off a spec sheet: pairing the 312 RPM part's 537.7 ticks/rev with a 435 RPM motor
  overstates the ceiling by ~40%. (V1 has no positional mechanism and no servo, so the Guide's
  `Mechanism` / `PositionalMotor` / `PositionalServo` templates were removed in Round 4.)

## 8. Drivetrain on Pedro 3.0.0: the `PathFollower` re-spec

`PathFollower` is the seam that keeps everything else independent of Pedro. Its 3.0.0 shape:

```java
public interface PathFollower {
    void update();                       // once per loop
    Pose pose();  void setPose(Pose p);
    void manual(double forward, double strafe, double turn);   // robot-frame powers
    void follow(Path path);              // restarts from t = 0
    boolean atParametricEnd();           // path geometry finished (true whenever not following)
    void hold(Pose pose, boolean scaled);
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
true when not following, `follow` restarts, the mode implied by the last call), so
`DrivetrainCommandTest` pins the behaviours that a Pedro upgrade could silently change.

## 9. `commands/Macros`

Multi-subsystem, one-button, bounded, reporting. Each macro is `reporting(name, body, outcome,
alongside...)` = `deadline(sequential(begin, body, finishWith), onInterrupt(markCancelled), alongside...)`;
every wait is `Waits.bounded(work, TIMEOUT)` = `race(work, Waits.waitMs(TIMEOUT))` on the injected
`Clock` (Ivy's own `waitMs` is wall-clock); `Outcome {IDLE, RUNNING, SUCCESS, TIMED_OUT, NO_TARGET,
CANCELLED}`; paths are built inside `followLazyCommand` suppliers at start, never stored;
requirements come from the commands composed.

**Ivy groups cost loops.** `Sequential` hands off one child per loop and never executes the child
it just started, so every child boundary is a whole loop, an `instant` is a loop by itself, and
nesting compounds (docs/01 §B.5). That is fine for a drive macro whose path takes seconds and
disastrous for the shooting cycle, whose steps are hundreds of milliseconds: the old nine-level tree
spent 7 idle loops between "flywheel recovered" and the next pulse and ~0.7 s per four-piece run.
**The shooting cycle is therefore one hand-written command, `commands/ShootCycle`**: a state machine
(`SPIN_UP → [ADVANCE →] [LIFT →] FEED → RECOVER → …`) on the clock and the sensors, every
zero-duration transition chained inside one `execute()`, requiring the storage and transfer only.
The sensorless variant runs the storage pulse beside the transfer leg from the same loop; with
sensors the same states wait on the exit edge, the staged piece and the cleared feed. Shot counting
(`pieceWasShot`: the sensors that exist must agree; a pulse count without them) and the
dead-reckoned storage count are unchanged. The measured four-piece sensorless run went from 3.96 s
to 3.26 s on the JVM (`MacrosTest`).

| Macro | Composition | Success |
|---|---|---|
| `intakeUntilFull()` | `intakeCommand` raced with `waitUntil(storage::isFull)` and timeout | count rose or full; a timed run on a robot that cannot detect full |
| `shootOne()` / `shootAll()` | snapshot instant, then `conditional(pieces > 0, bounded(ShootCycle(1 or piecesOnBoard())))`, with `heldFlywheel()` alongside (the shooter hold starts inside `schedule()` and rides the whole macro); never requires the drivetrain, so driving and the aim lock continue through a shot | as many shots counted as were on board |
| `aimAndShootAll(Pose, minTag, maxTag)` | `sequential(bounded(aimCore), bounded(ShootCycle))`, flywheel spinning up during the aim; the auto's move | same |
| `snapToHeading(rad)` | `bounded(drivetrain.turnToCommand(rad))` | within `SNAP_TOLERANCE_DEGREES` |
| `driveTo(Pose)` | `bounded(followLazyCommand(line from the current pose))`; no path inside `MIN_PATH_INCHES` | within `DRIVE_TO_TOLERANCE_INCHES` |
| `aimAt(Pose, minTag, maxTag)` / `aimHeading(...)` | the drivetrain turns (Pedro hold, re-issued as a visible tag refines the bearing) until the shooter's firing side faces the CELL: heading = bearing − `Shooter.HEADING_OFFSET_RAD`; the OpMode passes the CELL and tag range from `game/Field`. **The front camera sees the tags only while the robot faces the HIVE**, so `aimHeading` remembers the tags' disagreement with odometry and keeps applying it for `AIM_BIAS_MAX_AGE_MS` or until the pose is rewritten (`Drivetrain.getPoseWrites()`): face the HIVE, take the correction, turn, shoot by it | within `AIM_TOLERANCE_DEGREES` |

Success is measured after the fact against a snapshot taken at the start.

## 10. `game/`

Everything that changes with the season and nothing else. `Field`: the frame (origin A1), tiles,
HIVE CELL poses and tag ranges per alliance, the up-CELL after TIPs, the LOADING ZONE centre, the
pre-load count: only what the code uses (the FLOWER, GARDEN and inventory numbers live in docs/04).
`PieceType` (POLLEN, NECTAR): hue windows, saturation/value floors, `HUES_CALIBRATED`,
`isAtSensor(ColorSensor)`, `nectarAllianceAt`. `FieldPoses`: BLUE poses using Pedro
`new Pose(x, y, Math.toRadians(h))`, rotated by `FieldConstants.forAlliance`; placeholders until
the field is measured. `Robot` is the only production class that reads `PieceType`.

## 11. `pedro/`

`Constants.java`: `public static MecanumConfig drivetrainConfig`, `PinpointConfig localizerConfig`,
`ForesightConfig foresightConfig`, `Follower create(HardwareMap)` — see doc 01 §A.1 for the worked
version (`git show 20768b3:.../pedro/Constants.java`). Motor and Pinpoint names come from
`HardwareNames`. `Tuning.java`: `@Tuner` static factories for `MecanumTuner`, `PinpointTuner`,
`ForesightTuner`, `Tests` (doc 01 §A.7; note the two factories' differing argument orders).
`procedures/`: the four Quickstart AutoTune procedures the robot uses (Mecanum, Pinpoint,
Foresight, Tests), untouched; the five for localizers the robot does not own were deleted. As of 2026-09-13 only
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
| `diagnostics/MatchLogger` | CSV under `/sdcard/FIRST/data/`, one row every `MatchOpMode.LOG_EVERY_N_LOOPS` loops (2) | knows nothing about the robot: header in the constructor, cells in `logRow(Object...)`; `Robot` supplies `BIOBUZZ_COLUMNS` (26 columns incl. storage count, transfer state, heading hold / aim lock, shooter target vs actual); `NaN` for follower cells without a follower and for an unsampled intake current; flush every 50 rows; catches `Exception`; `MatchLogger(File, tag, header)` for JVM tests |
| `diagnostics/Tunables` | snapshot every `public static` non-final field of given classes; `changed()`, `restoreDefaults()` | first snapshot of a class wins; the class list is `opmodes/RobotTunables` |
| `diagnostics/BuildFlavor` | `isTuningBuild()`: is `com.pedropathing.tuning.autotune.Tuner` on the classpath | the runtime half of the `-Ptuning` guard (R704) |
| `field/FieldConstants` | `FIELD_SIZE_INCHES = 144`, `Symmetry {MIRROR_X, MIRROR_Y, ROTATE_180}` + `SYMMETRY`, `forAlliance`, `mirrorAcrossX/Y`, `rotate180` (heading transformed too), `isInsideField` | the one source of field size, used by `game/`; `SYMMETRY = ROTATE_180` per `docs/04` §2.5 |
| `field/PoseStorage`, `Alliance`, `StartPosition` | auto → teleop handoff; enums (`StartPosition` carries a driver-facing `label()`) | survives OpMode switch, not RC restart |
| `hardware/Hardware`, `HardwareNames` | fail-soft lookup; every config name | `Robot` calls `Hardware.reset()` first |
| `math/Angles` | `normalizeAngle`, `angleError` | feed controllers an error, never a raw angle |
| `math/ColorMath` | `toHsv` on floats, `hueDistance`, `matches` | §17 |
| `math/DriveScaling` | deadband, expo, `shape`, `slowScale` | |
| `time/Clock`, `SystemClock`, `MatchClock` | injectable monotonic time; BIOBUZZ periods (30 s AUTO, 120 s TELEOP, no endgame), `isFinalSeconds()` at 0:20, `isExpired()` at the buzzer | `forPeriod(period)`; the 1:00 FLOWER window has no code until a FLOWER mechanism exists |

The `Clock`-based `waitMs` / `bounded` commands live in `commands/Waits` (doc 01 §B.5).

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
| `util/math/*`, `util/time/*`, `util/control/JamDetector`, `util/hardware/*`, `diagnostics/LoopTimer`, `opmodes/Controls`, `AutoSelector`, `subsystems/ColorSensor`, tests' `FakeClock`, `FakeDcMotorEx` | same | verbatim, minus `@Configurable` |
| `subsystems/Limelight`, `util/field/{FieldConstants, PoseStorage}`, `game/FieldPoses`, `Robot`, `MainAuto`, `SelfTest` | same | `Pose` import → `com.pedropathing.math.Pose`, accessors `x()/y()/heading()` |
| `util/field/PoseFusion`, `util/math/VisionMath`, `MedianFilter`, `diagnostics/RateLimiter`, the blob half of `Limelight`, the collect/align macros | removed (Round 4) | no static tag to localise from this season; the camera faces front and is for AprilTags only |
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
  limelight.update()      → one LLResult cached; the fiducial list
  entrance.update(hue?)   → every loop when trusted: distance on a V3; the colour half only where hue is
                            the presence signal (no distance) or the G408 reject is wired
  presence.update(hue?)   → one of the fitted full / transfer / feed sensors per loop, in rotation;
                            distance alone where they have it
Robot suppliers (wired by wireSuppliers; null = not fitted or not trusted)
  storage.entrance ← pieceNear(entrance) and not an opponent NECTAR being rejected
  intake.reject    ← opponent NECTAR hue at the entrance (only once HUES_CALIBRATED)
  intake.full      ← storage.isFull()  ← count >= 4 OR pieceNear(storage-full sensor)
  transfer.atFeed  ← pieceNear(shooterFeed sensor)
Scheduler.execute()        commands read the cached values
Robot.writeActuators()     mechanisms apply intent
```

## 16. Limelight 3A

**BIOBUZZ scope.** Every AprilTag is a cluster on the underside of a moving HIVE CELL (IDs 30–45,
3.25 in, cluster origin at the centre of the CELL opening). The SDK states they are unsuitable for
field localisation, so there is no botpose in the wrapper; the AprilTag pipeline is used for
**aiming** only (the mean tx of the tags on the CELL the shooter must hit: red starts on tags
34–37, blue on 42–45, flipping after each TIP). **The camera is front-mounted** (Round 4 decision):
it sees the tags while the robot faces the HIVE and loses them once the rear shooter is turned
toward it, which is why `Macros.aimHeading` keeps the correction it took while they were visible
(§9). There is no piece detection. See docs/04 §3. Note also R704: no dashboard/streaming tools
during matches.

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

**Wrapper contract (`subsystems/Limelight`)**, one job:

```java
APRILTAG_PIPELINE_INDEX = 0;          // set once in the constructor; there is no other pipeline
CAMERA_YAW_OFFSET_DEGREES = 0;        // the camera's forward axis relative to the robot's: 0 = front
MAX_STALENESS_MS = 250;
```

- `update()`: `getLatestResult()`; on invalid/empty the tag list is empty; else it is the frame's
  `getFiducialResults()`.
- `hasTarget()` = valid and not stale; `getTx/getTy/getTa` the primary target; `getStatus()` for the
  bench and SelfTest (`fps > 0`).
- `getTagTx(minId, maxId)`: the mean `getTargetXDegrees()` of the visible tags whose ID is in the
  range, NaN when none; `getTagCount()`, `getTags()`. `Macros.aimHeading` turns it into a field
  bearing (`heading + CAMERA_YAW_OFFSET − tx`) and remembers its disagreement with odometry.
- The constructor's `pipelineSwitch` and `start()` are synchronous HTTP calls and fail soft
  (`Hardware.recordFailure`) when the camera is unpowered. `stop()` from `Robot.stop()`.
- Botpose, MegaTag2 and the colour pipeline are not wrapped (Round 4): nothing this season can use
  a field pose from tags that move, and the camera is not for piece detection. The SDK tables above
  stay as reference.

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
`update(boolean colour)` caches the distance (on a `DistanceSensor`) and, when asked, one
`NormalizedRGBA` converted with `ColorMath.toHsv(r, g, b, hsv)`; `update()` reads both;
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
- Any pose write releases the heading hold and invalidates anything derived from the old frame
  (`Drivetrain.getPoseWrites()`).
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

- **The camera faces front and the shooter fires out the rear** (decided 2026-09-16). The tags are
  in view only while the robot faces the HIVE, so `Macros.aimHeading` takes the correction then
  (the tags' bearing minus odometry's) and keeps applying it for `AIM_BIAS_MAX_AGE_MS` or until the
  pose is rewritten. Drive up facing the HIVE, then pull the aim lock: the turn is aimed by the
  tags. The teleop card shows `tag-corrected` when a correction is in force. If the camera ever
  moves to the rear, `CAMERA_YAW_OFFSET_DEGREES = 180` and the same maths works live.
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
- **Shot counts and `SUCCESS` are pulse counts without sensors.** `ShootCycle.pieceWasShot` degrades
  to `true` for every sensor that is absent. The auto card's `4 (shootAll : SUCCESS)` means four
  pulses ran.
- **The storage count has three states.** Known from a sensor edge or `setCount`; unknown (no
  trusted entrance sensor, nothing on record: shoot buttons fire blind, the card shows `?`); handed
  over from auto (`PoseStorage`), which is the dead-reckoned remainder. On the sensorless robot the
  operator sets it (dpad up = 4, dpad left = 0), and count 4 is the only thing that holds the roller
  unless `sensor_storage_full` is fitted. Fit that sensor first.
- **Plugging in a sensor does not make it trusted** (2026-09-15). A hue-only entrance sensor is
  ignored for counting until `PieceType.HUES_CALIBRATED`; a V3 counts by distance at once. Read the
  `Sensors:` line on the init card.
- **Each colour-sensor half is an I2C transaction, none bulk-cached.** The loop reads only what it
  consumes (Round 4): a trusted entrance every loop, distance only on a V3 unless the G408 reject is
  wired; the fitted presence sensors one per loop in rotation, distance only; an untrusted entrance
  not at all. Benches read everything. Watch `Loop` on any bench footer; the anti-jam window wants
  at least ten samples (`STALL_TIMEOUT_MS / p95`).
- **The intake current is NaN unless the roller is pulling.** The ADC read happens only while
  intaking and not blocked; the CSV and the debug card show `NaN` the rest of the time, which is
  "not sampled", not a fault.
- **The match log is every second loop** (`MatchOpMode.LOG_EVERY_N_LOOPS`), 25 rows a second.
- **A bench edit outlives the bench.** Statics last until the app restarts; the match init card lists
  every changed tunable. Clear it by restarting the Robot Controller app, or BACK on a bench.
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

Driver-station OpModes: `Teleop` (Main), `Auto: shoot 4 + leave` (Main), the six `Bench: …`
OpModes and `SelfTest` (Bench; run SelfTest first at every event). Tuning has no OpMode; it is the
web UI on a `-Ptuning` build.
