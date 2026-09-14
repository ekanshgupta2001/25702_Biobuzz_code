# HANDOFF — FTC Team 25702 BIOBUZZ Robot Code

State as of **2026-09-13 (late night)**. Read this first; it tells you what exists, what does not, why
things are the way they are, and what to do next. Everything here is backed by a file in the repo.

---

## 1. What this is

- **Team 25702**, FTC 2026-27 game **BIOBUZZ presented by RTX** (kickoff 2026-09-12).
- Code targets the **V1 robot** (`~/Downloads/BIOBUZZ_V1_Robot_Physical_Architecture.md`, with one
  correction from the team): mecanum drivetrain, front funnel, 16 mm compliant intake roller, short
  ramp, low horizontal **4-piece** storage on Gecko side wheels, rear 90° vertical Gecko transfer,
  and a compliant flywheel shooter **fixed to the chassis, firing out the rear**. **No turret** (the
  spec's turret was dropped; the drivetrain aims), **no Flower mechanism, extension or diverter**
  (V2; no code).
- Stack: **Pedro Pathing 3.0.0** (`com.pedropathing:revhub:3.0.0` + `core:3.0.0`), **Ivy 1.1.1**
  command scheduler (`com.pedropathing.ivy:pedro:1.1.1`), **AutoTune** (`tuning:1.0.0`), FTC SDK
  **11.2.1** (inherited from the Pedro Quickstart this repo was cloned from), Gradle 9.1.0,
  AGP 8.13.2, Java 8 source level. **No Panels / FTC Dashboard.**
- Architecture is the FTC_Guide reference (`~/Documents/FTC_Guide`) re-based on Pedro 3 and
  Ivy 1.1.1: `Robot` / `subsystems` / `commands` / `opmodes` / `game` / `util`, every subsystem an
  Ivy resource with its own command factories, every loop `read → decide → Scheduler.execute() →
  write`, JVM unit tests against fakes, and the real OpMode lifecycle testable on the JVM.

## 2. State of the build

| Layer | Status | Tests |
|---|---|---|
| `util/**` (18 classes) | **implemented** | 140 |
| `subsystems/**` (10 classes + 4 templates) | **implemented**; `OpenLoopDrive` is the untuned-drivetrain stopgap | 78 |
| `Robot.java` | **implemented**: composition, supplier wiring, loop halves, localization, `logCells` | 11 |
| `commands/Macros.java`, `commands/Waits.java` | **implemented**: intakeUntilFull, collectPiece, alignToPiece, shootOne / shootAll, aimAt, aimAndShootAll, driveTo, relocalize, snapToHeading; clock-based waits | 23 + 2 |
| `game/Field.java`, `Scoring.java`, `FieldPoses.java` | **implemented** from docs/04; INFERRED numbers flagged in §9 | 10 + 7 |
| `game/PieceType.java` | **partial**: POLLEN / NECTAR sizes, target height, `isAtSensor` / `classify`; hue windows are placeholders | — |
| `opmodes/MatchOpMode.java` | **implemented**: extends `OpMode`, final lifecycle, `buildRobot()` / `openLogger()` seams | via `TeleopTest` |
| `opmodes/teleop/Teleop.java`, `Controls.java` | **implemented**: `@TeleOp "Teleop"` (see the bindings in §5) | 15 + 6 |
| `opmodes/auto/MainAuto.java`, `AutoRoutine.java`, `AutoSelector.java` | **implemented**: `@Autonomous "Auto: shoot 4 + leave"`, hardcoded and open-loop for the first event (see §5) | 3 + 5 + 3 |
| `opmodes/SelfTest.java`, `ConceptCommands.java` | **stubs** | — |
| `pedro/Constants.java`, `pedro/Tuning.java` | `drivetrainConfig` filled (names + directions, no tuning needed); `create()` still `null` until AutoTune, so `Drivetrain` fails soft; `Tuning` is the Quickstart stub | — |
| `pedro/procedures/*` | Quickstart AutoTune procedures, untouched | — |
| `docs/01..04` | written and kept current | — |

`./gradlew :TeamCode:testDebugUnitTest` → **295 tests, 0 failures** (no placeholders remain).
`:TeamCode:assembleDebug` builds the APK with `Teleop` and `Auto: shoot 4 + leave` registered on
the Driver Station.

**What was done on 2026-09-13, in order** (each step green before the next):
1. docs/01–04 researched and written; `util/**` ported and tested.
2. `subsystems/**` implemented on Pedro 3 / Ivy 1.1.1 behind the `PathFollower` seam.
3. `Robot` implemented; Macros state skeleton and `PieceType` constants scaffolded.
4. `Macros` filled in with Ivy groups and Pedro paths; `Waits` added; `Storage.hasExitSensor()` and
   the `Intake.captureCommand` end-condition fix; `game/Field`, `Scoring`, `FieldPoses` written.
5. `opmodes/` split into `auto/` and `teleop/`; `MatchOpMode`, `Controls`, `Teleop` implemented;
   `Macros.driveTo` added; APK assembled.
6. The team confirmed **V1 has no turret**: `Turret` deleted, `Shooter.HEADING_OFFSET_RAD` added,
   the aim law moved to `Macros.aimHeading`, an aim lock added to the drivetrain's heading hold,
   the teleop re-bound (aim on the driver's right trigger), docs corrected.
7. The **first-competition autonomous**, hardcoded because Pedro will not be tuned in time:
   `OpenLoopDrive` (Pedro's `Mecanum` on `Constants.drivetrainConfig`, no localizer), `AutoRoutine`
   (shoot the four pre-loads with `shootAll`, then a timed drive off the wall), `AutoSelector`,
   `MainAuto`; `shootAll` unrolled to dodge an Ivy `Repeat` crash; start poses turned to put the
   rear-firing shooter toward the HIVE.

Git: HEAD is the Quickstart's `b431238`; **nothing from this work is committed**. `git status`
shows `TeamCode/build.gradle`, `build.dependencies.gradle`, `pedro/Constants.java` (drivetrain
config) and `pedro/Tuning.java` (three unused imports someone added in Android Studio; harmless)
modified plus the new `docs/`,
`TeamCode/src/test/` and the new source packages. Suggested first commit: everything, as
"V1 architecture: docs, util, subsystems, Robot, macros, game, teleop, hardcoded auto, tests".

## 3. Read these first

| File | Why |
|---|---|
| `docs/02-robot-physical-architecture.md` | what the V1 robot is and which subsystem owns each mechanism |
| `docs/03-software-architecture.md` | the loop contract, package map, per-subsystem API, Limelight and colour-sensor specs, lessons-learned rules |
| `docs/01-libraries-pedro-3.0.0-ivy-1.1.1.md` | the two libraries' real APIs, verified from bytecode: read before touching `Drivetrain`, `Macros` or `AutoRoutine` |
| `docs/04-biobuzz-season-analysis.md` | sourced game/field/rule facts, and §9 "implications for our robot" |
| `opmodes/teleop/Controls.java` | every binding and the help card; change a button there and nowhere else |
| `game/Field.java` Javadoc | the field frame (origin corner, axes, heading) every pose and path depends on |
| `opmodes/auto/AutoRoutine.java` | the first-competition auto and its five tunables (`LEAVE_*`, `SETTLE_MS`, `SHOOT_BUDGET_MS`) |
| `util/hardware/HardwareNames.java` | the exact strings the Robot Controller configuration must use |

Each remaining stub's Javadoc header is the agreed contract for that file; implement to it.

## 4. Build and test

```bash
# Gradle 9.1 may reject a newer system JDK; the Android Studio JBR always works.
JAVA_HOME="/Applications/Android Studio.app/Contents/jbr/Contents/Home" \
  ./gradlew :TeamCode:compileDebugJavaWithJavac :TeamCode:testDebugUnitTest --console=plain

# Results: TeamCode/build/test-results/testDebugUnitTest/*.xml
# APK / deploy: the green Run button in Android Studio (or :TeamCode:assembleDebug)
```

Expect `BUILD SUCCESSFUL` plus three javac warnings about Java 8 source level (harmless).

## 5. Repo map

`TeamCode/src/main/java/org/firstinspires/ftc/teamcode/`

```
Robot.java                     impl  composition root: readSensors()/writeActuators(), supplier wiring, logCells()
opmodes/
  MatchOpMode.java             impl  abstract base (extends OpMode); final lifecycle read→decide→execute→write;
                                     buildRobot()/openLogger() seams let TeleopTest run the real lifecycle on the JVM
  SelfTest.java                STUB  LinearOpMode diagnostics, per-subsystem pass/fail
  ConceptCommands.java         STUB  teaching OpMode for Ivy concepts and traps
  teleop/
    Controls.java              impl  enum of every gamepad binding + help card; read() takes one Snapshot per loop
    Teleop.java                impl  @TeleOp "Teleop": Pedro manual-drive default command, macros (incl. Pedro paths
                                     to the shooting spot / park), aim lock on R-trigger, flywheel toggle, haptics, telemetry
  auto/
    MainAuto.java              impl  @Autonomous "Auto: shoot 4 + leave": selector in init, routine on start, PoseStorage
                                     every loop, buzzer safety net
    AutoRoutine.java           impl  hardcoded first-competition routine: shootAll, settle, timed leave (JVM-tested);
                                     the Pedro-path version replaces it once tuned
    AutoSelector.java          impl  dpad alliance / start-position menu, A confirms, B unlocks
subsystems/
  PathFollower.java            impl  interface: the slice of Pedro 3 Follower the Drivetrain calls
  PedroPathFollower.java       impl  production adapter; raw() exposes the Follower
  Drivetrain.java              impl  manual/field-centric drive, heading hold (+ aim-lock setpoint), followLazy/turnTo/hold commands
  OpenLoopDrive.java           impl  timed open-loop driving through Pedro's Mecanum (names + directions only); writes on intent change only
  Intake.java                  impl  velocity roller, anti-jam, full-storage interlock, capture command
  Storage.java                 impl  edge-counted 4-piece queue, advanceOne/advanceUntil commands, hasExitSensor()
  Transfer.java                impl  liftOne (no double-feed) and feed (sensor-or-pulse) commands
  Shooter.java                 impl  RPM flywheel, spinUp / holdSpeed / idle commands; fixed, fires out the rear (HEADING_OFFSET_RAD)
  Limelight.java               impl  AprilTag tx aiming (getTagTx), colour blobs, gated botpose (null this season)
  ColorSensor.java             impl  NormalizedColorSensor wrapper, one per sensor point
  templates/                   impl  Mechanism<S>, PositionalMotor<S>, PositionalServo<S>, VelocityMotor
commands/Macros.java           impl  bounded, outcome-reporting multi-subsystem macros (Ivy groups + Pedro paths); aimHeading law
commands/Waits.java            impl  Clock-based waitMs / bounded (Ivy's own waitMs is wall-clock)
game/Field.java                impl  field frame (origin A1), tiles, HIVE/CELLs, tag IDs, FLOWERs, zones, element counts
game/Scoring.java              impl  point values and ranking-point thresholds
game/FieldPoses.java           impl  BLUE poses (two starts, shooting spot, park, garden approach); RED via FieldConstants.forAlliance
game/PieceType.java            part  POLLEN / NECTAR sizes, target height, placeholder hue windows
pedro/Constants.java           part  drivetrainConfig (MecanumConfig) filled; PinpointConfig / ForesightConfig / create() await AutoTune
pedro/Tuning.java              stub  @Tuner factories for AutoTune
pedro/procedures/*.java        impl  Quickstart tuners (Mecanum, Pinpoint, Foresight, Tests, ...)
util/
  control/JamDetector          impl  current-based stall detection, pure logic
  diagnostics/LoopTimer, MatchLogger, RateLimiter   impl  (MatchLogger is generic: Robot supplies BIOBUZZ_COLUMNS)
  field/Alliance, FieldConstants (SYMMETRY = ROTATE_180), PoseFusion, PoseStorage, StartPosition
  hardware/Hardware (fail-soft lookup), HardwareNames (every config name, V1 + 5 sensor points)
  math/Angles, ColorMath, DriveScaling, MedianFilter, VisionMath
  time/Clock, SystemClock, MatchClock (BIOBUZZ timing, isFlowerUnlocked at 1:00)
```

Tests live in `TeamCode/src/test/java/.../teamcode/` mirroring the tree; fakes are
`subsystems/FakePathFollower` (mirrors Pedro 3's state machine), `subsystems/FakeDcMotorEx`,
`subsystems/FakePedroDrivetrain` (Pedro's motor-layer interface), `util/time/FakeClock` and
`opmodes/FakeTelemetry`. Every test does `Scheduler.reset()` in
`@Before` and `@After`. `RobotTest` assembles a whole robot from the fakes through the composition
constructor; `TeleopTest` and `MainAutoTest` run the real `Teleop` / `MainAuto` lifecycle (`init` →
`init_loop` → `start` → `loop`) on the JVM with real `Gamepad`s driven by `copy()`; `AutoRoutineTest`
runs the whole auto to completion in fake time.

**Teleop at a glance** (the source of truth is `Controls`; the init screen shows this card):

| Pad | Input | Does |
|---|---|---|
| driver | L-stick / R-stick X | drive (field-centric by default, Pedro `manual`, heading hold when the turn stick is centred) |
| driver | L-trigger (hold) | precision slow mode |
| driver | **R-trigger (hold)** | **aim lock**: heading hold points the shooter at the current up-CELL while the sticks still translate |
| driver | LB / Y / BACK | field ↔ robot centric / re-zero heading / abort macro (moving a stick also aborts a drive macro) |
| driver | A / X | collect a piece / turn to face a piece (camera) |
| driver | B / RB | Pedro path to the shooting spot / to park |
| driver | dpad | snap to 90 / 0 / 270 / 180° |
| operator | RB / LB / B / X | intake / outtake / eject / stop intake (X also cancels any macro) |
| operator | Y | intake until the storage is full |
| operator | R-trigger / A | shoot one / shoot all |
| operator | L-trigger | flywheel armed on/off (comes back by itself after a shot macro) |
| operator | dpad down | "our HIVE tipped": the aim target flips to the other CELL |
| operator | BACK | debug telemetry |

**Auto at a glance** (`Auto: shoot 4 + leave`, hardcoded for the first event, no Pedro tuning; it
pre-selects `Teleop` on the Driver Station): place the robot touching its wall with the **rear
(shooter) toward the up-facing CELL**, pick the alliance on gamepad 1's dpad during init and press A. On start: `storage.setCount(4)` →
`shootAll()` (one spin-up, four timed feed cycles, `SHOOT_BUDGET_MS` 20 s cap) → `SETTLE_MS` →
drive `LEAVE_DIRECTION` (default BACKWARD, away from the wall) at `LEAVE_POWER` for `LEAVE_MS` →
stop. The alliance goes to teleop through `PoseStorage`; there is no pose. At the 30 s buzzer
anything still running is cancelled and every mechanism stopped (G403).

## 6. Decisions already made, and why

| Decision | Why |
|---|---|
| V1 scope only | The team's V1 spec; the final-robot spec was used by mistake for half a day and everything Flower/diverter-related was removed. |
| No turret on V1: the shooter is fixed and fires out the rear; aiming is the drivetrain heading | Team correction on 2026-09-13. A standing aim cannot be an Ivy command (a never-ending priority-0 drivetrain command would suspend driver control for good), so it is an aim-lock setpoint inside `Drivetrain`'s heading hold, fed by `Macros.aimHeading` (bearing − `Shooter.HEADING_OFFSET_RAD`, tag-refined). `aimAt` is the one-shot turn for auto. |
| `FieldConstants.SYMMETRY = ROTATE_180` | The BIOBUZZ field is 180° rotationally symmetric (GARDEN A1/F6, LOADING ZONE A5/F2). A mirror puts every blue auto in the wrong place. |
| Field frame origin = A1 corner (audience-side red), +X toward column F, +Y toward row 6 | Pinned by the 180° rotation mapping A5→F2 and A1→F6; written once in `Field`'s Javadoc, checked by `FieldTest`. |
| No AprilTag localisation; tags are aiming targets | Every tag rides on a moving HIVE CELL; the SDK v12 notes say they are unsuitable for localisation. `PoseFusion` stays for a future static reference; `Limelight.getBotposeAsPedroPose()` is expected to return null. |
| `PathFollower` seam over Pedro | Every drivetrain command runs on the JVM against `FakePathFollower`; Pedro's behaviour changes are caught by `DrivetrainCommandTest`, not on the field. |
| Keep `Drivetrain.followLazyCommand`, never bare `PedroCommands.follow` | Ivy 1.1.1's `PedroCommands.follow` has no requirement and no `setEnd`; an interrupted follow would keep driving. Ours holds at `path.endPose()` on a natural end with `holdEnd` and hands back with `manual(0,0,0)` otherwise. |
| `VelocityMotor` template | Four mechanisms are velocity rollers/flywheels; intent-then-write and ticks/rev live in one place. |
| Default commands: priority −1, `SUSPEND`, `BlockedBehavior.QUEUE`, logic in `setExecute` | Ivy resumes a suspended command without calling `start()`; `CANCEL` would drop it for the whole match. |
| Storage hard-stops the intake at 4 | BIOBUZZ G407: at most 4 controlled scoring elements. `Intake.setFullSupplier(storage::isFull)`. |
| Transfer/lift commands fall back to timed pulses without a feed sensor | So the robot cycles on a bench before sensors are fitted; wire `setAtFeedSupplier` to turn on the interlocks. |
| Sensorless shooting dead-reckons the storage count | With no exit sensor the count only goes up; `feedOneCore` calls `storage.markExited()` after a completed pulse cycle so the intake interlock releases (`Storage.hasExitSensor()`). |
| `Macros.INTAKE_RUNS_STORAGE = true` | Whether the side wheels must run for the channel to accept a piece is open (§10); one flag flips it. |
| `MatchLogger` knows nothing about `Robot` | The logger takes a header and cells; `Robot.logCells(loopMs)` supplies the row in `MatchLogger.BIOBUZZ_COLUMNS` order. Testable with a temp dir. |
| No Panels, no dashboard | R704 prohibits streaming tools during matches; tunables are plain `public static` fields. |
| Macro timeouts on the injected `Clock` (`commands/Waits`) | Ivy's `waitMs` is wall-clock; `MacrosTest` runs every macro deterministically on `FakeClock`, no sleeps. |
| `shootOne` / `shootAll` never require the drivetrain | Driving and the aim lock continue through a shot; autonomous aims with `aimAt` / `aimAndShootAll`. |
| `opmodes/` split into `auto/` and `teleop/`; both depend on `opmodes.MatchOpMode`, never on each other | Shared lifecycle in one place; season OpModes separated. |
| Teleop reads one `Controls.Snapshot` per loop | SDK `*WasPressed()` consumes on read; a press during a macro must be consumed, not latched to fire when the macro ends. |
| Sticks abort only a macro that owns the drivetrain; operator X cancels any macro | A driver grabbing the sticks must not cancel a shot in progress. |
| The flywheel hold is a standing operator wish, re-scheduled after any macro | Ivy ends (does not suspend) a preempted priority-0 command; `restoreFlywheelHold()` brings it back. |
| `MatchOpMode.buildRobot()` / `openLogger()` protected seams | The only way to run the real OpMode lifecycle on the JVM (`HardwareMap` and the SD-card logger are Android-only). |
| First-competition auto is hardcoded and open-loop (`OpenLoopDrive` over Pedro's `Mecanum`) | No AutoTune before the first event means no localizer, no paths and no aiming; motor names + directions are all `Mecanum` needs, and the same `drivetrainConfig` feeds the tuned follower later. |
| `OpenLoopDrive` writes only when its intent changes | Once stopped it goes silent, so it can never fight the tuned follower that will share the motors. |
| `shootAll` is unrolled into `Storage.CAPACITY` guarded steps, not Ivy's `repeat` | A `sequential` interrupted (by a timeout) before reaching a `Repeat` calls `end()` on it and NPEs; the auto's shooting budget makes that path reachable in a match. |

## 7. Rules that bite

**Pedro 3.0.0** (docs/01 §A)
- `new Follower(Localizer, Drivetrain, Algorithm)`; the Quickstart comment in `pedro/Constants.java` has the order wrong.
- Exactly one `follower.update()` per loop; it ticks the localizer itself.
- `atParametricEnd()` means the path is done. `isBusy()` only clears inside a hold; never use it as "still following".
- `PinpointLocalizer`'s constructor starts an IMU calibration: build in `init()`, sleep ≥1 s or re-zero in `start()`.
- Paths need a heading (`.constant/.linear/.tangent`) or `follow()` throws; `followLazyCommand` catches and records it.
- No callbacks, no `setMaxPower`, no `turnTo`: poll, use Ivy, or `path.with(cfg.maxPathSpeed.at(p))`. A turn is `hold(pose.withHeading(h))`.

**Ivy 1.1.1** (docs/01 §B)
- `Command.unless()` never finishes; use `conditional(cond, real, instant(() -> {}))`.
- `Commands.lazy(...)` contributes no requirements; always `.requiring(subsystem)`.
- `waitMs` is wall clock and not fakeable; use `Waits.waitMs(clock, ms)` / `Waits.bounded(...)`.
- A direct hardware call from an `instant` without a requirement is overwritten by the default command before the write.
- `Scheduler` is static: `Scheduler.reset()` first thing in `init()` and in every test.
- `deadline` and `parallel` end their unfinished children with the **group's own** end condition (bytecode-verified), so a child's `ec == NATURALLY` branch fires when the group finishes naturally. Judge by the sensor or condition, never by `EndCondition` (`Intake.captureCommand` does), or use `race`.
- A `sequential` hands off one child per `execute()`: a child's `start()` never runs at `schedule()` time.
- **Never put `repeat(...)` inside a group that can be interrupted before reaching it**: `Repeat.end()` dereferences a list it only builds in `start()` and NPEs (seen in `AutoRoutineTest`). Unroll into guarded `conditional` steps (`Macros.shootAllCore`).

**SDK gamepads and OpModes** (bytecode-verified on 11.2.1)
- `*WasPressed()` consumes its flag on read and only advances inside `Gamepad.copy()` / `fromByteArray()`. Read every edge once per loop (`Controls.read`), never twice; `resetEdgeDetection()` in `start()`.
- `new Gamepad()` and an `OpMode` subclass construct on the JVM (no Android in their constructors); `Telemetry` is an interface. `HardwareMap` is Android-only: keep it behind `buildRobot()`. In a test's inner OpMode subclass, `robot` resolves to the inherited field, not the fixture: write `Outer.this.robot`.

**Game and robot rules** (docs/04)
- G407 max 4 pieces; G408 never control opponent NECTAR (colour sensing at the storage entrance).
- R503 **8 motors and 8 servos max**: without a turret V1 needs 8–10 (drive 4, intake, storage 1–2, transfer, shooter 1–2). A single flywheel and a single storage motor fit exactly; otherwise storage and transfer share a motor. `HardwareNames` keeps the optional second-motor names.
- R704: no dashboard/streaming tools in matches. **Remove `com.pedropathing:tuning` from competition builds** (its web server is always on).
- Season SDK is **v12.0** (AprilTag cluster API); this repo is on 11.2.1.
- AUTO 30 s → 8 s no-motion transition (G403: do not INIT a twitchy TeleOp early) → TELEOP 120 s; no endgame; FLOWER unlock at 1:00.

## 8. Next steps, in order

What the next steps build on (all implemented and tested):
- **Robot**: composition ctor `Robot(Drivetrain, OpenLoopDrive, Intake, Storage, Transfer, Shooter,
  Limelight, ColorSensor ×4, Clock)` (pass `new ColorSensor(null)` for an unfitted point and
  `new OpenLoopDrive((com.pedropathing.drivetrain.Drivetrain) null, clock)` for no motors). The four sensor
  points' type-specific predicates live only in `Robot.wireSuppliers()`; absent exit / feed sensors
  pass `null` so Storage dead-reckons and Transfer pulses. `robot.logCells(loopMs)` is the log row;
  `stopMechanisms()` then `writeActuators()` is the mid-OpMode stop; `startMatch(Period)` builds the
  `MatchClock` that `readSensors()` ticks.
- **Macros**: every macro is `begin(name)` → bounded work → `finish(...)`, reporting an `Outcome`.
  Aim with `drivetrain.setAimLock(() -> macros.aimHeading(Field.cell(alliance, side), tags[0],
  tags[1]))` where `side = Field.upCellSide(alliance, tips)` and `tags = Field.tagRange(alliance,
  side)`; one-shot with `aimAt(...)`; shoot with `shootOne()` / `shootAll()`; auto's move is
  `aimAndShootAll(...)`; drive with `driveTo(pose)`. Abort is `Scheduler.cancel(macro)` +
  `robot.abortMacro()`.
- **OpModes**: extend `opmodes.MatchOpMode`, implement `logTag()` and `matchPeriod()`, fill the
  hooks; override `buildRobot()` / `openLogger()` only in tests. Alliance and pose reach teleop
  through `PoseStorage`, which `MainAuto` writes every loop.
- **Driving before Pedro is tuned**: `robot.openLoopDrive.driveForMsCommand(f, s, t, ms)` (Pedro's
  robot-frame signs: +forward ahead, +strafe left, +turn CCW). Once `Constants.create()` returns a
  follower, use `Drivetrain` / `Macros.driveTo` instead and leave `OpenLoopDrive` idle.

1. ~~Hardcoded auto~~ **done 2026-09-13** (see "Auto at a glance" in §5). **Before the first event,
   on the real robot, in this order:**
   - Robot Controller configuration names must equal `HardwareNames` (`front_left_drive`,
     `front_right_drive`, `back_left_drive`, `back_right_drive`, `intake`, `storage`, `transfer`,
     `shooter`, optional `storage_2` / `shooter_2`, `limelight`, `sensor_*`). Init telemetry lists
     anything missing; the robot still runs without it.
   - Run the Mecanum Tuner (AutoTune web UI) once and fix any reversed direction in
     `Constants.drivetrainConfig`.
   - Measure `Shooter.TICKS_PER_REV` / `SHOOT_RPM` for a shot into the up-CELL from the wall, and the
     `Storage` / `Transfer` pulse timings on a bench cycle (`Storage.ADVANCE_TIMEOUT_MS` dominates the
     per-piece time without sensors; four pieces plus spin-up must fit `SHOOT_BUDGET_MS`).
   - Set `AutoRoutine.LEAVE_DIRECTION` / `LEAVE_POWER` / `LEAVE_MS` for the real placement: the robot
     must clearly stop touching the wall and never reach the HIVE structure.
   - Run `Auto: shoot 4 + leave` once on the practice field with the 30 s clock, then `Teleop`.
2. **`SelfTest`** — per-subsystem checks; command → `writeActuators()` → settle → `readSensors()` → sample;
   `stopMechanisms()` + `writeActuators()` in the `finally`.
3. **`game/PieceType` (numbers only)** — measure the placeholder hue windows and saturation/value
   floors on real pieces; add the alliance-aware red-vs-blue NECTAR check for G408.
4. **`pedro/Constants` + `Tuning`** — `drivetrainConfig` is done; add `localizerConfig` and
   `foresightConfig` from AutoTune (docs/01 §A.7, worked example in
   `git show 20768b3:TeamCode/src/main/java/org/firstinspires/ftc/teamcode/pedro/Constants.java`)
   and make `create()` return the follower. Until then `Drivetrain.isAvailable()` is false and
   everything else still runs.
5. **The Pedro-path auto**, once tuned: replace `AutoRoutine.build()` with legs from the current pose
   (`Macros.driveTo` / `followLazyCommand` bounded with `Waits`), the opener
   `aimAndShootAll(Field.firstTargetCell(alliance), tags...)` from `FieldPoses.BLUE_SHOOTING_SPOT`
   (rotated for red), mid-path actions via `race(follow, waitUntil(...))`, and park via
   `driveTo(FieldPoses.BLUE_PARK)`; seed the start pose from `FieldPoses.startPose(start)`.
6. **SDK 12.0 upgrade** before any AprilTag-cluster work; then drop `tuning` for competition builds.

## 9. Constants to measure on the robot

| File | Constant | How |
|---|---|---|
| `Intake` | `MOTOR_FREE_SPEED_TICKS_PER_SEC`, `INTAKE_TICKS_PER_SEC`, `STALL_CURRENT_AMPS` | SelfTest: command and read back velocity; watch amps in a deliberate jam |
| `Storage`, `Transfer` | `*_TICKS_PER_SEC`, timeouts | bench cycle with pieces |
| `Shooter` | `TICKS_PER_REV`, `SHOOT_RPM`, `AT_SPEED_TOLERANCE_RPM` | encoder spec, then shots into the up-CELL (~53 in high, verify in the Onshape CAD) |
| `Shooter` | `HEADING_OFFSET_RAD` | which way the flywheel fires relative to the intake (π = out the rear); confirm on the built robot |
| `Limelight` | `CAMERA_YAW_OFFSET_DEGREES` | camera yaw relative to the robot's forward axis; the aim law combines it with a tag's tx |
| `Limelight` | `CAMERA_HEIGHT_INCHES`, `CAMERA_PITCH_DEGREES` (positive down), offsets | tape measure after the mount is final |
| `PositionalServo` | `TRAVEL_MS_PER_UNIT` | stopwatch a full sweep |
| `Drivetrain` | `HEADING_HOLD_P/I/D` (also the aim lock's gains) | on the field with the drivetrain tuned |
| `Macros` | `AIM_TOLERANCE_DEGREES`, the `*_TIMEOUT_MS` values | on the field; a shot that misses wide is the aim tolerance, a shot that never fires is a timeout |
| `pedro/Constants` | `drivetrainConfig` directions (Mecanum Tuner: each wheel drives the robot forward), then `PinpointConfig`, `ForesightConfig` | AutoTune web UI at `http://192.168.43.1:10158` |
| `AutoRoutine` | `LEAVE_DIRECTION`, `LEAVE_POWER`, `LEAVE_MS`, `SETTLE_MS`, `SHOOT_BUDGET_MS` | on the practice field: the robot must clearly stop touching the wall and never reach the HIVE structure |
| `game/PieceType` | hue windows, saturation/value floors | real pieces under match lighting |
| `game/Field` | `HIVE_LATERAL_OFFSET_INCHES`, `UP_CELL_OPENING_HEIGHT_INCHES`, `FLOWER_INSET_INCHES`, LOADING ZONE / GARDEN placement within their tiles | Onshape field CAD (docs/04 §10), then a tape measure at the first event |
| `game/FieldPoses` | every pose (start standoffs, `BLUE_SHOOTING_SPOT`, `BLUE_PARK`, `BLUE_GARDEN_APPROACH`) | drive the real field; correct the BLUE value in place and red follows |

## 10. Open hardware questions

- Motor count: two storage motors and two flywheels together would be nine; which one is single,
  or which pair shares a motor.
- Shooter firing direction (out the rear is assumed) and whether a rear-facing camera is fitted so
  tag tx can refine the aim; with a front camera the aim runs on odometry.
- Whether the storage side wheels must run while intaking (`Macros.INTAKE_RUNS_STORAGE`).
- Which way is "off the wall" for LEAVE given how the robot is placed (`AutoRoutine.LEAVE_DIRECTION`).
- Sensor type at each of the five reserved points (`HardwareNames.SENSOR_*`); colour sensor at the
  storage entrance is effectively mandatory for G408.
- Whether a Limelight is fitted in V1, and where (everything fails soft without it).
- Flywheel count, feed gate or transfer pulse.

## 11. References

- Specs: `~/Downloads/BIOBUZZ_V1_Robot_Physical_Architecture.md` (authoritative, except that V1 has
  no turret), `~/Downloads/BIOBUZZ_Robot_Physical_Architecture.md` (final/V2 robot, not in scope).
- Reference codebase: `~/Documents/FTC_Guide` (Pedro 2.1.2 / Ivy 1.0.0 / Panels; docs in `docs/`).
- Competition Manual V1 (HTML): https://ftc-resources.firstinspires.org/ftc/game/cm-html ;
  Team Updates: https://ftc-resources.firstinspires.org/ftc/game/tu-00 ; Q&A opens 2026-09-28.
- SDK v12.0: https://github.com/FIRST-Tech-Challenge/FtcRobotController/releases/tag/v12.0
- Pedro: https://pedropathing.com ; Ivy: https://github.com/Pedro-Pathing/Ivy ;
  Maven repo `https://repo.dairy.foundation/releases/`.
- Library sources used for the docs live in the Gradle cache under
  `~/.gradle/caches/modules-2/files-2.1/com.pedropathing*/` (revhub/tuning sources jars; core is bytecode only).
- Claude Code session memory for this project:
  `~/.claude/projects/-Users-amitgupta-Documents-25702-Biobuzz-code/memory/`.
