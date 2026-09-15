# HANDOFF — FTC Team 25702 BIOBUZZ Robot Code

State as of **2026-09-15**. Read this first; it tells you what exists, what does not, why
things are the way they are, and what to do next. Everything here is backed by a file in the repo.

---

## 1. What this is

- **Team 25702**, FTC 2026-27 game **BIOBUZZ presented by RTX** (kickoff 2026-09-12).
- Code targets the **V1 robot** (`docs/specs/BIOBUZZ_V1_Robot_Physical_Architecture.md`, with one
  correction from the team): mecanum drivetrain, front funnel, 16 mm compliant intake roller, short
  ramp, low horizontal **4-piece** storage on Gecko side wheels, rear 90° vertical Gecko transfer,
  and a compliant flywheel shooter **fixed to the chassis, firing out the rear**. **No turret** (the
  spec's turret was dropped; the drivetrain aims), **no Flower mechanism, extension or diverter**
  (V2; `docs/specs/BIOBUZZ_V2_Robot_Physical_Architecture.md`, no code).
- Stack: **Pedro Pathing 3.0.0** (`com.pedropathing:revhub:3.0.0` + `core:3.0.0`), **Ivy 1.1.1**
  command scheduler (`com.pedropathing.ivy:pedro:1.1.1`), **AutoTune** (`tuning:1.0.0`, tuning
  builds only), FTC SDK **11.2.1** (inherited from the Pedro Quickstart this repo was cloned from),
  Gradle 9.1.0, AGP 8.13.2, Java 8 source level. **No Panels / FTC Dashboard.**
- Architecture: `Robot` / `subsystems` / `commands` / `opmodes` / `game` / `util`, every subsystem an
  Ivy resource with its own command factories, every loop `read → decide → Scheduler.execute() →
  write`, JVM unit tests against fakes, and the real OpMode lifecycle testable on the JVM. It is the
  FTC_Guide reference architecture (a separate repo, Pedro 2.1.2 / Ivy 1.0.0 / Panels) re-based on
  Pedro 3 and Ivy 1.1.1.

## 2. State of the build

| Layer | Status | Tests |
|---|---|---|
| `util/**` (18 classes) | **implemented**; `MatchLogger` prunes to `KEEP_NEWEST`; `JamDetector` forgives attempts only after healthy current | 135 |
| `subsystems/**` (10 classes + 4 templates) | **implemented**; `OpenLoopDrive` is the untuned-drivetrain layer; `ColorSensor` caches distance; `Shooter` has a latched at-speed and optional PIDF; `Intake` has the G408 reject scaffold | 92 |
| `Robot.java` | **implemented**: composition, supplier wiring and the sensor-trust decision (every point by presence, distance first; the entrance trusted by distance or measured hues; the reject only with measured hues), `sensingSummary()`, presence sensors read in rotation, loop halves, localization write-back only on `ACCEPTED`, `logCells` | 18 |
| `commands/Macros.java`, `commands/Waits.java` | **implemented**: every macro through `reporting()` (never left RUNNING); metered sensorless feed; blind shooting when the count is unknowable (`ASSUME_FULL_WHEN_UNCOUNTED`); a timed intake on a robot that cannot detect full is SUCCESS; aim bounded; recovery only between pieces | 38 + 2 |
| `game/Field.java`, `Scoring.java`, `FieldPoses.java`, `PieceType.java` | **implemented**; `PieceType` hue windows are placeholders, `nectarAllianceAt` for G408 | 18 (+ via ColorSensorTest) |
| `opmodes/MatchOpMode.java`, `RobotTunables.java` | **implemented**: extends `OpMode`, final lifecycle, `buildRobot()` / `openLogger()` seams; snapshots the tunables first in `init()`; tuning-build and tuned-this-session warnings on the init card | via `TeleopTest` |
| `opmodes/teleop/Teleop.java`, `Controls.java` | **implemented**: `@TeleOp "Teleop"`; one drive default (Pedro or open loop); count, pose and alliance from auto; operator count buttons; drive and camera macros gated on each control's declared `Needs` | 26 + 6 |
| `opmodes/auto/MainAuto.java`, `AutoRoutine.java`, `AutoSelector.java` | **implemented**: `@Autonomous "Auto: shoot 4 + leave"`, hardcoded and open-loop for the first event; hands alliance and piece count to teleop; warns loudly when started unlocked | 6 + 7 + 3 |
| `opmodes/test/*` (`BenchOpMode` + 6 benches + `SelfTest`) | **implemented**: pit tools that measure §9's constants on the real robot; group "Bench" on the Driver Station; every footer lists tunables changed this session (BACK restores them) | 9 |
| `pedro/Constants.java`, `pedro/Tuning.java` | `drivetrainConfig` filled (names + directions); `localizerConfig` / `foresightConfig` `null` until AutoTune, so `create()` returns `null` and `Drivetrain` fails soft; four `@Tuner` factories registered (tuning build only) | — |
| `pedro/procedures/*` | Quickstart AutoTune procedures, untouched, tuning build only | — |
| `docs/01..04`, `docs/specs/*` | written and kept current | — |

`./gradlew :TeamCode:testDebugUnitTest` → **368 tests, 0 failures**. `:TeamCode:assembleDebug`
builds the competition APK (no AutoTune) with `Teleop`, `Auto: shoot 4 + leave`, six `Bench: …`
OpModes and `SelfTest` registered.

**What was done on 2026-09-14** (each step committed green; the review that drove it is
`fixthese.md`, and §8 has the item-by-item table):
1. Baseline commit of the 2026-09-12/13 work; `.claude/` ignored.
2. Teleop drives before Pedro is tuned: `Robot` builds exactly one motor layer, `Teleop` schedules
   exactly one drive default, drive macros refuse politely without a follower (A1, B5).
3. AutoTune registered (`Tuning.java` factories, `NotReady` for unfilled configs) and kept out of
   competition builds behind `-Ptuning` (A2, A4, D7, D12).
4. Heading hold works on the robot: the pose is written back only on an accepted fix; blue's
   driver frame; no endgame phase; Pinpoint settle handled; dpad overrides the alliance (B1, B3, B4,
   B6, B7, C12).
5. The shooting cycle: metered sensorless feed with storage and transfer running together, every
   macro reports a terminal outcome, only the hold owns the shooter, intake no longer runs the
   storage (B2, C6, C7, C8, C9, D1).
6. The shooting cycle on the real robot (found reviewing the plan, §8 G): blind shooting when the
   count is unknowable, auto→teleop piece count, bounded aim, a 5th piece, no dead recovery wait,
   heading hold yields to small turns.
7. Sensors and guards: presence by distance, G408 reject scaffold (off), flywheel PIDF + latched
   at-speed, stall numbers, jam attempts forgiven only after healthy current, second-motor
   directions, camera macros and botpose gated (C1–C5, C10, C11).
8. `opmodes/test/`: the benches and `SelfTest`; the empty `SelfTest` / `ConceptCommands` stubs removed (D5).
9. This document, `fixthese.md`'s status, docs, log pruning (D2, D3, D4, D8).

**What was done on 2026-09-15** (Round 2 of `fixthese.md`: a review of the fixed code; §8.1 has the table):
1. Sensors are trusted because they were measured, not because they are in the configuration: the
   entrance counts by distance, a hue-only entrance sensor waits for `PieceType.HUES_CALIBRATED`,
   `Storage` knows whether a full sensor exists, a timed intake is a success, the operator can set
   the count on the dpad, and every init card prints what each sensor point is doing (R2-A2, A3, A4,
   A10, B1, B2, B6).
2. Loop budget: one intake current read per loop, velocity writes only on change, presence sensors
   read in rotation, loop stats on the auto card (R2-A1).
3. Pit edits are visible: tunables changed on a bench are listed on every card (BACK restores them on
   a bench), a `-Ptuning` APK announces itself, custom PIDF off really restores the SDK's, and the
   second motor's direction applies live (R2-A5, A6, A7, A9).
4. Match-day guards: an unlocked auto warns but runs, controls declare what they need and Teleop
   gates generically, and the final-seconds rumble feels different from "full" (R2-A8, B4).

## 3. Read these first

| File | Why |
|---|---|
| `docs/02-robot-physical-architecture.md` | what the V1 robot is and which subsystem owns each mechanism |
| `docs/03-software-architecture.md` | the loop contract, package map, per-subsystem API, Limelight and colour-sensor specs, lessons-learned rules, and §20's practical notes |
| `docs/01-libraries-pedro-3.0.0-ivy-1.1.1.md` | the two libraries' real APIs, verified from bytecode: read before touching `Drivetrain`, `Macros` or `AutoRoutine` |
| `docs/04-biobuzz-season-analysis.md` | sourced game/field/rule facts, and §9 "implications for our robot" |
| `fixthese.md` | the 2026-09-14 review, with the status of every item at the top |
| `opmodes/teleop/Controls.java` | every binding and the help card; change a button there and nowhere else |
| `opmodes/test/*Bench.java` | how each constant in §9 gets measured |
| `game/Field.java` Javadoc | the field frame (origin corner, axes, heading) every pose and path depends on |
| `opmodes/auto/AutoRoutine.java` | the first-competition auto and its five tunables (`LEAVE_*`, `SETTLE_MS`, `SHOOT_BUDGET_MS`) |
| `util/hardware/HardwareNames.java` | the exact strings the Robot Controller configuration must use |

## 4. Build and test

```bash
# Gradle 9.1 may reject a newer system JDK; the Android Studio JBR always works.
JAVA_HOME="/Applications/Android Studio.app/Contents/jbr/Contents/Home" \
  ./gradlew :TeamCode:compileDebugJavaWithJavac :TeamCode:testDebugUnitTest --console=plain

# Competition APK (no AutoTune; R704):
./gradlew :TeamCode:assembleDebug
# Tuning APK (AutoTune web UI at http://192.168.43.1:10158; never at a match):
./gradlew -Ptuning :TeamCode:assembleDebug

# Results: TeamCode/build/test-results/testDebugUnitTest/*.xml
# Deploy: the green Run button in Android Studio deploys whichever variant you last built.
```

Expect `BUILD SUCCESSFUL` plus javac warnings about Java 8 source level (harmless).

## 5. Repo map

`TeamCode/src/main/java/org/firstinspires/ftc/teamcode/`

```
Robot.java                     impl  composition root: readSensors()/writeActuators(), supplier wiring, alliance, logCells()
opmodes/
  MatchOpMode.java             impl  abstract base (extends OpMode); final lifecycle read→decide→execute→write;
                                     buildRobot()/openLogger() seams let tests run the real lifecycle on the JVM;
                                     init card warns on a tuning build and lists tunables changed this session
  RobotTunables.java           impl  the classes whose public statics are tunables (for util/diagnostics/Tunables)
  teleop/
    Controls.java              impl  enum of every gamepad binding + help card + what each needs (Needs);
                                     read() takes one Snapshot per loop
    Teleop.java                impl  @TeleOp "Teleop": one drive default (Pedro or open loop), macros, aim lock on
                                     R-trigger, flywheel toggle, haptics, telemetry; count/pose/alliance from auto
  auto/
    MainAuto.java              impl  @Autonomous "Auto: shoot 4 + leave": selector in init, routine on start,
                                     PoseStorage every loop (alliance, start, pose, piece count), buzzer safety net
    AutoRoutine.java           impl  hardcoded first-competition routine: shootAll, settle, timed leave (JVM-tested)
    AutoSelector.java          impl  dpad alliance / start-position menu, A locks, B unlocks (START runs regardless)
  test/
    BenchOpMode.java           impl  base for the pit benches: same Robot, no Ivy, readSensors → onBench → writeActuators;
                                     footer shows loop, sensing summary, tunables changed; BACK restores defaults
    IntakeBench.java           impl  "Bench: Intake": free speed, stall amps, anti-jam toggle, reject readout
    StorageBench.java          impl  "Bench: Storage": count, sensors, SENSORLESS_FEED_PULSE_MS pulse, edge sampling
    TransferBench.java         impl  "Bench: Transfer": lift/feed, the same pulse
    ShooterBench.java          impl  "Bench: Shooter": ticks/rev, rpm, spin-up, shot dip + recovery, PIDF toggle
                                     (shows what the hub holds), live second-motor direction flip
    ColorSensorBench.java      impl  "Bench: Color sensors": HSV, hue windows, NECTAR alliance, presence distance
    LimelightBench.java        impl  "Bench: Limelight": pipelines, tags, blob, mount check
    SelfTest.java              impl  "SelfTest": PASS / FAIL / SKIP per subsystem; run first at every event
subsystems/
  PathFollower.java            impl  interface: the slice of Pedro 3 Follower the Drivetrain calls
  PedroPathFollower.java       impl  production adapter; raw() exposes the Follower
  Drivetrain.java              impl  manual/field-centric drive with a driver-frame offset, heading hold (+ aim-lock
                                     setpoint), followLazy/turnTo/hold/driveForMs commands, Pinpoint settle
  OpenLoopDrive.java           impl  open-loop driving through Pedro's Mecanum (names + directions only); built only
                                     when there is no follower; writes on intent change only
  Intake.java                  impl  velocity roller, anti-jam, full-storage interlock, capture command, G408 reject (off)
  Storage.java                 impl  edge-counted 4-piece queue; has{Entrance,Exit,Full}Sensor, canDetectFull; advance commands
  Transfer.java                impl  liftOne / feed (sensor-or-pulse) and feedForMs commands
  Shooter.java                 impl  RPM flywheel; latched atSpeed; waitForSpeed / holdSpeed / idle; optional PIDF;
                                     fixed, fires out the rear (HEADING_OFFSET_RAD)
  Limelight.java               impl  AprilTag tx aiming (getTagTx), colour blobs, gated botpose (null this season)
  ColorSensor.java             impl  NormalizedColorSensor wrapper, one per sensor point; caches colour + distance
  templates/                   impl  Mechanism<S>, PositionalMotor<S>, PositionalServo<S>, VelocityMotor
commands/Macros.java           impl  bounded, always-reporting multi-subsystem macros (Ivy groups + Pedro paths); aimHeading law
commands/Waits.java            impl  Clock-based waitMs / bounded (Ivy's own waitMs is wall-clock)
game/Field.java                impl  field frame (origin A1), tiles, HIVE/CELLs, tag IDs, FLOWERs, zones, element counts
game/Scoring.java              impl  point values and ranking-point thresholds
game/FieldPoses.java           impl  BLUE poses (two starts, shooting spot, park, garden approach); RED via FieldConstants.forAlliance
game/PieceType.java            part  POLLEN / NECTAR sizes, target height, placeholder hue windows, nectarAllianceAt
pedro/Constants.java           part  drivetrainConfig filled; localizerConfig / foresightConfig null until AutoTune
pedro/Tuning.java              impl  @Tuner factories (tuning build only)
pedro/procedures/*.java        impl  Quickstart tuners (tuning build only)
util/
  control/JamDetector          impl  current-based stall detection, healthy-current window before attempts reset
  diagnostics/LoopTimer, MatchLogger (KEEP_NEWEST prune), RateLimiter, Tunables (snapshot / changed / restore),
             BuildFlavor (is AutoTune in this APK?)
  field/Alliance, FieldConstants (SYMMETRY = ROTATE_180), PoseFusion, PoseStorage (+ piece count), StartPosition
  hardware/Hardware (fail-soft lookup), HardwareNames (every config name, V1 + 5 sensor points)
  math/Angles, ColorMath, DriveScaling, MedianFilter, VisionMath
  time/Clock, SystemClock, MatchClock (BIOBUZZ timing, no endgame, isFlowerUnlocked at 1:00)
```

Tests live in `TeamCode/src/test/java/.../teamcode/` mirroring the tree; fakes are
`subsystems/FakePathFollower` (mirrors Pedro 3's state machine), `subsystems/FakeDcMotorEx` (records
velocity/power/direction/PIDF writes), `subsystems/FakePedroDrivetrain` (Pedro's motor-layer
interface), `subsystems/FakeColorRangeSensor` (a REV V3: colour + distance), `util/time/FakeClock`
and `opmodes/FakeTelemetry`. Every test does `Scheduler.reset()` in `@Before` and `@After`.
`RobotTest` assembles a whole robot from the fakes; `TeleopTest`, `MainAutoTest` and
`BenchOpModesTest` run the real OpModes' lifecycle (`init` → `init_loop` → `start` → `loop`) on the
JVM with real `Gamepad`s driven by `copy()`; `AutoRoutineTest` runs the whole auto in fake time.

**Teleop at a glance** (the source of truth is `Controls`; the init screen shows this card):

| Pad | Input | Does |
|---|---|---|
| driver | L-stick / R-stick X | drive (field-centric with the alliance's driver frame and a heading hold when Pedro is tuned; robot-centric open loop before that) |
| driver | L-trigger (hold) | precision slow mode |
| driver | **R-trigger (hold)** | **aim lock**: heading hold points the shooter at the current up-CELL while the sticks still translate (Pedro tuned only) |
| driver | LB / Y / BACK | field ↔ robot centric / re-zero heading (face away from your wall) / abort macro (moving a stick also aborts a drive macro) |
| driver | A / X | collect a piece / turn to face a piece (camera; refused until `Limelight.MOUNT_CALIBRATED`) |
| driver | B / RB | Pedro path to the shooting spot / to park (Pedro tuned only) |
| driver | dpad | snap to 90 / 0 / 270 / 180° (Pedro tuned only) |
| operator | RB / LB / B / X | intake / outtake / eject / stop intake (X also cancels any macro) |
| operator | Y | intake until the storage is full; with no entrance or full sensor it is a timed run (`intake (timed)`, SUCCESS when the timer ends) |
| operator | R-trigger / A | shoot one / shoot all. **Without a trusted entrance sensor the count is unknown: shoot one = one pulse, shoot all = 4 pulses (X stops early)** |
| operator | dpad up / dpad left | **count = 4** (holds the intake roller, shoot all fires exactly four) / count = 0, back to unknown. The sensorless robot's entrance sensor is the operator |
| operator | L-trigger | flywheel armed on/off (comes back by itself after a shot macro) |
| operator | dpad down | "our HIVE tipped": the aim target flips to the other CELL |
| operator | BACK | debug telemetry |

**Auto at a glance** (`Auto: shoot 4 + leave`, hardcoded for the first event, no Pedro tuning; it
pre-selects `Teleop` on the Driver Station): place the robot touching its wall with the **rear
(shooter) toward the up-facing CELL**, pick the alliance on gamepad 1's dpad during init (A locks the
menu; START runs whatever is shown, and the card blinks `NOT LOCKED` until A is pressed). On start: `storage.setCount(4)` → `shootAll()` (one spin-up,
four metered storage+transfer pulses with the flywheel recovering between them, `SHOOT_BUDGET_MS`
20 s cap) → `SETTLE_MS` → drive `LEAVE_DIRECTION` (default BACKWARD, away from the wall) at
`LEAVE_POWER` for `LEAVE_MS` → stop. Alliance, start and the remaining piece count go to teleop
through `PoseStorage`; there is no pose until Pedro is tuned. At the 30 s buzzer anything still
running is cancelled and every mechanism stopped (G403).

## 6. Decisions already made, and why

| Decision | Why |
|---|---|
| V1 scope only | The team's V1 spec; the final-robot spec was used by mistake for half a day and everything Flower/diverter-related was removed. |
| No turret on V1: the shooter is fixed and fires out the rear; aiming is the drivetrain heading | Team correction on 2026-09-13. A standing aim cannot be an Ivy command (a never-ending priority-0 drivetrain command would suspend driver control for good), so it is an aim-lock setpoint inside `Drivetrain`'s heading hold, fed by `Macros.aimHeading` (bearing − `Shooter.HEADING_OFFSET_RAD`, tag-refined). `aimAt` is the one-shot turn for auto. |
| `FieldConstants.SYMMETRY = ROTATE_180` | The BIOBUZZ field is 180° rotationally symmetric (GARDEN A1/F6, LOADING ZONE A5/F2). A mirror puts every blue auto in the wrong place. |
| Field frame origin = A1 corner (audience-side red), +X toward column F, +Y toward row 6 | Pinned by the 180° rotation mapping A5→F2 and A1→F6; written once in `Field`'s Javadoc, checked by `FieldTest`. |
| Driver frame per alliance (`Drivetrain.setDriverHeadingOffset`, `Field.driverForwardHeading`) | +X points at the blue driver, so field-centric "forward" for blue is −X. Aim lock and macros stay in the true frame. Y re-zeroes to the driver's forward. |
| No AprilTag localisation; tags are aiming targets | Every tag rides on a moving HIVE CELL; the SDK v12 notes say they are unsuitable for localisation. `PoseFusion` stays for a future static reference; `Limelight.getBotposeAsPedroPose()` is gated to null until `BOTPOSE_FRAME_VERIFIED`. |
| Pose write-back only on `PoseFusion.Result.ACCEPTED` | Writing the fused pose every loop released the heading hold every loop (it never corrected anything) and re-wrote the Pinpoint over I2C for nothing. |
| One motor layer at a time | `Robot` builds `OpenLoopDrive` on the real motors only when `Constants.create()` returns null; two Pedro `Mecanum` objects can never share the four drive motors. `Teleop` schedules exactly one drive default. |
| `PathFollower` seam over Pedro | Every drivetrain command runs on the JVM against `FakePathFollower`; Pedro's behaviour changes are caught by `DrivetrainCommandTest`, not on the field. |
| Keep `Drivetrain.followLazyCommand`, never bare `PedroCommands.follow` | Ivy 1.1.1's `PedroCommands.follow` has no requirement and no `setEnd`; an interrupted follow would keep driving. Ours holds at `path.endPose()` on a natural end with `holdEnd` and hands back with `manual(0,0,0)` otherwise. |
| `VelocityMotor` template | Four mechanisms are velocity rollers/flywheels; intent-then-write, ticks/rev and the PIDF hook live in one place. |
| Default commands: priority −1, `SUSPEND`, `BlockedBehavior.QUEUE`, logic in `setExecute` | Ivy resumes a suspended command without calling `start()`; `CANCEL` would drop it for the whole match. |
| Storage hard-stops the intake at 4 | BIOBUZZ G407: at most 4 controlled scoring elements. `Intake.setFullSupplier(storage::isFull)`. |
| Every sensor point judges presence by distance when the device has it; hue only classifies (G408), and only once `PieceType.HUES_CALIBRATED` | A hue window nobody has measured fails silently to "no piece", which the interlocks read as "keep going" and the count reads as "nothing ever entered". A REV V3 proximity read does not care about colour or lighting. |
| A sensor is trusted when measured, not when configured; `Robot.wireSuppliers()` is the only place that decides | Plugging in an entrance sensor with unmeasured hues used to turn a robot that shot blind into one that reported NO_TARGET (fixthese R2-A3). `Robot.sensingSummary()` prints the decision on every card. |
| A timed intake on a robot that cannot detect full is SUCCESS, named `intake (timed)` | Eight seconds of intake followed by three failure blips reads as "broken" to an operator; the timer ending is the whole job. |
| Operator dpad up / left set the count to 4 / 0 | On the sensorless robot the operator is the only entrance sensor; count 4 is what holds the roller (G407) and makes shoot all fire exactly four. |
| Bench tunable edits are shown on every card, not restored on stop | Tune on the bench, then try it on the practice field, is the pit workflow; the danger was only that nothing showed the diverged value. BACK on a bench restores. |
| The unlocked auto still runs | Refusing to run when someone forgets A costs more than the default route; this routine is alliance-safe. The path auto must drive only when `isConfirmed()`. |
| `VelocityMotor.write` sends only on change, refreshed every 12 loops; presence sensors read in rotation | Six unchanged `setVelocity` calls and eight colour I2C reads per loop were the avoidable cost; the refresh recovers a hub that reset mid-match. |
| Every macro is built through `Macros.reporting()` = `deadline(sequential(begin, body, finish), onInterrupt(markCancelled))` | A `Waits.bounded` timeout or an abort used to leave the outcome RUNNING for good. Ivy groups are `CommandBuilder`s whose `setEnd` replaces the group's own end, so the callback has to be a child, not a `setEnd`. |
| Sensorless feed = storage and transfer run together for `SENSORLESS_FEED_PULSE_MS` | The old cycle ran the storage for 2.5 s into a stopped transfer per shot (a jam machine) and "lift" then "feed" were the same motor at two speeds. |
| Only `holdSpeedCommand` requires the shooter; the spin-up wait (`waitForSpeedCommand`) requires nothing | Two siblings requiring the same resource inside a group both set the target and the last-executed one silently wins. |
| `Macros.INTAKE_RUNS_STORAGE = false` | With the transfer stopped, the side wheels would push every stored piece against it for the whole intake; `Storage` has no jam detection. |
| Without an entrance sensor a zero count is "unknown": shoot one fires one pulse, shoot all fires `CAPACITY` | The count only rises on that sensor's edge or through `setCount`; a robot that refuses to shoot because it cannot count is useless at an event. |
| `PoseStorage` carries the piece count from auto to teleop | A cut auto that shot two of four must not start teleop believing the robot is empty (G407's interlock would be two pieces off); a sensorless robot has no other way to know. |
| Flywheel recovery is waited for only between pieces; a 150 ms dwell after the last | Up to 1.65 s of held resources doing nothing after every `shootOne` and at the end of every `shootAll`. |
| `Shooter.atSpeed()` is latched over `AT_SPEED_LOOPS` consecutive in-band loops, tolerance 5 % | One noisy sample through a ±47 t/s band released shots and the old band never held long enough to latch. |
| `Shooter.CUSTOM_PIDF = false`, `Intake.REJECT_ENABLED = false`, `Limelight.MOUNT_CALIBRATED = false` | Each needs a measurement before it can be right; off is safer than a guess, and the benches are how they get measured. |
| `HEADING_HOLD_STICK_DEADBAND = 0.001` | The stick arrives shaped; 0.05 against a shaped value swallowed raw deflections up to ~0.28 and fought the driver's small corrections. |
| Transfer/lift commands fall back to timed pulses without a feed sensor | So the robot cycles on a bench before sensors are fitted; wire `setAtFeedSupplier` to turn on the interlocks. |
| `MatchLogger` knows nothing about `Robot`; keeps `KEEP_NEWEST` files | The logger takes a header and cells; `Robot.logCells(loopMs)` supplies the row. A season of practice is thousands of files otherwise. |
| No Panels, no dashboard; AutoTune behind `-Ptuning` | R704 prohibits streaming tools during matches and AutoTune's servers are always bound while it is on the classpath. |
| Macro timeouts on the injected `Clock` (`commands/Waits`) | Ivy's `waitMs` is wall-clock; `MacrosTest` runs every macro deterministically on `FakeClock`, no sleeps. |
| `shootOne` / `shootAll` never require the drivetrain | Driving and the aim lock continue through a shot; autonomous aims with `aimAt` / `aimAndShootAll` (aim bounded by `AIM_TIMEOUT_MS`, then it shoots anyway). |
| Teleop reads one `Controls.Snapshot` per loop | SDK `*WasPressed()` consumes on read; a press during a macro must be consumed, not latched to fire when the macro ends. |
| Sticks abort only a macro that owns the drivetrain; operator X cancels any macro | A driver grabbing the sticks must not cancel a shot in progress. |
| The flywheel hold is a standing operator wish, re-scheduled after any macro | Ivy ends (does not suspend) a preempted priority-0 command; `restoreFlywheelHold()` brings it back. |
| `MatchOpMode.buildRobot()` / `openLogger()` protected seams; `BenchOpMode.buildRobot()` | The only way to run the real OpMode lifecycle on the JVM (`HardwareMap` and the SD-card logger are Android-only). |
| First-competition auto is hardcoded and open-loop | No AutoTune before the first event means no localizer, no paths and no aiming; motor names + directions are all `Mecanum` needs, and the same `drivetrainConfig` feeds the tuned follower later. |
| `shootAll` is unrolled into `CAPACITY + 1` guarded steps, not Ivy's `repeat` | A `sequential` interrupted before reaching a `Repeat` calls `end()` on it and NPEs; the +1 is a piece already in the lift. |
| Benches drive subsystems directly, no Ivy | Nothing else is scheduled, so nothing re-asserts idle; a bench that stops calling a mechanism calls its `stop()`. The match code path (config names, directions, velocity, sensors) is still the one exercised. |

## 7. Rules that bite

**Pedro 3.0.0** (docs/01 §A)
- `new Follower(Localizer, Drivetrain, Algorithm)`; the Quickstart comment in `pedro/Constants.java` has the order wrong.
- Exactly one `follower.update()` per loop; it ticks the localizer itself.
- `atParametricEnd()` means the path is done, **and is also true whenever the follower is not following**: anything that puts the follower in HOLD or MANUAL mid-path makes `followLazyCommand` report "arrived". Keep paths and turns sequential, never parallel.
- `isBusy()` only clears inside a hold; never use it as "still following".
- `PinpointLocalizer`'s constructor starts an IMU calibration: build in `init()`; `Drivetrain` re-applies the init pose once `LOCALIZER_SETTLE_MS` has passed.
- Paths need a heading (`.constant/.linear/.tangent`) or `follow()` throws; `followLazyCommand` catches and records it.
- No callbacks, no `setMaxPower`, no `turnTo`: poll, use Ivy, or `path.with(cfg.maxPathSpeed.at(p))`. A turn is `hold(pose.withHeading(h))`.
- `@Tuner` factories run at RC start-up (`TunerScanner`): static, zero-arg, return `Procedure`, never throw (`Tuning.NotReady`).

**Ivy 1.1.1** (docs/01 §B)
- `Command.unless()` never finishes; use `conditional(cond, real, instant(() -> {}))`.
- `Commands.lazy(...)` contributes no requirements; always `.requiring(subsystem)`.
- `waitMs` is wall clock and not fakeable; use `Waits.waitMs(clock, ms)` / `Waits.bounded(...)`.
- A direct hardware call from an `instant` without a requirement is overwritten by the default command before the write.
- `Scheduler` is static: `Scheduler.reset()` first thing in `init()` and in every test.
- `deadline` and `parallel` end their unfinished children with the **group's own** end condition, so a child's `ec == NATURALLY` branch fires when the group finishes naturally. Judge by the sensor or condition, never by `EndCondition`, or use `race`.
- A `sequential` hands off one child per `execute()`: a child's `start()` never runs at `schedule()` time, and every hand-off costs a 20 ms loop (a macro with a dozen groups is ~0.5 s of pure hand-offs).
- **Never put `repeat(...)` inside a group that can be interrupted before reaching it**: `Repeat.end()` NPEs. Unroll into guarded `conditional` steps (`Macros.shootAllCore`).
- **`setEnd` on a group replaces the group's own end**, the one that ends its children. Run interrupt work as a child: `deadline(body, onInterrupt(cb))` (`Macros.reporting`).

**SDK gamepads and OpModes** (bytecode-verified on 11.2.1)
- `*WasPressed()` consumes its flag on read and only advances inside `Gamepad.copy()` / `fromByteArray()`. Read every edge once per loop (`Controls.read`), never twice; `resetEdgeDetection()` in `start()`.
- `new Gamepad()` and an `OpMode` subclass construct on the JVM (no Android in their constructors); `Telemetry` is an interface. `HardwareMap` is Android-only: keep it behind `buildRobot()`. In a test's inner OpMode subclass, `robot` resolves to the inherited field, not the fixture: write `Outer.this.robot`.
- Iterative OpModes may not write motors from `stop()`; the SDK zeroes them itself when the OpMode ends.
- The SDK's **TestHardware** utility OpMode (Utility menu, 11.2+) spins any motor and shows any colour/distance sensor by config name: the first check at every event, before any bench.

**Game and robot rules** (docs/04)
- G407 max 4 pieces; G408 never control opponent NECTAR (colour sensing at the storage entrance; `Intake.REJECT_ENABLED` once the hues are measured).
- R503 **8 motors and 8 servos max**: V1 needs 8–10 (drive 4, intake, storage 1–2, transfer, shooter 1–2). A single flywheel and a single storage motor fit exactly; otherwise storage and transfer share a motor. `HardwareNames` keeps the optional second-motor names; `Shooter.SECOND_MOTOR_DIRECTION` / `Storage.SECOND_MOTOR_DIRECTION` for an opposed pair.
- R704: no dashboard/streaming tools in matches. **The competition APK is plain `assembleDebug`; never deploy a `-Ptuning` build at an event.** A tuning APK now says so as the first line of every match init card.
- Season SDK is **v12.0** (AprilTag cluster API); this repo is on 11.2.1.
- AUTO 30 s → 8 s no-motion transition (G403: do not INIT a twitchy TeleOp early) → TELEOP 120 s; no endgame; FLOWER unlock at 1:00.

**The robot** (docs/03 §20 has the long form)
- Camera facing decides whether tag-aiming exists: a front camera only sees the tags while the robot is *not* aimed, so `aimHeading` runs on odometry; a rear camera needs `CAMERA_YAW_OFFSET_DEGREES = 180`.
- `Macros.SENSORLESS_FEED_PULSE_MS` is the pulse the robot uses; `Transfer.*_PULSE_MS` only apply to direct `liftOne/feed` commands.
- Shot counts and `SUCCESS` are pulse counts without sensors.
- Colour sensors are I2C outside the bulk read. The entrance is read every loop; the fitted presence sensors (full, transfer, feed) one per loop in rotation, so with three fitted each is up to ~60 ms old. Watch `Loop` on any bench footer.
- **Tunables are `public static` and live until the Robot Controller app restarts.** A value changed on a bench is what Teleop runs with; the init card lists every such value (`!! TUNED THIS SESSION`). BACK on a bench restores them all.
- `SECOND_MOTOR_DIRECTION` applies live; `PieceType.HUES_CALIBRATED` and `Limelight.MOUNT_CALIBRATED` are read when the robot is built, so change them and re-INIT.
- Entrance edge counting samples at loop rate; `Bench: Storage` shows whether a passing piece is in view for two or more loops.

## 8. Next steps, in order

### 8.1 What `fixthese.md` asked for, and where each item went

| Item | Change | Proof |
|---|---|---|
| A1, B5 | one motor layer (`Robot` ctor), one drive default (`Teleop.onInit`), open-loop `driveForMsCommand` | `TeleopTest.stickForwardDrivesOpenLoopWhenPedroIsNotTuned`, `OpenLoopDriveTest`, `AutoRoutineTest` |
| A2, D7, D12 | `Tuning.java` four `@Tuner` factories + `NotReady`; `Constants.localizerConfig/foresightConfig` placeholders with `HardwareNames.PINPOINT` | compiles under `-Ptuning` |
| A3, D11 | baseline commit; `.claude/` ignored | git log |
| A4 | `tuning` dependency and `pedro/Tuning.java`, `pedro/procedures/**` only with `-Ptuning` | both builds succeed |
| B1 | `Robot.updateLocalization` writes back only on `ACCEPTED` | `RobotTest.headingHoldCorrectsThroughTheFullLoop`, `updateLocalizationLeavesTheFollowerAloneWithoutAFix` |
| B2, C9 | metered sensorless feed: storage + transfer together for `SENSORLESS_FEED_PULSE_MS`, recovery between pieces | `MacrosTest.sensorlessShootAllOfFourFinishesInsideThirteenSeconds`, `AutoRoutineTest.storageNeverPushesIntoAStoppedTransferWithoutSensors` |
| B3 | `Drivetrain.setDriverHeadingOffset`, `Field.driverForwardHeading`, `resetHeading()` to the driver's forward | `DrivetrainCommandTest`, `TeleopTest.redForwardIsFieldPlusX` |
| B4 | dpad always flips the alliance in `Teleop.onInitLoop` | `TeleopTest.dpadOverridesTheAllianceLeftByAuto` |
| B6 | `MatchClock.forTeleop()` has no endgame | `MatchClockTest.teleopClockHasNoEndgameInBiobuzz` |
| B7 | `Drivetrain.LOCALIZER_SETTLE_MS`, `onStart()`, deferred pose re-apply; `MainAuto.onStart` seeds the start pose | `MainAutoTest.startSeedsTheStartPoseForTheSelectedAlliance` |
| C1 | `ColorSensor.getDistanceInches()` cached per loop; `Robot.pieceNear` for the three presence points; `PRESENCE_DISTANCE_INCHES` | `ColorSensorTest`, `RobotTest.presenceUsesDistanceWhenTheSensorHasIt` |
| C2 | `PieceType.nectarAllianceAt`, `Robot.setAlliance`, `Intake.setRejectSupplier` / `REJECT_ENABLED = false` / `REJECT_MS` | `IntakeCommandTest.opponentPieceAtTheEntranceIsThrownBackOutForRejectMs`, `RobotTest.opponentNectarTriggersRejectOnlyForTheOtherAlliance` |
| C3 | `Shooter.AT_SPEED_TOLERANCE_RPM = 150`, `AT_SPEED_LOOPS = 5`, `CUSTOM_PIDF` + `PIDF_*` + `applyPidf()`, `VelocityMotor.setVelocityPidf` | `ShooterTest.atSpeedNeedsConsecutiveInBandLoops`, `customPidfIsWrittenOnlyWhenEnabled` |
| C4 | `Intake.STALL_CURRENT_AMPS = 7.0`, `STALL_TIMEOUT_MS = 300` | measure with `Bench: Intake` |
| C5 | `JamDetector.configure(..., healthyResetMs)`, `Intake.HEALTHY_RESET_MS = 500` | `JamDetectorTest.aBriefDipBetweenPulsesDoesNotResetTheAttempts` |
| C6 | `Macros.reporting()` wraps every macro | `MacrosTest.boundedTimeoutLeavesATerminalOutcome` |
| C7 | `Shooter.waitForSpeedCommand()` requires nothing; only the hold owns the shooter | `MacrosTest` shooting tests |
| C8 | `Macros.INTAKE_RUNS_STORAGE = false` | `MacrosTest.intakeUntilFullTimesOutAndStopsEverything` |
| C10 | `Limelight.MOUNT_CALIBRATED = false` gates COLLECT/ALIGN | `TeleopTest.cameraMacrosAreIgnoredUntilTheMountIsMeasured` |
| C11 | `Limelight.BOTPOSE_FRAME_VERIFIED = false` gates the botpose | Javadoc names the frame mismatch |
| C12 | `Teleop.onInitLoop` localizes only with a camera, every `LOCALIZE_RETRY_MS` | `TeleopTest` |
| C13 | SDK 12.0 upgrade | **deferred**: its own task before any SDK AprilTag work |
| D1 | `AutoRoutineTest` asserts the captured leave powers | `AutoRoutineTest.shootsFourPreloadsThenLeavesTheWall` |
| D2, D3, D4 | this file, `docs/03` §11 and §20, `docs/specs/` | — |
| D5 | `opmodes/test/SelfTest` on `BenchOpMode`; `ConceptCommands` deleted | `BenchOpModesTest.selfTestPassesTheMotorsAndSkipsWhatIsNotFitted` |
| D6 | wheel directions: TestHardware / Mecanum Tuner, then `Constants.drivetrainConfig` | on the robot |
| D8 | `MatchLogger.KEEP_NEWEST = 40` | `MatchLoggerTest.openingALogPrunesTheOldestBeyondKeepNewest` |
| D9, D10, D15 | known, no action; see §7 | — |
| D13, D14 | field and standoff numbers | §9 |
| G1 (review) | blind shooting: `Storage.hasEntranceSensor`, `Macros.piecesOnBoard` returns `CAPACITY` when unknowable | `MacrosTest.shootOneFiresOnePulseWhenTheCountIsUnknowable`, `TeleopTest.operatorShootOneFeedsOnASensorlessRobot` |
| G2 | `PoseStorage.save(..., pieceCount)`; `MainAuto` writes, `Teleop.onInit` applies | `MainAutoTest.handsThePieceCountToTeleop`, `TeleopTest.inheritsThePieceCountLeftByAuto` |
| G3 | `HEADING_HOLD_STICK_DEADBAND = 0.001` | `DrivetrainCommandTest.aSmallDeliberateTurnReleasesTheHold`, `TeleopTest.aLightTurnStickIsNotSwallowedByTheHold` |
| G4 | `aimAndShootAll` bounds its aim | `MacrosTest.aimAndShootAllGivesUpAimingAfterTheTimeoutAndStillShoots` |
| G5 | `shootAllCore` builds `CAPACITY + 1` feed steps | `MacrosTest.shootAllShootsAPieceWaitingInTheLiftToo` |
| G6 | recovery only between pieces (`afterShot`) | `MacrosTest.shootOneDoesNotWaitForRecoveryAfterItsOnlyShot`, `shootAllWaitsForRecoveryOnlyBetweenPieces` |
| G7 | `Shooter.SECOND_MOTOR_DIRECTION`, `Storage.SECOND_MOTOR_DIRECTION` | `ShooterTest.secondFlywheelTakesItsOwnDirection` |
| G8 | `markCancelled` guard | `MacrosTest.markCancelledLeavesAFinishedMacroAlone` |

**Round 2** (2026-09-15; the full verdict for each claim, including the parts that were wrong, is in `fixthese.md` "Round 2 status"):

| Item | Change | Proof |
|---|---|---|
| R2-A1 | one intake current read per loop; `VelocityMotor.write` on change only (refresh every 12); presence sensors in rotation; loop stats on the auto card | `IntakeCommandTest.currentIsReadOnceALoop`, `VelocityMotorTest.anUnchangedVelocityIsNotResentEveryLoop`, `RobotTest.presenceSensorsAreReadInRotationAndTheEntranceEveryLoop` |
| R2-A2 | `Storage.hasFullSensor` / `canDetectFull`; operator count buttons; §8.2 step 0 | `RobotTest.aFullSensorIsKnownToTheStorage`, `TeleopTest.operatorCanMarkTheRobotFullWithoutASensor` |
| R2-A3, B6 | entrance counts by presence (`pieceEnteringStorage`); `PieceType.HUES_CALIBRATED` gates hue trust and the reject | `RobotTest.entranceCountsByDistanceBeforeTheHuesAreMeasured`, `aHueOnlyEntranceSensorIsNotTrustedUntilCalibrated` |
| R2-A4 | `intakeUntilFull` outcome by what the robot can know | `MacrosTest.intakeUntilFullIsATimedRunWithNoWayToKnowFull`, `intakeUntilFullSucceedsWhenTheFullSensorTrips` |
| R2-A5 | `Tunables` + `RobotTunables`; card lists; bench BACK | `TunablesTest`, `TeleopTest.initCardListsValuesTunedOnABench`, `BenchOpModesTest.benchBackRestoresTheTunablesAndTheCardListsThem` |
| R2-A6 | SDK PIDF read at construction and restored | `ShooterTest.turningCustomPidfOffRestoresTheSdkCoefficients` |
| R2-A7 | `BuildFlavor.isTuningBuild()` on every match init card | `BuildFlavorTest` |
| R2-A8 | unlocked warning; path-auto rule in `AutoRoutine` | `MainAutoTest.startingUnlockedWarnsButRuns` |
| R2-A9 | live second-motor direction; bench dpad right | `ShooterTest.secondFlywheelDirectionFollowsTheStaticAtRuntime`, `BenchOpModesTest.shooterBenchFlipsTheSecondMotorLiveAndTogglesPidfBothWays` |
| R2-A10 | via R2-A3 and the count buttons | as above |
| R2-B1, B2 | trust decided once in `wireSuppliers`; `sensingSummary()`; `Macros.ASSUME_FULL_WHEN_UNCOUNTED` | `MacrosTest.assumeFullCanBeTurnedOffByName` |
| R2-B4 | `Controls.Needs`, generic gates | `TeleopTest.everyDriveMacroIsRefusedWithoutAFollower` |
| R2-B3, B5 | **deferred**: structure only, no field effect; after the first event | — |
| rumble | final seconds = one 600 ms buzz, distinct from full | `TeleopTest.finalSecondsGiveOneLongRumbleOnBothPads` |

### 8.2 Before the first event, on the real robot, in this order

0. **Fit `sensor_storage_full` first** (a REV Color Sensor V3 at the fourth slot). It is one config
   name, the G407 interlock already exists, and it needs only `Robot.PRESENCE_DISTANCE_INCHES`. Until
   it is fitted the operator sets the count by hand (dpad up = 4). Every init card's `Sensors:` line
   says what each point is doing.
1. **Robot Controller configuration** names must equal `HardwareNames` (`front_left_drive`,
   `front_right_drive`, `back_left_drive`, `back_right_drive`, `intake`, `storage`, `transfer`,
   `shooter`, optional `storage_2` / `shooter_2`, `limelight`, `sensor_*`). Every OpMode's init
   screen lists anything missing; the robot still runs without it.
2. **Utility → TestHardware** (SDK): spin each drive motor by name; each must push the robot forward.
   Fix any reversed one in `Constants.drivetrainConfig`. (Or the Mecanum Tuner on a `-Ptuning` build.)
3. **`SelfTest`**: every row PASS or SKIP. A FAIL names the motor, its sign, or the sensor.
4. **`Bench: Shooter`**: confirm `TICKS_PER_REV` (measured t/s × 60 / true rpm), find `SHOOT_RPM`
   for a shot into the up-CELL from the wall, read the settled rpm spread into
   `AT_SPEED_TOLERANCE_RPM`, fire one and read the dip/recovery into `SHOT_RECOVERY_*`. Try `CUSTOM_PIDF`.
5. **`Bench: Storage` and `Bench: Transfer`**: find `SENSORLESS_FEED_PULSE_MS` (one pulse moves
   exactly one piece to the flywheel); check the entrance sensor's in-view loop count.
6. **`Bench: Intake`**: free speed → `MOTOR_FREE_SPEED_TICKS_PER_SEC`; clean-capture amps and a
   deliberate jam → `STALL_CURRENT_AMPS` / `STALL_TIMEOUT_MS`.
7. **`Bench: Color sensors`** with real POLLEN and NECTAR under venue lighting: hue windows and
   floors into `PieceType`, then `PieceType.HUES_CALIBRATED = true`; distance with and without a
   piece into `Robot.PRESENCE_DISTANCE_INCHES`; then consider `Intake.REJECT_ENABLED` (and whether
   the roller alone can eject from the entrance). Read the footer's loop p95 with 0, 1 and all
   sensors fitted: the anti-jam window needs `STALL_TIMEOUT_MS / p95` of at least 10 samples.
8. **`Bench: Limelight`** if fitted: fps, tag tx, and the mount check against a tape measure; then
   `Limelight.MOUNT_CALIBRATED = true`. Decide the facing (§10).
9. Set `AutoRoutine.LEAVE_DIRECTION` / `LEAVE_POWER` / `LEAVE_MS` for the real placement: the robot
   must clearly stop touching the wall and never reach the HIVE structure.
10. Run `Auto: shoot 4 + leave` once on the practice field with the 30 s clock, then `Teleop`: the
    operator fires Shoot One on the sensorless robot and sees one pulse, then presses dpad up, sees
    the intake roller hold, and Shoot All fires four.
11. **Before every match**: the init card must show no `!! TUNING BUILD` line and no
    `!! TUNED THIS SESSION` list you did not mean (restart the Robot Controller app to clear it).

### 8.3 Then

- **`pedro/Constants` + `Tuning`**: on a `-Ptuning` build, Pinpoint Tuner → paste `localizerConfig`,
  Foresight Tuner → paste `foresightConfig`; `create()` then returns the follower and the heading
  hold, aim lock, snap turns and Pedro paths come alive in `Teleop`.
- **The Pedro-path auto** (docs/03 §5 leg pattern): `aimAndShootAll(Field.firstTargetCell(alliance),
  tags...)` from `FieldPoses.BLUE_SHOOTING_SPOT` (rotated for red), park via `driveTo(FieldPoses.BLUE_PARK)`.
  Keep paths and turns sequential (§7).
- **SDK 12.0 upgrade** before any SDK AprilTag-cluster work.

## 9. Constants to measure on the robot

| File | Constant | How |
|---|---|---|
| `Intake` | `MOTOR_FREE_SPEED_TICKS_PER_SEC`, `INTAKE_TICKS_PER_SEC`, `STALL_CURRENT_AMPS`, `STALL_TIMEOUT_MS` | `Bench: Intake`: peak velocity unloaded; peak amps of a clean capture vs a deliberate jam |
| `Intake` | `REJECT_MS`, `REJECT_ENABLED` | `Bench: Color sensors` for the hues first, then a piece of the other colour at the entrance in `Bench: Intake` |
| `Storage`, `Transfer`, `Macros` | `ADVANCE_TICKS_PER_SEC`, `LIFT/FEED_TICKS_PER_SEC`, **`SENSORLESS_FEED_PULSE_MS`** | `Bench: Storage` / `Bench: Transfer`: one pulse moves exactly one piece |
| `Shooter` | `TICKS_PER_REV`, `SHOOT_RPM`, `AT_SPEED_TOLERANCE_RPM`, `SHOT_RECOVERY_MIN_MS`, `SHOT_RECOVERY_TIMEOUT_MS`, `CUSTOM_PIDF` + `PIDF_*` | `Bench: Shooter` |
| `Shooter`, `Storage` | `SECOND_MOTOR_DIRECTION` | `Bench: Shooter` / `Bench: Storage` dpad right flips it live; opposite measured signs = fighting |
| `Shooter` | `HEADING_OFFSET_RAD` | which way the flywheel fires relative to the intake (π = out the rear); confirm on the built robot |
| `Robot` | `PRESENCE_DISTANCE_INCHES` | `Bench: Color sensors`: distance with and without a piece at the full/transfer/feed points |
| `game/PieceType` | hue windows (`POLLEN`, `NECTAR_RED_HUE_DEGREES`, `NECTAR_BLUE_HUE_DEGREES`), `HUE_TOLERANCE_DEGREES`, `MIN_SATURATION`, `MIN_VALUE`, then **`HUES_CALIBRATED`** | `Bench: Color sensors` on real pieces under match lighting (hold Y for min/max) |
| `Macros` | `ASSUME_FULL_WHEN_UNCOUNTED`, `INTAKE_TIMEOUT_MS` | a strategy call, not a measurement: `true` shoots blind when nothing can count; the timer is how long a sensorless intake runs |
| `Limelight` | `CAMERA_HEIGHT_INCHES`, `CAMERA_PITCH_DEGREES` (positive down), offsets, `CAMERA_YAW_OFFSET_DEGREES`, then `MOUNT_CALIBRATED` | tape measure after the mount is final; `Bench: Limelight` estimate vs tape |
| `Drivetrain` | `HEADING_HOLD_P/I/D` (also the aim lock's gains) | on the field with the drivetrain tuned |
| `Macros` | `AIM_TOLERANCE_DEGREES`, the `*_TIMEOUT_MS` values | on the field; a shot that misses wide is the aim tolerance, a shot that never fires is a timeout |
| `pedro/Constants` | `drivetrainConfig` directions (TestHardware / Mecanum Tuner), then `PinpointConfig`, `ForesightConfig` | `-Ptuning` build, AutoTune web UI at `http://192.168.43.1:10158` |
| `AutoRoutine` | `LEAVE_DIRECTION`, `LEAVE_POWER`, `LEAVE_MS`, `SETTLE_MS`, `SHOOT_BUDGET_MS` | on the practice field |
| `game/Field` | `HIVE_LATERAL_OFFSET_INCHES`, `UP_CELL_OPENING_HEIGHT_INCHES`, `FLOWER_INSET_INCHES`, LOADING ZONE / GARDEN placement | Onshape field CAD (docs/04 §10), then a tape measure at the first event |
| `game/FieldPoses` | every pose (start standoffs, `ROBOT_HALF_LENGTH_INCHES` incl. pre-load protrusion, `BLUE_SHOOTING_SPOT`, `BLUE_PARK`, `BLUE_GARDEN_APPROACH`) | drive the real field; correct the BLUE value in place and red follows |
| `PositionalServo` | `TRAVEL_MS_PER_UNIT` | V2 only |

## 10. Open hardware questions

- Motor count: two storage motors and two flywheels together would be nine; which one is single,
  or which pair shares a motor. Directions for a pair: `SECOND_MOTOR_DIRECTION`.
- **Camera: fitted on V1, and facing which way?** Front = piece detection only, aim on odometry.
  Rear = tag-refined aiming while aimed. Undecided; the code defaults to "not measured".
- Whether the storage side wheels must run while intaking (`Macros.INTAKE_RUNS_STORAGE`, off).
- Which way is "off the wall" for LEAVE given how the robot is placed (`AutoRoutine.LEAVE_DIRECTION`).
- Sensor type at each of the five reserved points (`HardwareNames.SENSOR_*`); a colour sensor at the
  storage entrance is effectively mandatory for G408, and a REV V3 at the full/transfer/feed points
  gives presence by distance.
- Whether the intake roller alone can eject a piece already at the storage entrance (the G408
  reject), or the storage must reverse with it.
- Flywheel count, feed gate or transfer pulse.

## 11. References

- Specs: `docs/specs/BIOBUZZ_V1_Robot_Physical_Architecture.md` (authoritative, except that V1 has
  no turret), `docs/specs/BIOBUZZ_V2_Robot_Physical_Architecture.md` (final/V2 robot, not in scope).
- Reference codebase: the team's FTC_Guide repo (Pedro 2.1.2 / Ivy 1.0.0 / Panels), external to this one.
- Competition Manual V1 (HTML): https://ftc-resources.firstinspires.org/ftc/game/cm-html ;
  Team Updates: https://ftc-resources.firstinspires.org/ftc/game/tu-00 ; Q&A opens 2026-09-28.
- SDK v12.0: https://github.com/FIRST-Tech-Challenge/FtcRobotController/releases/tag/v12.0
- Pedro: https://pedropathing.com ; Ivy: https://github.com/Pedro-Pathing/Ivy ;
  Maven repo `https://repo.dairy.foundation/releases/`.
- Library sources used for the docs live in the Gradle cache under
  `~/.gradle/caches/modules-2/files-2.1/com.pedropathing*/` (revhub/tuning sources jars; core is bytecode only).
