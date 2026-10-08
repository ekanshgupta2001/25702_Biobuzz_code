# HANDOFF — FTC Team 25702 BIOBUZZ Robot Code

State as of **2026-10-07**: the rebase onto the real robot (2026-09-27), then Teleop and the tests
simplified (2026-10-07). Read this first: it says what
exists, what does not, why, and what to do next. Everything here is backed by a file in the repo or a
measurement you can repeat.

---

## 1. What this is

Team 25702, FTC 2026-27 game **BIOBUZZ presented by RTX**. Code for the robot in
`Non wheels full assembly.glb`: a mecanum chassis that collects game pieces through a full-width
roller, carries them down a tunnel, and launches them out of the **rear** into a HIVE CELL.

Stack: **Pedro Pathing 3.0.0** (`revhub` + `core`), **Ivy 1.1.1** command scheduler, **AutoTune**
(`tuning:1.0.1`), FTC SDK **11.2.1**, Gradle 9.1.0, Java 8 source level.
**No Panels, no FTC Dashboard: AutoTune (the Pedro tuning site) is the only tuning tool. It is in every
build, so the APK is NOT match legal (R704)** until it is stripped (§4).

### The robot, from the CAD

The `.glb` is a **parts-layout file, not a positioned assembly** — 33.9 × 49.3 × 17.2 in overall with
the sub-assemblies spread along one axis. So it tells you the mechanisms and the hardware and
**nothing about geometry**. Every pose, height and standoff in `game/` is still a placeholder.

| Mechanism | Motors | Hardware |
|---|---|---|
| **Drivetrain** | 4 × goBILDA 5203-2402-0014 (13.7:1, **435 RPM**) | mecanum + **goBILDA Pinpoint + 2 pods** |
| **Intake** — roller **and tunnel**, one motor | 1 × 5203-2402-0051 (50.9:1, **117 RPM**) | 10 × 16 mm × 16 mm 30A rollers on a 288 mm REX shaft; tunnel = 8 × Gecko 32 mm, 4 × Gecko 72 mm, 3 × Gecko 48 mm, 5 sprockets + chain |
| **Shooter** | 2 × non-goBILDA "bare" motors | two **opposed 2.9 in** flywheels, 1.26 in wide; GT2 40T per motor + GT2 50T + HTD-3mm-43T + 97T belt |
| **Limelight 3A** | — | front-mounted, AprilTag aiming only |

**Seven motors.** R503 allows eight, so there is exactly one spare port.

`Assembly 1` and `Assembly 2` in the CAD (two 6000 RPM motors and Gecko wheels each) are **dead design
iterations**. They have no code and should not get any.

### Three things the code deliberately does not do

- **It never counts game pieces.** There is no sensor in the roller, the tunnel or the shooter. Shoot
  One fires one tunnel pulse; Shoot All fires `Macros.PIECES_PER_LOAD` (4). `getShotsFired()` is a
  pulse count. The operator decides when the robot is empty (G407 caps it at 4 controlled).
- **It does not localise from AprilTags.** Every BIOBUZZ tag rides on a moving HIVE CELL. The pose is
  the Pinpoint's alone; tags only refine the aim.
- **It does not fail soft.** A config name that is not in the configuration throws at OpMode init.

## 2. State of the build

| Layer | Files | Lines | Status |
|---|---|---|---|
| `Robot.java` | 1 | 203 | composition root |
| `subsystems/` | 4 | 910 | Drivetrain, Intake, Shooter, Limelight |
| `commands/` | 3 | 513 | Macros, Shoot, Waits (used by Auto and Teleop's shot buttons) |
| `opmodes/` | 8 | 774 | Teleop (one OpMode), Auto ×3 + its base, three benches |
| `game/`, `util/` | 9 | 748 | field frame, poses, clock, jam detector, angles |
| `pedro/` | 2 | 188 | Constants + Tuning registration |
| `pedro/procedures/` | 5 | 2,212 | the four Quickstart AutoTune procedures + our Shooter Tuner; stripped before events |

Team code, excluding the procedures: **3,336 lines across 27 files**, down from 4,732 across 33 before
the 2026-10-07 simplification. The APK builds.

**There is no test suite and none is planned.** The JVM suite was deleted in `0efa320`. Verification is
the compile gate plus §8's on-robot sequence.

Driver Station OpModes: `Teleop`, `Auto BLUE`, `Auto RED` (group **Main**); `Bench: Drive`,
`Bench: Intake`, `Bench: Limelight` (group **Bench**). The Shooter Tuner is on the AutoTune site, not in
the OpMode list.

## 3. Read these first

| File | Why |
|---|---|
| `docs/03-software-architecture.md` | the loop contract, package map, per-subsystem API, the aim law, the library rules that shaped the design, and §18's practical notes |
| `docs/02-robot-physical-architecture.md` | the real mechanism set and which subsystem owns each |
| `docs/01-libraries-pedro-3.0.0-ivy-1.1.1.md` | the two libraries' real APIs, verified from bytecode. Read before touching `Drivetrain` or `Macros` |
| `docs/04-biobuzz-season-analysis.md` | sourced game and rule facts, and §9's implications |
| `opmodes/teleop/Teleop.java` | every binding, read straight off the gamepads; the header comment is the button card |
| `game/Field.java` Javadoc | the field frame (origin A1, +X toward F, +Y toward 6) that every pose depends on |
| `util/hardware/HardwareNames.java` | the exact strings the Robot Controller configuration must use |
| `fixthese.md` | five review rounds. Round 5 is this rebase; rounds 1–4 describe code that no longer exists |

## 4. Build and run

```bash
# Gradle 9.1 may reject a newer system JDK; the Android Studio JBR always works.
JAVA_HOME="/Applications/Android Studio.app/Contents/jbr/Contents/Home" \
  ./gradlew :TeamCode:compileDebugJavaWithJavac --console=plain

./gradlew :TeamCode:assembleDebug   # the one APK — AutoTune on :10158
```

**There is one build, and it always contains AutoTune.** The team chose this on 2026-09-30 so that
every file sits in the normal folders and Android Studio resolves them with no variant to select. FTC
Dashboard was removed on 2026-10-07: AutoTune is the only tuning tool, and the shooter is tuned there
too. The price: **every APK binds AutoTune's web server and is not match legal (R704).** Before an
event, delete the `tuning` line from `build.dependencies.gradle` and the two sources that import it
(`pedro/Tuning.java`, `pedro/procedures/`), then build and deploy. Nothing on the card warns you; the
giveaway is the tuning site still answering at :10158. (A two-variant split with the tuning files in
`src/tuning/java` was tried and reverted before it was committed, because the team wanted every file in
the normal folders.)

## 5. Repo map

`TeamCode/src/main/java/org/firstinspires/ftc/teamcode/`

```
Robot.java                     composition root: 4 public final subsystems, readSensors()/writeActuators(),
                               loop Hz, battery, alliance. No supplier wiring, no sensor policy.
subsystems/
  Drivetrain.java              ONE Mecanum, plus a Follower once tuned. Sticks (field/robot centric), pose,
                               followLazy/driveForMs for Auto. hasFollower() gates the tuned half.
  Intake.java                  roller + tunnel on one motor: in/out/off on setPower, JamDetector
  Shooter.java                 2 opposed motors, kS/kV/kP on setPower, distance->ticks/sec table,
                               fixed-speed override (MANUAL_TICKS_PER_SEC)
  Limelight.java               the target CELL's tag tx, and nothing else. The one fail-soft device.
commands/
  Macros.java                  shootOne/shootAll/aimAndShootAll/driveTo; Auto's aim law
  Shoot.java                   the cycle as one state machine: SPIN_UP -> FEED -> SETTLE -> RECOVER -> DWELL
  Waits.java                   waitMs, and bounded() — the every-wait-has-a-deadline rule
opmodes/
  MatchOpMode.java             Auto's base. Final read -> decide -> execute -> write.
  teleop/Teleop                the one match TeleOp: dpad alliance pick in init, every binding inline
  auto/Auto                    30 s routine; tuned (aim, shoot, park) and untuned (shoot, timed leave)
  auto/{BlueAuto, RedAuto}     13 lines each
  test/{DriveBench, IntakeBench, LimelightBench}   plain OpModes, one subsystem each
game/{Field, FieldPoses}       field frame, HIVE CELL poses, tag ranges. Every number a placeholder.
util/
  field/{Alliance, FieldConstants, PoseStorage}   ROTATE_180; PoseStorage carries the pose only
  math/Angles   time/MatchClock   control/JamDetector
pedro/{Constants, Tuning, procedures/*}          configs + create(); AutoTune factories, tuners, ShooterTuner
```

### Teleop at a glance

One OpMode, `Teleop`. During init, gamepad 1's **dpad left = RED, dpad right = BLUE**; it starts on the
side the last autonomous ran (BLUE if none). The header comment of `Teleop.java` is the button card.

| Pad | Input | Does |
|---|---|---|
| driver | L-stick / R-stick X | drive. Field-centric with the alliance's driver frame once tuned; robot-centric before that |
| driver | L-trigger (hold) | slow mode (`Teleop.SLOW_SCALE`) |
| driver | **R-trigger (hold)** | **aim**: turns the rear shooter to the up-CELL while the sticks still translate. Odometry bearing, plus a Limelight correction taken while the tags were in view (kept `TAG_CORRECTION_MS`). Needs the tuned pose |
| driver | options / Y | field ↔ robot centric / re-zero heading (face away from your wall) |
| operator | RB / LB / both | intake in / out / stop (both also cancels a running shot; bumpers are ignored until both are up) |
| operator | L-trigger | flywheel on / off |
| operator | R-trigger / A | shoot one pulse / shoot four (ignored while a shot is running) |
| operator | dpad up | flywheel speed from the distance table ↔ fixed `Shooter.MANUAL_TICKS_PER_SEC` (fixed is automatic with no pose) |
| operator | dpad left / right | fixed speed −/+ `Teleop.SPEED_STEP_TICKS_PER_SEC` |
| operator | dpad down | "our HIVE tipped": aim and distance switch to the other CELL. **Press once at the start if auto tipped it** |

No rumbles: everything is on the telemetry card.

### Auto at a glance

Place the robot touching its wall with the **rear (shooter) toward the up-facing CELL** and run
`Auto BLUE` or `Auto RED`. There is no selector and no lock, so there is no way to run the wrong route.

- **Tuned**: aim at the CELL with the tag-refined aim law → shoot four → path to park. 20 + 3 + 5.
- **Not tuned**: shoot from where it was placed → back off the wall on a timer. 20 + 3.

The first TIP needs only **three** POLLEN: the up-CELL starts with 3 NECTAR, the robot pre-loads 4
POLLEN, and a HIVE tips on 3 + 3. That is why the routine spends its budget shooting, not driving.
At the buzzer anything still running is cancelled and every mechanism stopped (G403).

## 6. Decisions already made, and why

| Decision | Why |
|---|---|
| **One `Mecanum`, shared with the `Follower`** | `Mecanum(HardwareMap, MecanumConfig)` resolves the four `DcMotorEx` itself and wraps each in its own `CachedMotor`. Two instances over four motors keep two power caches and fight — *that*, and only that, is what the old "one motor layer at a time" doctrine was working around. One instance, handed to the `Follower` when it exists, makes the doctrine and `OpenLoopDrive` both unnecessary. |
| **The flywheel is open-loop power plus a P term, not `setVelocity`** | `setPower(kV*target + kP*(target − velocity) + kS)` every loop, velocity from one motor, motors left in their default run mode. Feedforward-dominant with a P term that saturates almost at once, so it holds the known-good power and slams to full on sag. That is why there is **no recovery wait between shots and no at-speed dwell latch** — both existed to paper over the SDK velocity PID's slow recovery on a high-inertia wheel. |
| **Everything in ticks/sec, never RPM** | `getVelocity()` is native ticks/sec. An RPM API needs `TICKS_PER_REV`, and pairing a 312 RPM part's 537.7 with a 435 RPM motor overstates the ceiling by ~40%. With no conversion there is no wrong number to hold. |
| **A hand-measured distance table, not a curve fit** | Taken from a competition-proven reference robot whose own source preserves four abandoned attempts in comments — a linear fit, a quartic, then a quadratic — before it settled on a measured table. Do not re-derive a polynomial. |
| **Crash loudly on a bad config name** | No `Hardware` wrapper, no `isAvailable()`, no null-motor guards. The old code had 51 `isAvailable()` guards, and what they bought was a robot that boots, drives onto the field, and silently plays a match with one mechanism disabled. A throw at init names the device while you are still in the pit. |
| **…except the Limelight** | Its constructor and `update()` are synchronous network calls to another computer over USB-Ethernet, not config lookups. A camera that is unpowered but correctly configured must not kill the OpMode, so it keeps a try/catch and `isConnected()`. That is the distinction: a missing *name* is a build error, an unreachable *peer* is a runtime condition. |
| **Teleop and the benches are plain OpModes** (2026-10-07) | The team could not debug the hook-based base class, the `Controls` enum and the heading-hold/aim-lock layer. Teleop now reads the gamepads directly and writes the loop out (`readSensors` → gamepads → `Scheduler.execute()` → `writeActuators`); each bench builds only the subsystem it tests. Ivy stays for Auto and for Teleop's shot buttons. |
| **Alliance: Auto from the OpMode class, Teleop from the dpad** | `BlueAuto`/`RedAuto` fix the route, so the wrong one cannot run. Teleop is one OpMode with a dpad pick in init that starts on the side Auto last ran (`PoseStorage`); the card shows the alliance in capitals. |
| **A fixed-speed override, on one button** | Every automatic shot input is pose-derived, so one hard collision breaks the distance table and the aim at once. Operator dpad-up switches the flywheel to `MANUAL_TICKS_PER_SEC` (trimmed on dpad left/right); the driver simply stops holding the aim trigger. |
| **Two default commands read a supplier every loop** | `Intake.operatorControlCommand` and `Shooter.armedControlCommand`. Ivy has **no duplicate guard**, so re-scheduling an intent on each press re-runs `start()` every loop; and Ivy *ends* rather than suspends a preempted priority-0 command, so a press-scheduled hold needs restoring by hand after every shot. At priority −1 with `SUSPEND` a shooting cycle preempts them and they resume by themselves. |
| **The shooting cycle is one state machine, not an Ivy group tree** | Ivy's `Sequential` hands off one child per `execute()` and never executes the child it just started, so every boundary costs a whole ~20 ms loop and an `instant` costs one by itself. The old nine-level tree spent 7 idle loops between "flywheel recovered" and "next pulse". A state machine chains every zero-duration transition inside one `execute()`. It also sidesteps the `Repeat.end()` NPE for free. |
| **`Shoot` refuses to run with a zero flywheel target** | `atTarget()` is `|target − velocity| < tolerance`, so a zero target is "at speed" on tick one and the tunnel would push the whole load through a dead wheel. |
| **Benches and tuners go through the subsystem classes** | They construct `Drivetrain`/`Intake`/`Limelight`/`Shooter` rather than grabbing motors by name, so a bench drives exactly the code the match runs and each motor has one power cache. |
| **Every wait has a deadline (`Waits.bounded`)** | The reference codebase writes `waitUntil(shooter::atTarget)` with no timeout; a dead flywheel hangs its autonomous for the rest of the match. One `race` against a timer is the whole fix. |
| **Every macro reports a terminal `Outcome`** | A timeout or an abort used to leave it `RUNNING` for ever. Built as `deadline(sequential(...), onInterrupt(markCancelled))` — **not** `setEnd` on the group, because Ivy's `setEnd` *replaces* the group's own end, the one that ends its children. |
| **`FieldConstants` rotates, it does not mirror** | The BIOBUZZ field is 180° rotationally symmetric: GARDEN A1/F6, LOADING ZONE A5/F2. A C/D mirror would put them on F1 and F5 and every blue auto in the wrong place. The enum with three arms is gone — a run-time selector over a fact the field cannot change is just somewhere for a wrong value to hide. |
| **SDK 11.2.1 is fine** | The docs used to defer a 12.0 upgrade for the AprilTag *cluster* API. The tags are read by the Limelight's own pipeline over USB-Ethernet, not by the SDK's `AprilTagProcessor`, so nothing in the confirmed hardware needs it. It is a future task, not a prerequisite. |
| **AutoTune is the only tuning tool; always built in, stripped by hand before events** | The Pedro tuning site tunes the drivetrain, and our `ShooterTuner` procedure on the same site tunes kS/kV/kP by timing spin-ups. FTC Dashboard was removed on 2026-10-07. AutoTune binds a web server whenever it is in the APK, and R704 prohibits streaming tools during matches, so the event APK must have it removed (§4). |

## 7. Rules that bite

**Pedro 3.0.0** (docs/01 §A)
- `new Follower(Localizer, Drivetrain, Algorithm)`. The Quickstart's own comment has the order wrong.
- Exactly one `follower.update()` per loop; it ticks the localizer itself.
- `atParametricEnd()` means the path is done **and is also true whenever the follower is not
  following** — anything that puts it in HOLD or MANUAL mid-path makes a follow command report
  "arrived". Keep paths and turns sequential, never parallel.
- `isBusy()` only clears inside a hold. Never use it as "still following".
- A `PinpointLocalizer` constructor starts an IMU calibration; build in `init()`. `Drivetrain`
  re-applies the init pose once `LOCALIZER_SETTLE_MS` has passed.
- Paths need a heading (`.constant/.linear/.tangent`) or `follow()` throws.
- No callbacks, no `setMaxPower`, no `turnTo`. A turn is `hold(pose.withHeading(h))`.
- `@Tuner` factories run at RC start-up: static, zero-arg, return `Procedure`, never throw.
- **The official docs are wrong in four places** (docs/01 A.4, A.6, B.1, and the `Mecanum`/`CachedMotor`
  note). Follow the bytecode, not the page.

**Ivy 1.1.1** (docs/01 §B)
- `Command.unless()` never finishes. Use `conditional(cond, real, instant(() -> {}))`.
- `Commands.lazy(...)` contributes no requirements; always `.requiring(subsystem)`.
- A direct hardware call from an `instant` with no requirement is overwritten by the default command.
- `Scheduler` is static: `Scheduler.reset()` first thing in `init()`.
- **No duplicate guard.** Scheduling the same intent every loop re-runs `start()` every loop.
- Ivy **ends** rather than suspends a preempted priority-0 command.
- `deadline` and `parallel` end unfinished children with the **group's own** end condition. Judge by the
  sensor or the condition, never by `EndCondition`.
- A `sequential` hands off one child per `execute()`; every boundary is a whole loop.
- **Never put `repeat(...)` in a group that can be interrupted before reaching it**: `Repeat.end()` NPEs.
- **`setEnd` on a group replaces the group's own end.** Run interrupt work as a child.

**SDK gamepads and OpModes**
- `*WasPressed()` consumes its flag on read. Read every edge once per loop, in one place, and
  `resetEdgeDetection()` in `start()` — `Teleop` and `MatchOpMode` both do.
- Iterative OpModes may not write motors from `stop()`; the SDK zeroes them itself.
- `Direction` is applied to commanded power *and* reported velocity, so a reversed motor reads positive.
- **Utility → TestHardware** spins any motor by config name. First check at every event.

**Game rules** (docs/04)
- G407: at most 4 controlled scoring elements. **Nothing on this robot enforces it — the operator does.**
- G408: never control the opponent's NECTAR. There is no colour sensing, so this is also the operator's job.
- R503: 8 motors and 8 servos max. We use 7 motors, 0 servos.
- R704: no dashboard or streaming tools in matches. The everyday APK is **not** match legal: strip AutoTune first (§4).
- AUTO 30 s → 8 s no-motion transition (G403) → TELEOP 120 s. No endgame. FLOWER unlock at 1:00.

## 8. On the real robot, in this order

1. **Robot Controller configuration** names must equal `HardwareNames`: `front_left_drive`,
   `front_right_drive`, `back_left_drive`, `back_right_drive`, `intake`, `shooter_left`,
   `shooter_right`, `pinpoint`, `limelight`. A wrong name throws at init and names itself.
2. **Utility → TestHardware**: spin each of the seven motors by name. Each drive motor must push the
   robot forward — fix any reversed one in `Constants.drivetrainConfig`. Confirm the two flywheels
   **counter-rotate**.
3. **`Bench: Drive`**: robot-centric, every stick direction moves the robot the way it says.
4. **AutoTune** at `http://192.168.43.1:10158`: Mecanum Tuner → `drivetrainConfig`;
   Pinpoint Tuner → `localizerConfig`; Foresight Tuner → `foresightConfig`. Paste all three. `create()`
   then returns a follower and field-centric, the pose, aiming and Auto's paths all come alive. Back on
   `Bench: Drive`, OPTIONS switches to field-centric and x / y / heading should track a tape measure.
5. **`Bench: Intake`**: RB in, LB out, both stop. Confirm one motor turns **both** roller and tunnel.
   Note the current of a clean pick-up and of a deliberate stall; `STALL_CURRENT_AMPS` goes between them.
6. **Shooter Tuner** on the AutoTune site: set a target, then raise `kV` until the wheel gets close on its
   own, `kS` until it starts cleanly, and `kP` until the spin-up time stops improving without a big
   overshoot. Tick Done and **paste the code block into `Shooter.java`** (values typed there only last
   until the RC app restarts). Then the distance table: in Teleop's fixed-speed mode, park at
   24/48/72/96 in, trim until the piece lands in the CELL, and write each pair into
   `DISTANCES_INCHES` / `TICKS_PER_SEC`.
6a. **`Bench: Limelight`**: enter the camera pose in the Limelight web UI, then at a tape-measured floor
   distance confirm `distance` and `x`/`y` agree (if a tag on the left shows negative `y`, flip
   `ROBOT_SPACE_Y_SIGN`). Once tuned, compare "Pose from tags" with the Pinpoint pose.
7. **Teleop on the practice field**: pick the alliance on the dpad, drive, flywheel on, shoot one —
   one pulse should move one piece. Face the HIVE so the card says `tag-corrected`, then hold the aim
   trigger and confirm the rear swings onto the CELL. Press dpad-up and confirm the speed goes fixed.
8. **`Auto BLUE`** with the 30 s clock. Set `Auto.LEAVE_POWER` / `LEAVE_MS` so the robot clearly stops
   touching the wall and never reaches the HIVE. Then check Teleop inherits the pose and the alliance.
9. **Before every event**: delete the `tuning` line from `build.dependencies.gradle` and the two sources
   that import it (`pedro/Tuning.java`, `pedro/procedures/`), then build and deploy. Confirm the tuning
   site no longer answers at :10158. The everyday build is not match legal and nothing else warns you.

## 9. Constants to measure

| File | Constant | How |
|---|---|---|
| `Shooter` | `kS`, `kV`, `kP` | Shooter Tuner on the AutoTune site (§8 step 6) |
| `Shooter` | `DISTANCES_INCHES` / `TICKS_PER_SEC`, `MANUAL_TICKS_PER_SEC`, `TOLERANCE_TICKS_PER_SEC` | Teleop fixed-speed mode, from real shots (§8 step 6) |
| `Shooter` | `SECOND_MOTOR_REVERSED` | TestHardware: the two wheels must counter-rotate at the same power |
| `Shooter` | `HEADING_OFFSET_RAD` | π = fires out the rear. Confirm on the built robot |
| `Intake` | `IN`, `OUT` | `Bench: Intake`: enough to carry pieces without grinding them |
| `Intake` | `STALL_CURRENT_AMPS`, `STALL_TIMEOUT_MS` | `Bench: Intake` current readout, two runs (§8 step 5) |
| `Shoot` | `FEED_PULSE_MS`, `RECOVER_TIMEOUT_MS`, `FINAL_DWELL_MS` | Teleop shoot one: one pulse must move exactly one piece |
| `Teleop` | `AIM_P`, `AIM_MAX_TURN`, `TAG_CORRECTION_MS`, `SLOW_SCALE` | on the field, once tuned: aiming should settle without wobbling |
| `Macros` | `AIM_TOLERANCE_DEGREES`, `AIM_BIAS_MAX_AGE_MS`, the `*_TIMEOUT_MS` values | on the field. A shot wide is the tolerance; a shot that never fires is a timeout |
| `Auto` | `LEAVE_POWER`, `LEAVE_MS`, `SETTLE_MS`, `SHOOT_BUDGET_MS` | practice field |
| `game/Field` | `HIVE_CENTER`, `CELL_SPACING_INCHES`, `HIVE_LATERAL_OFFSET_INCHES` | Onshape field CAD (docs/04 §10), then a tape measure |
| `game/FieldPoses` | every pose, and `ROBOT_HALF_LENGTH_INCHES` | drive the real field. Correct the BLUE value; red follows by rotation |
| `pedro/Constants` | `drivetrainConfig` directions, then `localizerConfig`, `foresightConfig` | TestHardware, then AutoTune (§8) |

## 10. Open questions

- **Which of the two flywheel motors is which**, and the belt ratio through the GT2 40T/50T and
  HTD-3mm-43T train. The code needs neither — it works in ticks/sec at the encoder — but the reachable
  top speed does.
- **Whether one tunnel pulse really moves exactly one piece**, or whether the tunnel needs its own
  gate. If a pulse feeds two, `Shoot.FEED_PULSE_MS` may not be able to separate them and the mechanism
  needs a look.
- **Whether the roller can eject a piece already in the tunnel** (it is the only way to shed a piece
  without shooting it, and G408 may require it).
- **Where the Limelight ends up**, and whether `CAMERA_YAW_OFFSET_DEGREES` is still 0. A tag dead ahead
  must read tx ≈ 0.
- **The spare motor port**: nothing needs it yet.

## 11. References

- Specs: `docs/specs/BIOBUZZ_V1_Robot_Physical_Architecture.md` (superseded in part by the CAD — no
  turret, no separate storage or transfer), `docs/specs/BIOBUZZ_V2_Robot_Physical_Architecture.md`.
- Reference codebase: `22131-Decode-master 2/` — a competition-proven robot this code's shape is
  modelled on. Git-ignored; it is an input, not part of this project.
- Competition Manual V1: https://ftc-resources.firstinspires.org/ftc/game/cm-html ·
  Team Updates: https://ftc-resources.firstinspires.org/ftc/game/tu-00
- Pedro: https://pedropathing.com · Ivy: https://github.com/Pedro-Pathing/Ivy ·
  Maven `https://repo.dairy.foundation/releases/`
- Library sources for docs/01 live in the Gradle cache under
  `~/.gradle/caches/modules-2/files-2.1/com.pedropathing*/`.
