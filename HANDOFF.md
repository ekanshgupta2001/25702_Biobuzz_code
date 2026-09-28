# HANDOFF — FTC Team 25702 BIOBUZZ Robot Code

State as of **2026-09-27**, after the rebase onto the real robot. Read this first: it says what
exists, what does not, why, and what to do next. Everything here is backed by a file in the repo or a
measurement you can repeat.

---

## 1. What this is

Team 25702, FTC 2026-27 game **BIOBUZZ presented by RTX**. Code for the robot in
`Non wheels full assembly.glb`: a mecanum chassis that collects game pieces through a full-width
roller, carries them down a tunnel, and launches them out of the **rear** into a HIVE CELL.

Stack: **Pedro Pathing 3.0.0** (`revhub` + `core`), **Ivy 1.1.1** command scheduler, **AutoTune**
(`tuning:1.0.0`, tuning builds only), FTC SDK **11.2.1**, Gradle 9.1.0, Java 8 source level.
**No Panels, no FTC Dashboard** (R704).

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
| `Robot.java` | 1 | 208 | composition root |
| `subsystems/` | 4 | 1,201 | Drivetrain, Intake, Shooter, Limelight |
| `commands/` | 3 | 541 | Macros, Shoot, Waits |
| `opmodes/` | 11 | 1,425 | one base, teleop ×2 + Controls, auto ×3, three benches |
| `game/`, `util/` | 10 | 805 | field frame, poses, clock, jam detector, maths |
| `pedro/` | 2 | 178 | Constants + Tuning registration |
| `pedro/procedures/` | 4 | 2,100 | vendored AutoTune; **0 bytes in the match APK** |

Team code, excluding the vendored procedures: **4,358 lines across 31 files**, down from 7,730 across
53. Both APKs build. The competition APK contains **zero** AutoTune references across all nineteen dex
files — verified by scanning dex strings, not by trusting the Gradle exclusion.

**There is no test suite and none is planned.** The JVM suite was deleted in `0efa320`. Verification is
the compile gate plus §8's on-robot sequence.

Driver Station OpModes: `Teleop BLUE`, `Teleop RED`, `Auto BLUE`, `Auto RED` (group **Main**);
`Bench: Shooter`, `Bench: Intake`, `SelfTest` (group **Bench**).

## 3. Read these first

| File | Why |
|---|---|
| `docs/03-software-architecture.md` | the loop contract, package map, per-subsystem API, the aim law, the library rules that shaped the design, and §18's practical notes |
| `docs/02-robot-physical-architecture.md` | the real mechanism set and which subsystem owns each |
| `docs/01-libraries-pedro-3.0.0-ivy-1.1.1.md` | the two libraries' real APIs, verified from bytecode. Read before touching `Drivetrain` or `Macros` |
| `docs/04-biobuzz-season-analysis.md` | sourced game and rule facts, and §9's implications |
| `opmodes/teleop/Controls.java` | every binding and the help card. Change a button there and nowhere else |
| `game/Field.java` Javadoc | the field frame (origin A1, +X toward F, +Y toward 6) that every pose depends on |
| `util/hardware/HardwareNames.java` | the exact strings the Robot Controller configuration must use |
| `fixthese.md` | five review rounds. Round 5 is this rebase; rounds 1–4 describe code that no longer exists |

## 4. Build and run

```bash
# Gradle 9.1 may reject a newer system JDK; the Android Studio JBR always works.
JAVA_HOME="/Applications/Android Studio.app/Contents/jbr/Contents/Home" \
  ./gradlew :TeamCode:compileDebugJavaWithJavac --console=plain

./gradlew :TeamCode:assembleDebug              # COMPETITION APK — no AutoTune
./gradlew -Ptuning :TeamCode:assembleDebug     # tuning APK — AutoTune web UI on :10158
```

**The Run button deploys whichever variant you built last.** After any tuning session, rebuild the
competition APK before you go to a match. There used to be a runtime `BuildFlavor` warning on the init
card that caught this; it was deleted, so it is now a human check. Build plain, then deploy.

## 5. Repo map

`TeamCode/src/main/java/org/firstinspires/ftc/teamcode/`

```
Robot.java                     composition root: 4 public final subsystems, readSensors()/writeActuators(),
                               loop Hz, battery, alliance. No supplier wiring, no sensor policy.
subsystems/
  Drivetrain.java              ONE Mecanum, plus a Follower once tuned. Sticks, heading hold, aim lock,
                               followLazy/turnTo/driveForMs. hasFollower() gates the tuned half.
  Intake.java                  roller + tunnel on one motor: in/out/idle/off on setPower, JamDetector
  Shooter.java                 2 opposed motors, kS/kV/kP on setPower, distance->ticks/sec table,
                               manual override, open-loop bench mode
  Limelight.java               the target CELL's tag tx, and nothing else. The one fail-soft device.
commands/
  Macros.java                  shootOne/shootAll/aimAndShootAll/aimAt/snapToHeading/driveTo; the aim law
  Shoot.java                   the cycle as one state machine: SPIN_UP -> FEED -> SETTLE -> RECOVER -> DWELL
  Waits.java                   waitMs, and bounded() — the every-wait-has-a-deadline rule
opmodes/
  MatchOpMode.java             THE base. Final read -> decide -> execute -> write. Benches override
                               usesScheduler() to false and drive subsystems directly.
  teleop/{Teleop, Controls}    behaviour and bindings
  teleop/{BlueTeleop, RedTeleop}   13 lines each: the annotation and the side
  auto/Auto                    30 s routine; tuned (aim, shoot, park) and untuned (shoot, timed leave)
  auto/{BlueAuto, RedAuto}     13 lines each
  test/{SelfTest, ShooterBench, IntakeBench}
game/{Field, FieldPoses}       field frame, HIVE CELL poses, tag ranges. Every number a placeholder.
util/
  field/{Alliance, FieldConstants, PoseStorage}   ROTATE_180; PoseStorage carries the pose only
  math/{Angles, DriveScaling}   time/MatchClock   control/JamDetector
pedro/{Constants, Tuning, procedures/*}          tuning build only for the last two
```

### Teleop at a glance

`Controls` is the source of truth and the init card prints it.

| Pad | Input | Does |
|---|---|---|
| driver | L-stick / R-stick X | drive. Field-centric with the alliance's driver frame and a heading hold once tuned; robot-centric before that |
| driver | L-trigger | precision slow mode |
| driver | **R-trigger (hold)** | **aim lock**: points the rear shooter at the up-CELL while the sticks still translate. Tag-corrected when the camera saw the tags recently. Off in MANUAL. |
| driver | LB / Y / **A** / BACK | drive frame toggle / re-zero heading / **re-seed pose (robot must be on its start)** / abort macro |
| driver | B / RB | path to the shooting spot / to park |
| driver | dpad | snap to 90 / 0 / 270 / 180° |
| operator | RB / LB / X | intake on-off / reverse on-off / stop intake and cancel any macro |
| operator | R-trigger / A | shoot one pulse / shoot four |
| operator | L-trigger | flywheel armed on/off |
| operator | **dpad up** | **MANUAL**: flywheel speed becomes `Shooter.MANUAL_TICKS_PER_SEC` and the aim lock switches off. Odometry, the distance table and the aim law all leave the loop. |
| operator | dpad left / right | trim `MANUAL_TICKS_PER_SEC` by `Teleop.SPEED_TRIM_TICKS_PER_SEC` |
| operator | dpad down | "our HIVE tipped": the aim target flips to the other CELL |

Rumbles, because a driver cannot read telemetry mid-match: **1 blip** a macro succeeded · **3 blips** a
macro failed, or a drive control was refused with no follower · **3 blips on the operator pad** the
intake has jammed and anti-jam has given up — reverse it by hand · **one long buzz on both** 20 s left.

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
| **Alliance is the OpMode class** | `BlueTeleop`/`RedTeleop`/`BlueAuto`/`RedAuto`, 13 lines each. Deletes `AutoSelector`, the A-locks/B-unlocks ceremony and the whole "started UNLOCKED" failure mode. The side is chosen by picking the OpMode, so it cannot be stale, unconfirmed, or inherited wrong. |
| **MANUAL is a first-class mode, on one button** | Every automatic path is pose-derived — the aim law, the distance table, field-centric drive — so one hard collision breaks all of them at once and no sensor on this robot would notice. Operator dpad-up takes odometry, the table and the aim law out of the loop together, and dpad left/right trims the speed. This is the legal substitute for the dashboard slider an FTC-Dashboard robot would trim with (R704). |
| **Two default commands read a supplier every loop** | `Intake.operatorControlCommand` and `Shooter.armedControlCommand`. Ivy has **no duplicate guard**, so re-scheduling an intent on each press re-runs `start()` every loop; and Ivy *ends* rather than suspends a preempted priority-0 command, so a press-scheduled hold needs restoring by hand after every shot. At priority −1 with `SUSPEND` a shooting cycle preempts them and they resume by themselves. |
| **The shooting cycle is one state machine, not an Ivy group tree** | Ivy's `Sequential` hands off one child per `execute()` and never executes the child it just started, so every boundary costs a whole ~20 ms loop and an `instant` costs one by itself. The old nine-level tree spent 7 idle loops between "flywheel recovered" and "next pulse". A state machine chains every zero-duration transition inside one `execute()`. It also sidesteps the `Repeat.end()` NPE for free. |
| **`Shoot` refuses to run with a zero flywheel target** | `atTarget()` is `|target − velocity| < tolerance`, so a zero target is "at speed" on tick one and the tunnel would push the whole load through a dead wheel. |
| **Benches hold no motor handles of their own** | A second `DcMotorEx` on a port the subsystem already holds is a second power cache, and `Robot.stopMechanisms()` then cannot stop it — a bench could leave a flywheel spinning after the OpMode ends. `Shooter.setOpenLoopPower` is a *mode*, so `update()` stays the only writer. |
| **`SelfTest` does not check wheel direction** | The SDK applies `Direction` to commanded power and reported velocity together, so a backwards wheel reads positive. Directions are **Utility → TestHardware**'s job. The card says so rather than implying a check it cannot make. |
| **Every wait has a deadline (`Waits.bounded`)** | The reference codebase writes `waitUntil(shooter::atTarget)` with no timeout; a dead flywheel hangs its autonomous for the rest of the match. One `race` against a timer is the whole fix. |
| **Every macro reports a terminal `Outcome`** | A timeout or an abort used to leave it `RUNNING` for ever. Built as `deadline(sequential(...), onInterrupt(markCancelled))` — **not** `setEnd` on the group, because Ivy's `setEnd` *replaces* the group's own end, the one that ends its children. |
| **`FieldConstants` rotates, it does not mirror** | The BIOBUZZ field is 180° rotationally symmetric: GARDEN A1/F6, LOADING ZONE A5/F2. A C/D mirror would put them on F1 and F5 and every blue auto in the wrong place. The enum with three arms is gone — a run-time selector over a fact the field cannot change is just somewhere for a wrong value to hide. |
| **SDK 11.2.1 is fine** | The docs used to defer a 12.0 upgrade for the AprilTag *cluster* API. The tags are read by the Limelight's own pipeline over USB-Ethernet, not by the SDK's `AprilTagProcessor`, so nothing in the confirmed hardware needs it. It is a future task, not a prerequisite. |
| **No Panels, no Dashboard; AutoTune behind `-Ptuning`** | R704 prohibits streaming tools during matches, and AutoTune's HTTP server is bound whenever the library is on the classpath. |

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
- `*WasPressed()` consumes its flag on read. Read every edge once per loop (`Controls.read`), and
  `resetEdgeDetection()` in `start()` — `MatchOpMode` does it.
- Iterative OpModes may not write motors from `stop()`; the SDK zeroes them itself.
- `Direction` is applied to commanded power *and* reported velocity, so a reversed motor reads positive.
- **Utility → TestHardware** spins any motor by config name. First check at every event.

**Game rules** (docs/04)
- G407: at most 4 controlled scoring elements. **Nothing on this robot enforces it — the operator does.**
- G408: never control the opponent's NECTAR. There is no colour sensing, so this is also the operator's job.
- R503: 8 motors and 8 servos max. We use 7 motors, 0 servos.
- R704: no dashboard or streaming tools in matches. Competition APK is plain `assembleDebug`.
- AUTO 30 s → 8 s no-motion transition (G403) → TELEOP 120 s. No endgame. FLOWER unlock at 1:00.

## 8. On the real robot, in this order

1. **Robot Controller configuration** names must equal `HardwareNames`: `front_left_drive`,
   `front_right_drive`, `back_left_drive`, `back_right_drive`, `intake`, `shooter_left`,
   `shooter_right`, `pinpoint`, `limelight`. A wrong name throws at init and names itself.
2. **Utility → TestHardware**: spin each of the seven motors by name. Each drive motor must push the
   robot forward — fix any reversed one in `Constants.drivetrainConfig`. Confirm the two flywheels
   **counter-rotate**.
3. **`SelfTest`**: every row PASS, or WARN on the follower row before AutoTune has run.
4. **`-Ptuning` build → AutoTune** at `http://192.168.43.1:10158`: Mecanum Tuner → `drivetrainConfig`;
   Pinpoint Tuner → `localizerConfig`; Foresight Tuner → `foresightConfig`. Paste all three. `create()`
   then returns a follower and the heading hold, aim lock, snap turns and paths all come alive.
5. **`Bench: Intake`**: one clean capture (note the peak current), then anti-jam off and a deliberate
   stall (note that peak). `STALL_CURRENT_AMPS` goes between them; `STALL_TIMEOUT_MS` must outlast a
   clean capture's longest over-threshold run. Confirm one motor turns **both** roller and tunnel.
6. **`Bench: Shooter`**, the four steps on the card: `kS` (lowest power the wheel still turns at),
   `kV` (steady power ÷ settled velocity), `kP` (fire a piece with RB and trim until recovery is quick
   without overshoot), then the distance table from 24/48/72/96 in into `DISTANCES_INCHES` /
   `TICKS_PER_SEC`. Set `MANUAL_TICKS_PER_SEC` to whatever scores from your usual spot.
7. **Teleop on the practice field**: drive, arm, fire — one pulse should move one piece. Face the HIVE,
   pull the aim lock, turn away, and confirm the card still says `tag-corrected`. Then press dpad-up and
   confirm MANUAL really takes the table and the aim law out.
8. **`Auto BLUE`** with the 30 s clock. Set `Auto.LEAVE_POWER` / `LEAVE_MS` so the robot clearly stops
   touching the wall and never reaches the HIVE. Then check teleop inherits the pose.
9. **Before every match**: build the **plain** APK. The tuning build is not match legal and nothing
   warns you any more.

## 9. Constants to measure

| File | Constant | How |
|---|---|---|
| `Shooter` | `kS`, `kV`, `kP` | `Bench: Shooter` steps 1–3 |
| `Shooter` | `DISTANCES_INCHES` / `TICKS_PER_SEC`, `MANUAL_TICKS_PER_SEC`, `TOLERANCE_TICKS_PER_SEC` | `Bench: Shooter` step 4, from real shots |
| `Shooter` | `SECOND_MOTOR_REVERSED` | `Bench: Shooter` X flips it live; matched signs at the same power |
| `Shooter` | `HEADING_OFFSET_RAD` | π = fires out the rear. Confirm on the built robot |
| `Intake` | `IN`, `OUT`, `IDLE` | `Bench: Intake`: enough to carry pieces without grinding them |
| `Intake` | `STALL_CURRENT_AMPS`, `STALL_TIMEOUT_MS` | `Bench: Intake`, two runs (§8 step 5) |
| `Shoot` | `FEED_PULSE_MS`, `RECOVER_TIMEOUT_MS`, `FINAL_DWELL_MS` | `Bench: Shooter` RB: one pulse must move exactly one piece |
| `Drivetrain` | `HEADING_HOLD_P/I/D`, `HEADING_HOLD_MAX_TURN` | on the field, once tuned |
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
