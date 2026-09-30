# Software Architecture

How the BIOBUZZ code is organised, and why. **Scope: the robot in `02-robot-physical-architecture.md`** —
seven motors, a front camera, no piece sensors, a shooter bolted to the chassis firing out the rear.
Library facts are in `01-libraries-pedro-3.0.0-ivy-1.1.1.md`; season facts in
`04-biobuzz-season-analysis.md`; status, bring-up order and the numbers still to measure in `HANDOFF.md`.

The Javadoc header of each class is its contract. This document is the map, and it is the place where
cross-class rules live — the ones no single file can state.

**The shape of the thing:** four subsystems, two command files, one OpMode base, one `Robot`. Nothing
arbitrates between subsystems, nothing counts, nothing fails soft except the camera, and every
mechanism writes hardware in exactly one method.

---

## 1. The loop contract

Every OpMode — match or bench — extends `opmodes/MatchOpMode`, whose `loop()` is **final**:

```java
public final void loop() {
    robot.readSensors();                          // 1. observe  — one consistent snapshot
    onDecide();                                   // 2. decide   — read gamepads, schedule commands
    if (usesScheduler()) Scheduler.execute();     //    driver control and macros both run here
    robot.writeActuators();                       // 3. act      — every mechanism writes hardware
    onAfterAct();
    onTelemetry();
}
```

`Robot` has `readSensors()` and `writeActuators()` and **deliberately no `update()`** that does both: a
single method would let a caller put observe-and-act on the same side of the scheduler, and every
command would then decide on last loop's data. The bug would look like a tuning problem. The order is
structural, not a convention — a subclass cannot reorder it.

Lifecycle (all five SDK methods are `final`):

| SDK | `MatchOpMode` does | Hook |
|---|---|---|
| `init()` | `Scheduler.reset()` → `new Robot(hardwareMap, alliance())` → telemetry interval → hook | `onInit()` |
| `init_loop()` | `robot.readSensors()` → hook | `onInitLoop()` |
| `start()` | `gamepad1/2.resetEdgeDetection()` → `robot.startMatch(matchPeriod())` → `robot.drivetrain.onStart()` → hook | `onStart()` |
| `loop()` | above | `onDecide()`, `onAfterAct()`, `onTelemetry()` |
| `stop()` | hook → `Scheduler.reset()` → `robot.stop()` | `onStop()` |

The subclass contract is three methods: `alliance()`, `matchPeriod()`, and `usesScheduler()` (false for
a bench). `Scheduler.reset()` is first in `init()` because the scheduler is static and survives OpMode
restarts. `Robot.stop()` releases non-actuator hardware only — the SDK rejects motor writes from an
iterative OpMode's `stop()` and zeroes the motors itself. The SDK transmits telemetry after every
`init_loop()` and `loop()`, so nothing calls `telemetry.update()`.

**A bench shares this base rather than duplicating it.** It wants the same `Robot`, the same config
names and the same write path, but no scheduler: nothing else is running, so nothing re-asserts idle,
and a bench that stops calling a mechanism calls its `stop()`. `usesScheduler()` returning false is the
whole difference. This used to be a second, near-identical base class.

## 2. Package tree

`TeamCode/src/main/java/org/firstinspires/ftc/teamcode/`

```
Robot.java                     composition root: 4 public final subsystems + macros, readSensors()/
                               writeActuators(), loop Hz, battery, abortMacro(), stopMechanisms()
subsystems/
  Drivetrain.java              ONE Mecanum + an optional Follower: sticks, heading hold, aim lock, paths
  Intake.java                  roller AND tunnel on one motor: in/out/idle/off, JamDetector
  Shooter.java                 2 opposed motors, kS/kV/kP on setPower, distance→ticks/sec table
  Limelight.java               the target CELL's tag tx, for the aim correction, and nothing else
commands/
  Macros.java                  shootOne/shootAll/aimAndShootAll/aimAt/snapToHeading/driveTo; the aim law
  Shoot.java                   the shooting cycle as one state machine
  Waits.java                   waitMs + bounded (the every-wait-has-a-deadline rule)
opmodes/
  MatchOpMode.java             THE one OpMode base; benches included (usesScheduler() = false)
  teleop/
    Teleop.java                the match TeleOp, abstract; MANUAL override; aim lock on R-trigger
    Controls.java              enum of every binding + the generated help card; one Snapshot per loop
    BlueTeleop.java            @TeleOp "Teleop BLUE"  — 13 lines: the annotation and the side
    RedTeleop.java             @TeleOp "Teleop RED"
  auto/
    Auto.java                  the 30 s routine, abstract; tuned and untuned variants
    BlueAuto.java              @Autonomous "Auto BLUE", preselects Teleop BLUE
    RedAuto.java               @Autonomous "Auto RED", preselects Teleop RED
  test/
    SelfTest.java              "SelfTest": PASS / WARN / FAIL per mechanism; run first at every event
    IntakeBench.java           "Bench: Intake": every mode turns the right way, current, anti-jam
    DriveBench.java            "Bench: Drive": deadband/expo/slow, field vs robot centric, heading hold
    ShooterBench.java          "Bench: Shooter": live kS/kV/kP in FTC Dashboard
    LimelightBench.java        "Bench: Limelight": alliance tags → distance, flywheel speed, aim, pose
game/
  Field.java                   the field frame (origin A1), tiles, HIVE CELLs, tag ranges, LOADING ZONE
  FieldPoses.java              BLUE start / shooting spot / park (placeholders)
util/
  field/Alliance.java, FieldConstants.java (SYMMETRY = ROTATE_180), PoseStorage.java (the pose, only)
  hardware/HardwareNames.java  every config string, and nothing else
  math/Angles.java, DriveScaling.java
  time/MatchClock.java
  control/JamDetector.java
pedro/
  Constants.java               MecanumConfig + PinpointConfig + ForesightConfig; createMecanum(), create()
  Tuning.java                  @Tuner factories (AutoTune)
  procedures/*.java            the four Quickstart AutoTune procedures, untouched
```

37 files, 6,834 physical lines, of which 2,100 are `pedro/procedures/**` (Quickstart code). There is no `src/test/` tree — see §17.

## 3. Dependency rules

```
opmodes ─────────────► Robot, subsystems, commands, game, util, Pedro, Ivy, SDK
opmodes.{auto,teleop,test} ─► opmodes.MatchOpMode (never each other)
Robot ───────────────► subsystems, commands.Macros, util.field.Alliance, util.time.MatchClock, SDK
commands ────────────► Robot (back-reference), subsystems, util.math, Pedro paths/math, Ivy
subsystems ──────────► util.*, pedro.Constants (Drivetrain only), Pedro, Ivy, SDK hardware
game ────────────────► util.field, Pedro math.Pose
util.field ──────────► util.math.Angles, Pedro math.Pose
util.math, util.time, util.control ──► java.* ONLY
util.hardware ───────► nothing at all (string constants)
pedro ───────────────► util.hardware, Pedro
```

Rules that are worth stating because breaking them is easy:

- Nothing in `util/math`, `util/time` or `util/control` imports hardware or a library. That is what
  keeps them arguable on their own.
- `game/` is the only place the season is named. A subsystem says "piece", never "POLLEN".
- `pedro/` is the only place a `Follower` or a `Mecanum` is constructed.
- `HardwareNames` is the only place a config string appears — including for the drivetrain, which is
  why `pedro/Constants` points at it rather than retyping four names.
- A subsystem never imports another subsystem. There is no cross-subsystem wiring left to do: the
  suppliers that used to carry it (`intake.setFullSupplier(storage::isFull)` and friends) went with the
  sensors that fed them.

## 4. `Robot`: the composition root

```java
public final Drivetrain drivetrain;  public final Intake intake;
public final Shooter shooter;        public final Limelight limelight;
public final Macros macros;          public final Alliance alliance;
```

Construction, in order: every `LynxModule` to `BulkCachingMode.MANUAL` → copy the voltage sensors out
of the `DeviceMapping` once → the four subsystems → `new Macros(this)` last, because it takes `this`.

- **`readSensors()`** clears every hub's bulk cache, calls `limelight.update()` (one frame, cached),
  ticks the `MatchClock`, samples the battery at most every `VOLTAGE_SAMPLE_MS` (250; voltage is *not*
  bulk-cached, so each read is its own transaction) and tracks the loop time over `LOOP_AVERAGE_OVER`
  (10) loops. Clearing the caches here is what makes the whole loop see one snapshot.
- **`writeActuators()`** is `intake` → `shooter` → `drivetrain`, drivetrain last because its `update()`
  is the one `follower.update()` per loop.
- **`alliance` is `final`** and comes from the OpMode class, so it cannot be stale or unconfirmed (§5).
- **`MatchClock` is null before `startMatch()`.** Every caller null-checks; `init_loop` runs before it
  exists.
- **`abortMacro()`** is the shared cancel path: `drivetrain.cancelPath()` then `macros.markCancelled()`.
  Ivy cannot stop a follower that has been handed a path — the follower drives itself — so cancelling
  the command is not enough.
- **`stopMechanisms()`** stops the intake, disarms the shooter, cancels the path and then writes, for
  the autonomous buzzer safety net (G403) and `SelfTest`'s `finally`.

Subsystem conventions:

- **`update()` is the only method that writes hardware.** Commands set intent; `update()` applies it,
  once per loop, from `writeActuators()`. Every writer also writes **only on change** — an unchanged
  `setPower` is still a bus transaction.
- **Subsystems own their commands.** `*Command()` factories return `com.pedropathing.ivy.Command` with
  `.requiring(this)`.
- **Arbitration is Ivy's job, not a flag's.** Default commands sit at priority −1 with
  `InterruptedBehavior.SUSPEND` and `BlockedBehavior.QUEUE`; anything requiring the subsystem preempts
  them and they resume by themselves. Their logic lives in `setExecute`, because the Scheduler's resume
  path re-adds a suspended command **without** calling `start()` again.
- Tunables are plain `public static` fields. Edit and redeploy, or nudge one on a bench — and a static
  lives until the Robot Controller app restarts, which is a trap worth knowing (§18). There is no
  snapshot-and-report machinery any more; it went with the benches that wrote to it.
- Every constructor is `(HardwareMap[, names])` and resolves its own devices. There is no injection
  seam, because there is nothing left to inject into: no clock, no fake motors, no tests.

## 5. OpModes: one base, and the alliance in the class name

**The Driver Station list:**

| OpMode | Annotation | Group |
|---|---|---|
| `Teleop BLUE` / `Teleop RED` | `@TeleOp` | Main |
| `Auto BLUE` / `Auto RED` | `@Autonomous`, `preselectTeleOp` set to the matching teleop | Main |
| `Bench: Intake`, `Bench: Drive`, `Bench: Limelight`, `SelfTest` | `@TeleOp` | Bench |
| `Bench: Shooter` | `@TeleOp` | Bench |

**Alliance is baked into the OpMode class.** `BlueTeleop`, `RedTeleop`, `BlueAuto` and `RedAuto` are
~13-line subclasses that carry the annotation and pass a side to the abstract parent. This is the
single largest simplification in the OpMode layer, and it deletes:

- `AutoSelector` — the dpad menu, the A-locks / B-unlocks ceremony, and the edge reads it needed;
- the `NOT LOCKED` blink and the `started UNLOCKED` running warning;
- the rule that a path-based autonomous may only drive when `isConfirmed()`;
- the dpad alliance override in teleop init, and the "alliance inherited from whatever ran last"
  failure mode that made it necessary.

**There is now no way to run the wrong route.** Choosing the OpMode *is* choosing the alliance, at the
moment a human reads the screen. `PoseStorage` consequently carries the pose and nothing else.

**`Controls`** — one enum entry per binding: `(Pad, button, description, edge predicate | axis function,
Needs)`. The button, the label and the description are one object, and `helpLines()` renders the init
card from the same constants `Teleop` reads, so the card cannot drift from the code. The SDK's
`*WasPressed()` **consumes its flag on read**, so `Controls.read(gp1, gp2)` takes one immutable
`Snapshot` per loop and `Teleop` decides from that; a press during a macro is consumed that loop instead
of firing the moment the macro ends. A control that needs the tuned follower declares
`Needs.DRIVETRAIN`, and `Teleop` refuses those generically with
`Snapshot.anyPressed(Controls::requiresDrivetrain)` — so a new macro cannot slip past the gate by being
left off a hand-written list.

| Pad | Input | Does |
|---|---|---|
| driver | L-stick / R-stick X | drive — field-centric with the alliance's driver frame and a heading hold once Pedro is tuned; robot-centric before that |
| driver | L-trigger (hold) | precision slow mode |
| driver | **R-trigger (hold)** | **aim lock**: the heading hold points the rear shooter at the up-CELL while the sticks still translate. Off in MANUAL |
| driver | LB / **Y** / BACK | field ↔ robot centric / **re-zero the field heading** (face away from your wall) / abort macro (moving a stick also aborts a drive macro) |
| driver | **A** | **re-seed the pose** — the robot must physically be on its start line |
| driver | B / RB | Pedro path to the shooting spot / to park |
| driver | dpad | snap to 90 / 0 / 270 / 180° |
| operator | RB / LB / X | intake on-off / reverse on-off / stop intake (X also cancels any macro) |
| operator | R-trigger / A | shoot one (one tunnel pulse) / shoot four (`Macros.PIECES_PER_LOAD` pulses) |
| operator | L-trigger | flywheel armed on / off |
| operator | **dpad up** | **MANUAL**: flywheel speed from `Shooter.MANUAL_TICKS_PER_SEC`, aim lock off |
| operator | dpad left / right | manual speed − / + `Teleop.SPEED_TRIM_TICKS_PER_SEC` (100 t/s per press) |
| operator | dpad down | "our HIVE tipped": the aim target flips to the other CELL |

Driver A and Y are the two recovery buttons and they are different: **Y** treats the current facing as
"away from my wall" and keeps x/y; **A** rewrites the whole pose to the start pose, which is only true
if the robot is actually standing there.

**`Teleop`** — `onInit()` schedules exactly three defaults (`drivetrain.driverControlCommand` reading
shaped sticks, `intake.operatorControlCommand`, `shooter.armedControlCommand`), sets the driver frame
from `Field.driverForwardHeading(side)`, and applies `PoseStorage`'s pose if autonomous left one.
`onDecide()` takes one `Snapshot`, handles driver then operator, recomputes the flywheel target, and
holds or releases the aim lock. `onAfterAct()` rumbles on **transitions** only — one blip for a macro
success, three for a timeout or a refused press, three on the operator pad when anti-jam gives up, and
one long 600 ms buzz at 20 s remaining, deliberately unlike any other pattern. A driver cannot read
telemetry mid-match, so this is the whole output channel while the robot is moving.

**The MANUAL override is a first-class feature, not a fallback.** Operator dpad-up switches the flywheel
to `Shooter.MANUAL_TICKS_PER_SEC` (trimmed live on dpad left/right) and switches the aim lock off, so
the driver aims by hand. Odometry, the distance table and the aim law all leave the loop in one press.
It exists because **every automatic path here is pose-derived** — the aim law, the distance lookup, the
snap turns, the paths — so one hard collision can make all of them wrong at the same moment, and there
is no sensor anywhere on this robot that would notice. Two presses of dpad-up and the robot is back to
automatic; driver A re-seeds the pose and driver Y re-zeroes the heading when the collision is worth
recovering from properly.

**`Auto`** — 30 seconds, two routines chosen by `drivetrain.hasFollower()`:

| | Tuned | Not tuned |
|---|---|---|
| pose | `setPose(BLUE_START_FACING_HIVE` rotated for the side`)` | none — there is no localizer |
| speed | `setTargetForDistance(start.distance(target))` | `setManualTarget()` |
| body | `bounded(aimAndShootAll(cell, tags), SHOOT_BUDGET_MS)` → `waitMs(SETTLE_MS)` → `driveTo(BLUE_PARK)` | `bounded(shootAll(), SHOOT_BUDGET_MS)` → `waitMs(SETTLE_MS)` → `driveForMsCommand(LEAVE_POWER, 0, 0, LEAVE_MS)` |
| worth | 20 (TIP) + 3 (LEAVE) + 5 (PARK) | 20 + 3 |

Both end in `instant(robot::stopMechanisms)`. `onDecide()` is the buzzer safety net: once
`MatchClock.isExpired()`, `abortMacro()` and `stopMechanisms()` — a macro still inside its timeout does
not know the match is over, and G403 forbids powered movement after the period ends. `onAfterAct()`
writes `PoseStorage` **every loop**, not once at the end, so a cut or disabled autonomous still hands
teleop a pose.

The placement card says it plainly: **touching your wall, rear (shooter) toward the up-facing CELL**.
The first TIP needs only three POLLEN, because the up-CELL starts with three NECTAR and a HIVE tips on
3 + 3 (docs/04 §5.3) — which is why this routine spends its budget shooting rather than driving.

## 6. Subsystems

| Subsystem | Hardware | Contract | Commands |
|---|---|---|---|
| `Drivetrain` | one Pedro `revhub.drivetrains.Mecanum`, plus a `Follower` when `Constants.create()` can build one | `drive(f, s, t)`, `getPose()`/`setPose()`/`getPoseWrites()`, `hasFollower()`, heading hold with an external aim-lock setpoint (`setAimLock(DoubleSupplier)`), `holdHeading(rad)`, `atPose`/`atHeading`, `cancelPath()`, `resetHeading()`, field/robot-centric, `isLocalizerSettled()` | `driverControlCommand(f, s, t)` (default, −1/SUSPEND/QUEUE), `followLazyCommand(Supplier<Path>, holdEnd)`, `turnToCommand(rad)`, `driveForMsCommand(f, s, t, ms)` |
| `Intake` | 1 `DcMotorEx`, **open-loop power**, BRAKE | `Mode {OFF, IN, OUT, IDLE}` — the roller and the tunnel are one motor, so one mode covers both. `getCurrentAmps()` (NaN unless pulling), `isStallSuspected()`, `isUnjamming()`, `hasGivenUpUnjamming()` | `operatorControlCommand(Supplier<Mode>)` (default), `inCommand`/`outCommand`/`idleCommand`, `defaultIdleCommand` |
| `Shooter` | 2 `DcMotorEx`, **open-loop power**, FLOAT, default run mode | armed or off — no idle speed. `setTarget`/`getTarget` in **ticks/sec**, `getVelocity()` from one motor, `atTarget()` (one symmetric comparison), `setTargetForDistance(in)`, `setManualTarget()`, `HEADING_OFFSET_RAD` | `armedCommand()` (never done; owned for the length of a shot), `armedControlCommand(BooleanSupplier)` (default), `defaultIdleCommand` |
| `Limelight` | `Limelight3A` over USB-Ethernet | `getTagTx(minId, maxId)` — the mean tx of the visible tags in that ID range, NaN when none; `hasTarget()`, `isStale()`, `isConnected()`, `getTagCount()`, `getStatus()` | none: it is a data source |

Two things this table does not contain, and they are the point: there is **no positional mechanism, no
servo, and no velocity-PID wrapper**. Both mechanisms are open-loop power. `subsystems/templates/` is
gone, because a template shared by one class is not a template.

**Why both mechanisms use a default command that reads a supplier every loop.** `Intake.operatorControlCommand(Supplier<Mode>)`
and `Shooter.armedControlCommand(BooleanSupplier)` are scheduled once, at init, and re-read the
operator's wish in `setExecute`. They are **not** re-scheduled on each press, for two library reasons:

1. **Ivy has no duplicate guard.** Scheduling the same intent every loop would re-run `start()` every
   loop (docs/01 §B.2).
2. **Ivy *ends* rather than suspends a preempted priority-0 command.** A hold scheduled on the press
   would have to be restored by hand after every shot — which is exactly what the previous code did,
   with a `restoreFlywheelHold()` call that had to be remembered in every path out of a macro.

At priority −1 with `SUSPEND`, a shooting cycle preempts them and they resume by themselves, still
holding whatever the operator last asked for. The rest state is enforced continuously, so a missed edge
cannot leave a mechanism running.

## 7. The drivetrain: one `Mecanum`, shared

This is the decision that deleted three classes, and it rests on one verified library fact.

`com.pedropathing.revhub.drivetrains.Mecanum` (verified in `revhub-3.0.0-sources.jar`):

- its constructor is `Mecanum(HardwareMap, MecanumConfig)` — it resolves the four `DcMotorEx` **itself**,
  from the names in the config;
- it wraps each one in its **own `CachedMotor`**, which suppresses writes below
  `MecanumConfig.powerThreshold`;
- it implements Pedro's `Drivetrain` interface, and `drive(DrivePowers, boolean manual)` is **public**.

So two `Mecanum` objects over the same four motors hold **two independent power caches**, each believing
it knows what the motor was last told, and they fight. *That* — and only that — is why the old code
carried a doctrine of "one motor layer at a time", a separate `OpenLoopDrive` class for the untuned
case, a rule in `Robot` about which one to build, and a rule in `Teleop` about scheduling exactly one
drive default.

`Drivetrain` now builds **one** `Mecanum`, always, from names and directions alone, and hands **that
same instance** to the `Follower` if `Constants.localizerConfig` and `foresightConfig` have been filled:

```java
this.mecanum  = Constants.createMecanum(hardwareMap);      // never null
this.follower = Constants.create(hardwareMap, mecanum);    // null until AutoTune has run
```

| | With a follower | Without |
|---|---|---|
| sticks | `follower.manual(...)`, field-centric against the Pinpoint heading | `mecanum.drive(powers, true)`, robot-centric — there is no pose to rotate against |
| heading hold, aim lock | live | off |
| paths, snap turns, pose re-seed | live | the command factories return a command that finishes on tick one, and `Teleop` refuses the press with three blips |
| `driveForMsCommand` | works | **works** — which is what lets the untuned autonomous leave the wall |

`OpenLoopDrive`, `PathFollower` and `PedroPathFollower` are all deleted. The seam interface went with
the tests it existed for (§17); `Drivetrain` now talks to `com.pedropathing.follower.Follower` directly
and `getFollower()` hands it out for introspection.

The heading hold itself is unchanged and worth re-reading before touching:

- it is fed an **error**, never a raw angle, so the 0/2π seam cannot produce a full-speed spin the long
  way round;
- **it never fights the driver**: any turn input above `HEADING_HOLD_STICK_DEADBAND` hands control
  straight back and the heading is re-captured on release. That deadband is **0.001**, compared against
  the *shaped* stick — 0.05 there threw away raw deflections up to ~0.28;
- an aim lock supplies the setpoint every loop; NaN from it means "no opinion", which falls back to
  capture-and-hold;
- **any `setPose` releases the hold**, because the setpoint was captured in the old heading frame and
  keeping it across a frame change rotates the robot by the size of the correction.

A pose written within `LOCALIZER_SETTLE_MS` (1000) of construction lands during the Pinpoint's IMU
calibration and is lost (docs/01 A.9 gotcha 6), so `update()` repeats it once the window has passed.

## 8. The shooter: feedforward and a P term, in ticks per second

The control scheme changed completely in this rebase, and the reasons are worth keeping.

```java
// Shooter.update(), once per loop, from Robot.writeActuators()
if (armed) write(kV * target + kP * (target - getVelocity()) + kS);
else       write(0);
```

- **No `RUN_USING_ENCODER`, no `setVelocity`, no `setPIDFCoefficients`.** The motors are left in their
  default run mode. The hub's velocity loop **integrates**, and on a belted flywheel pair that is
  unloaded except for the instant a piece crosses the wheels, the integrator is exactly what makes
  recovery slow and then overshoot.
- **Everything is in encoder ticks per second** — the unit `getVelocity()` returns. There is no
  `TICKS_PER_REV` and no RPM anywhere in the class, because these are not goBILDA motors and nobody has
  counted their ticks per revolution (docs/02 §2). A measured t/s figure cannot be wrong about itself.
- **It is feedforward-dominant with a P term that saturates almost immediately.** `kV * target` carries
  the steady-state speed, `kS` pays for belt and bearing drag, and `kP` (0.01 per tick/sec of error)
  reaches full power at 100 t/s of error. So the wheel holds a known-good open-loop power and slams to
  full on any sag.
- **That is why there is no recovery wait between shots and no at-speed dwell latch.** `atTarget()` is
  one symmetric comparison against `TOLERANCE_TICKS_PER_SEC` (50). The old shooter needed the band held
  for 100 ms before releasing a shot because the hub's PID kept wandering back out; this scheme is back
  in band within a couple of loops, so a dwell would only add its own length to every shot.
- `atTarget()` says **nothing** about being armed — a stopped wheel with a target of 0 is at target — so
  anything that gates a shot on it checks `isArmed()` too, and `commands/Shoot` refuses to run at all
  when the target is ≤ 0 (§10).
- **One encoder is read, not two.** The pair is belted to the same piece; averaging two buys a second
  bus read and a number that belongs to neither wheel.
- `SECOND_MOTOR_REVERSED` is **geometry, not an option**: the flywheels oppose each other across the
  piece, so one runs reversed and both then take the same power. It is a constant only so that a swapped
  pair of leads can be answered in one place, and it is read once at construction.
- The wheel is armed or off. **There is no idle speed**, because a wheel held at a speed nobody shoots
  at is heat and noise. `armedCommand()` owns the resource for as long as a shot needs;
  `defaultIdleCommand()` disarms whenever nothing owns it. `armedCommand()`'s end deliberately does
  **not** disarm — the next owner takes over inside the same `Scheduler` pass, so an armed wheel is
  never zeroed across a shot.

### The distance table, and why it is not a curve fit

```java
private static final double[] DISTANCES_INCHES = {24, 48, 72, 96};
private static final double[] TICKS_PER_SEC    = {1150, 1250, 1400, 1550};
```

Linear between the measured points, **flat outside them**. Every number above is a placeholder;
`Bench: Shooter` is how they become real (park at a distance, raise the speed until the piece drops in
the middle of the CELL, write the pair down, move on).

This whole scheme — `kS = 0.08`, `kV = 0.00039`, `kP = 0.01`, the ±50 t/s tolerance, the table — is taken
from a competition-proven reference codebase for the same wheel size. **That codebase's own shooter file
preserves its abandoned attempts in comments: a linear fit, then a quartic, then a quadratic**, before
it settled on an interpolated table for the pose-derived shot. That history is the reason not to
re-derive a polynomial here. A fit is smooth where the shot is not, and it extrapolates past the last
point anyone measured with total confidence; `interpolate()` clamps at both ends instead, which is the
honest answer outside the measured range.

`setTargetForDistance(NaN)` leaves the target alone: with no pose to measure from, the last trusted
speed beats a speed derived from nothing.

## 9. `commands/Macros`, and the aim law

Multi-subsystem, one-button, bounded, reporting.

Every macro is built through `reporting(name, body, outcome, alongside...)`, which is

```java
deadline(sequential(begin(name), body, finishWith(result)), onInterrupt(this::markCancelled), alongside...)
```

so a `Waits.bounded` timeout, a `Scheduler.cancel` or an operator abort always leaves a **terminal**
`Outcome {IDLE, RUNNING, SUCCESS, TIMED_OUT, CANCELLED}` rather than `RUNNING` for ever. It is a
`deadline` child and **not** a `setEnd` on the sequential, because Ivy's groups are `CommandBuilder`s
whose `setEnd` *replaces* the group's own end — the one that ends its children (docs/01 §B.5 trap 12).
A natural finish fires the callback too, which is harmless: `markCancelled()` only acts on `RUNNING`, so
a terminal outcome is never overwritten and a late callback from a finished macro cannot blank the name
of one that is now running.

`alongside` commands run in parallel with the whole macro and are ended with it. They start inside
`Scheduler.schedule()`, before the first loop — which is what lets the flywheel hold take over from the
operator's displaced hold with no gap.

| Macro | Composition | Success |
|---|---|---|
| `shootOne()` | `bounded(new Shoot(intake, shooter, 1), SHOOT_ONE_TIMEOUT_MS)` with `shooter.armedCommand()` alongside | every requested pulse ran |
| `shootAll()` | the same with `PIECES_PER_LOAD` pulses and `SHOOT_ALL_TIMEOUT_MS` | same |
| `aimAndShootAll(Pose, minTag, maxTag)` | `sequential(bounded(aimCore, AIM_TIMEOUT_MS), bounded(Shoot, SHOOT_ALL_TIMEOUT_MS))`, flywheel spinning up **during** the aim | same |
| `aimAt(Pose, minTag, maxTag)` | `bounded(aimCore, AIM_TIMEOUT_MS)` | within `AIM_TOLERANCE_DEGREES` |
| `snapToHeading(rad)` | `bounded(drivetrain.turnToCommand(rad), SNAP_TIMEOUT_MS)` | within `SNAP_TOLERANCE_DEGREES` |
| `driveTo(Pose)` | `bounded(followLazyCommand(line from the current pose), DRIVE_TO_TIMEOUT_MS)`; **no path at all** inside `MIN_PATH_INCHES` | within `DRIVE_TO_TOLERANCE_INCHES` |

Success is measured after the fact, against the world, not against what the command believed.
`shootOne` and `shootAll` **never require the drivetrain**, so driving and the aim lock continue through
a shot; autonomous aims explicitly with `aimAndShootAll`, whose aim is bounded — if the robot cannot
settle in time it shoots anyway, because in autonomous a piece kept on board scores nothing and a near
miss might.

`aimCore` is Pedro's `hold` used as a turn primitive, re-issued whenever the wanted heading moves by
more than `AIM_REISSUE_DEGREES` (1°) so that a newly visible tag can refine it, with `setEnd` handing
the sticks back either way.

### The aim law

**The cleverest thing in the codebase, and the one most worth not breaking.**

The flywheel fires out the rear (`Shooter.HEADING_OFFSET_RAD` = π), so *aiming is turning the robot's
back to the CELL*. The camera faces front. Therefore **the tags are visible only while the shooter is
pointed the wrong way.** A front camera looks useless to a rear shooter; the trick is that it is not.

```java
double aimHeading(Pose target, int minTagId, int maxTagId)
```

- With a tag of the target CELL in view, the bearing comes from its `tx`:
  `pose.heading() + CAMERA_YAW_OFFSET − tx` (tx is positive to the right). The difference between that
  and the odometry bearing to the CELL is **remembered** as `aimBias`, along with the time and
  `drivetrain.getPoseWrites()`.
- With no tag in view, the odometry bearing is used **plus that remembered correction**, for as long as
  it is younger than `AIM_BIAS_MAX_AGE_MS` (5 s) and the pose has not been rewritten since.
- Either way `HEADING_OFFSET_RAD` is subtracted last, so the answer is the heading the robot must hold.
- NaN with no pose, or with neither a target nor a tag. Every caller handles NaN as "no opinion".

So the sequence that scores is: **drive up facing the HIVE, pull the aim lock, turn, shoot by the
correction.** The teleop card says `LOCKED, tag-corrected +1.4 deg` while a correction is in force and
`LOCKED, odometry only` otherwise, which is the only way a driver can tell the difference.

Two reasons the correction is allowed to outlive the sighting: odometry *heading* drifts very little
over a few seconds, and the target's placement error does not move at all. Two reasons it is dropped:
age, and `getPoseWrites()` changing — a rewritten pose invalidates anything computed against the old
frame, which is why that counter exists at all.

## 10. `commands/Shoot`: the cycle as a state machine

`SPIN_UP → FEED → SETTLE → RECOVER → DWELL → DONE`, one hand-written `Command`, requiring the **intake
only**.

**Why it is not an Ivy group tree.** `Sequential` hands off one child per `execute()` and never executes
the child it has just started, so **every child boundary costs a whole ~20 ms loop** and an `instant`
costs one by itself (docs/01 §B.5, §B.3). The previous nine-level tree spent 7 idle loops between
"flywheel recovered" and "next pulse commanded" — about 0.7 s across a four-piece run. A state machine
chains every zero-duration transition inside a single `execute()` (a bounded `for` loop over the
switch), so the next pulse is commanded **in the same loop** the flywheel comes back. It also sidesteps
the `Repeat.end()` NPE (docs/01 §B.5 trap 8) for free.

The action for a state is applied on **entry**, which is what makes a transition cost no loop:
`enter(FEED)` calls `intake.in()`, every other state calls `intake.stop()`.

| Constant | Default | What it is |
|---|---|---|
| `FEED_PULSE_MS` | 400 | how long the tunnel runs to carry one piece into the flywheel. Too short leaves it short of the wheel; too long feeds two |
| `SETTLE_AFTER_PULSE_MS` | 80 | quiet time before the recovery check, so the dip has actually started |
| `RECOVER_TIMEOUT_MS` | 900 | a flywheel that has not recovered by then gets the next piece anyway |
| `SPIN_UP_TIMEOUT_MS` | 3000 | past this it gives up rather than holding the intake all match |
| `FINAL_DWELL_MS` | 150 | after the last pulse, long enough for the piece to leave before the wheel is released |

**It refuses to run when `shooter.getTarget() <= 0.`** A zero target makes `atTarget()` true on tick one
(`|0 − 0| < 50`) and the tunnel would push the whole load through a dead flywheel onto the floor. The
caller sets the speed — from the distance table or the manual override — and if it did not, this cycle
does nothing. That is a deliberate silent refusal, and the macro reports `TIMED_OUT` for it.

**It requires the intake for the whole cycle**, including between pulses, so the default idle command
cannot re-assert "stopped" in a gap. It does **not** require the shooter (that is `armedCommand()`
running alongside, §9) and it does **not** require the drivetrain (so the driver keeps translating and
aiming through a shot).

`getShotsFired()` is a count of **pulses**. Nothing on this robot can tell a pulse that moved a piece
from a pulse that moved air, and the telemetry says "pulses" everywhere for that reason.

## 11. `game/`

Everything that changes with the season, and nothing else.

**`Field`** holds the frame — and this is the one place it is written down. Origin at the **A1 corner**
(audience-side red), +X toward column F (the blue wall), +Y toward row 6 (the far wall), inches, heading
in radians CCW from +X normalised to `[0, 2π)` by Pedro. Tile *(column, row)* has its centre at
`(12 + 24·col, 12 + 24·(row − 1))` with A = 0. The check that pins the origin is the 180° rotation:
`(x, y) → (144 − x, 144 − y)` must map the red LOADING ZONE on A5 onto the blue one on F2, and it does.

Beyond the frame, `Field` carries only what the aim law consumes: `HIVE_CENTER`, `CELL_SPACING_INCHES`
(18.8), `HIVE_LATERAL_OFFSET_INCHES` (12.4, **INFERRED**), `cell(alliance, side)`,
`startingUpCellSide`, `upCellSide(alliance, tips)`, `tagRange(alliance, side)`, the LOADING ZONE centre,
and `driverForwardHeading(alliance)`. The GARDEN, the FLOWERS, the inventory counts and the piece
geometry are all in docs/04 and none of them is in the code, because no code reads them. Adding a member
back is cheap; leaving an unread constant in the file whose job is to be the trusted source of field
truth is not.

**`FieldPoses`** is three poses — start, shooting spot, park — authored for **BLUE** and converted with
`FieldConstants.forAlliance`. One source of truth per location means a measurement correction is a
one-line change instead of a hunt for the rotated twin someone forgot. **Every number in it is a
placeholder.**

`game/PieceType` is deleted: there is no colour sensing, nothing classifies a piece, and the hue windows
it carried were never measured.

## 12. `pedro/`

`Constants.java` holds the three configs and the two factories:

| Member | State |
|---|---|
| `drivetrainConfig` | **filled** — four names from `HardwareNames`, left REVERSE / right FORWARD, `manualBrakeMode = true`. Confirm each wheel pushes the robot forward and flip the offending direction here |
| `localizerConfig` | `null` until the Pinpoint Tuner's block is pasted in (there is a template in the Javadoc) |
| `foresightConfig` | `null` until the Foresight Tuner's block is pasted in |
| `createMecanum(h)` | always constructible: names and directions are all `Mecanum` needs |
| `create(h, mecanum)` | `null` while either tuning config is missing; otherwise `new Follower(localizer, mecanum, algorithm)` |

Constructor order is `(Localizer, Drivetrain, Algorithm)` — the Quickstart's own comment has it wrong
(docs/01 A.9 gotcha 1). A direction verified once on the untuned robot carries into the tuned follower,
because the follower drives that very config.

`Tuning.java` registers the four `@Tuner` factories. They are invoked by `TunerScanner` **at Robot
Controller start-up**, so each must be `public static`, zero-arg, declared to return exactly
`Procedure`, and must never throw; a factory whose config is not filled yet returns a `NotReady`
procedure that says what to paste first. `Tuning.java`, `pedro/procedures/**` and the `tuning`
dependency are **always in the build**, as are **FTC Dashboard** and `opmodes/test/ShooterBench`, its
only user. Both libraries bind a web server whenever they are in the APK, so under R704 **the everyday
APK is not match legal**; before an event the two dependency lines and those three sources are removed
by hand (HANDOFF §4). This was a team choice on 2026-09-30: every file in the normal folders and nothing
for Android Studio to get wrong, over two things tried before it — a `-Ptuning` Gradle property (Studio
never passes it, so it showed every tuning source as unresolved) and a `competition`/`tuning` build
variant split with those sources in `src/tuning/java`.

## 13. `util/`

| Class | Purpose | Notes |
|---|---|---|
| `control/JamDetector` | current-based stall detection with bounded un-jamming; time is an argument | over-current must *persist* for `stallTimeoutMs`; healthy current for `healthyResetMs` forgives the attempt count, so the limit bounds *consecutive* failures. The `eligible` argument is load-bearing: pass true only while actively intaking, or holding a piece against a stop reads as a jam |
| `field/FieldConstants` | `FIELD_SIZE_INCHES = 144`, `SYMMETRY = ROTATE_180`, `forAlliance`, `rotate180` (heading transformed too) | the evidence for the rotation is in docs/04 §2.5; a mirror puts every blue autonomous in the wrong place |
| `field/Alliance` | which side we are on, and `opposite()` | fixed by the OpMode class (§5), never carried between OpModes |
| `field/PoseStorage` | the auto → teleop handoff: **the pose, and nothing else** | static, so it survives an OpMode switch but not an RC restart. Teleop must treat a missing pose as normal |
| `hardware/HardwareNames` | every config string | seven motors, `pinpoint`, `limelight`. No sensor names, because there are no sensors |
| `math/Angles` | `normalizeAngle`, `angleError` | `Math.atan2` returns `(−π, π]` and Pedro's `Pose` is `[0, 2π)`; mix them and a controller drives the long way round at full power. Feed controllers an **error**, never a raw angle |
| `math/DriveScaling` | deadband, expo, `shape`, `slowScale` | the deadband rescales so full magnitude is still reachable |
| `time/MatchClock` | BIOBUZZ periods: 30 s AUTO, 120 s TELEOP, `isFinalSeconds()` at 0:20, `isExpired()` at the buzzer | **no endgame period** in BIOBUZZ. The timestamp is passed in, so every time-based decision in one loop agrees with every other |

Deleted from `util/` in this rebase, with the reason: `time/Clock` + `SystemClock` (clock injection —
there are no tests to inject into), `hardware/Hardware` (fail-soft lookup — see §14),
`diagnostics/{Tunables, BuildFlavor, LoopTimer, MatchLogger}`, `math/ColorMath` (no colour sensors),
`field/StartPosition` (one start pose, one autonomous).

**Two of those deletions cost something real, and it is worth being honest about which:**

- `MatchLogger` wrote a CSV row per loop to `/sdcard/FIRST/data/`. Without it, a weak shot at an event
  can only be diagnosed from what is on the driver-station card at the time.
- `BuildFlavor.isTuningBuild()` printed `!! TUNING BUILD` on every match init card when the AutoTune
  library was in the APK. **Stripping AutoTune and Dashboard before an event (R704) is now a human check.**
  HANDOFF §8 says so as its own step.

## 14. Crash loudly, and the one thing that does not

**There is no `Hardware` wrapper, no `isAvailable()` anywhere, and no null-motor guards.** Every lookup
is a bare `hardwareMap.get(...)`. A config name that is wrong or missing throws at OpMode init, names
the device, and stops there.

That is the intended behaviour. The alternative — which this codebase used to implement, with **51**
`isAvailable()` guards — is a robot that boots, drives onto the field, and silently runs with one
mechanism disabled because a cable was loose. A stack trace naming `shooter_right` is a louder and
faster signal in the pit than a telemetry line nobody reads at 9 a.m. It also removes a whole class of
question from every method in every subsystem: *can my motor be null here?* No.

**The one exception is `Limelight`, and the distinction matters.** Its *lookup* is a bare
`hardwareMap.get` like every other. What is wrapped in try/catch is every **call** to the device:
`pipelineSwitch`, `start()`, `getLatestResult()`, `getStatus()`, `stop()`. Those are not config lookups
— they are **synchronous network calls to another computer over USB-Ethernet**, and a camera that is
correctly configured but unpowered, booting, or mid-reboot makes them throw or block on socket timeouts.
Killing the OpMode over that would cost a match for a mechanism the robot can drive and shoot without.

So: **a configuration error crashes; an unreachable camera degrades.** The first failed call sets
`cameraFailed`, which is never cleared — `start()` happens once, at init, so a camera that was not
talking then will not begin on its own — and every card carries `!! no Limelight: odometry aim only`.

## 15. The library rules that shaped this design

The full reference is docs/01. These are the ones that are *visible in the structure of this code*, so
that changing the structure without knowing them will quietly break something.

**Ivy 1.1.1**

| Rule | What it shaped |
|---|---|
| `Sequential` hands off one child per `execute()` and never executes the child it just started | `commands/Shoot` is a state machine, not a group tree (§10) |
| `setEnd` on a group **replaces** the group's own end — the one that ends its children | `Macros.reporting` runs its interrupt work as a `deadline` child, not a `setEnd` (§9) |
| A suspended command is resumed **without** `start()` being called again | every default command's logic is in `setExecute` (§4) |
| Ivy **ends** rather than suspends a preempted priority-0 command | the operator's intake and flywheel wishes are priority −1 defaults reading a supplier, not press-scheduled holds (§6) |
| There is **no duplicate guard**: scheduling the same object twice runs `start()` twice | same |
| `Command.unless()` never finishes (`NOOP` has no `done`) | use `conditional(cond, real, instant(() -> {}))` |
| `Commands.lazy(...)` contributes **no** requirements | always `.requiring(subsystem)` |
| A direct hardware call from a requirement-less `instant` is overwritten by the default command before the write lands | go through a factory that requires the subsystem |
| `deadline` and `parallel` end their unfinished children with the **group's own** end condition | judge by the sensor or the condition, never by `EndCondition` |
| Never put `repeat(...)` in a group that can be interrupted before reaching it — `Repeat.end()` NPEs | §10 |
| `Scheduler` is static and survives OpMode restarts | `Scheduler.reset()` first thing in `init()` |

**Pedro 3.0.0**

| Rule | What it shaped |
|---|---|
| `Mecanum` resolves its own motors and caches each one's power | **one** `Mecanum` per robot (§7) |
| `new Follower(Localizer, Drivetrain, Algorithm)` — the Quickstart comment has the order wrong | `Constants.create` |
| Exactly **one** `follower.update()` per loop; it ticks the localizer itself | `drivetrain.update()` last in `writeActuators()` |
| `atParametricEnd()` is **also true whenever the follower is not following** | keep paths and turns **sequential, never parallel** (§18) |
| `isBusy()` only clears inside a hold | never used as "still following" |
| `PinpointLocalizer`'s constructor starts an IMU calibration | build in `init()`; `Drivetrain` re-applies a pose written inside `LOCALIZER_SETTLE_MS` |
| A path with no heading makes `follow()` throw | `followLazyCommand` catches `path.endPose()` and finishes rather than throwing out of the loop |
| There are **no callbacks**, no `setMaxPower`, no `turnTo` | poll, or use Ivy; a turn is `hold(pose.withHeading(h))`; per-path limits are `path.with(cfg.maxPathSpeed.at(p))` |
| `PedroCommands.follow` has no requirement and no `setEnd` | `Drivetrain.followLazyCommand` exists for exactly that |
| **Ivy cannot stop a follower** that has been handed a path | every path command hands control back in `setEnd`; `Robot.abortMacro()` calls `cancelPath()` first |
| `@Tuner` factories run at RC start-up | `Tuning.NotReady` instead of a null dereference |

**The SDK**

- `*WasPressed()` **consumes its flag on read** and only advances inside `Gamepad.copy()`. Read every
  edge once per loop (`Controls.read`), never twice; `resetEdgeDetection()` in `start()`.
- An iterative OpMode may not write motors from `stop()` — the SDK zeroes them itself.
- **SDK 11.2.1 is fine.** See §16.

## 16. Vision scope, and the SDK version

The Limelight 3A is front-mounted and does exactly one job: the **mean `tx` of the AprilTags belonging
to the CELL the shooter must hit**, which `Macros.aimHeading` turns into a heading correction (§9).
`getTagTx(minId, maxId)` is the whole output; which IDs belong to which CELL lives in `game/Field`.

- Every BIOBUZZ tag is a cluster on the underside of a **moving** HIVE CELL, and the SDK v12.0 notes say
  they are unsuitable for absolute field localisation (docs/04 §3). So there is **no botpose**, no
  MegaTag2, no pose fusion — the pose is the Pinpoint's alone. The whole localisation stack was deleted
  in Round 4 and is not coming back this season.
- There is **no piece detection**. The colour-blob pipeline was deleted with it.
- `MAX_STALENESS_MS` is 250: a result older than that means the camera has stopped delivering frames,
  and `hasTarget()` goes false.
- `CAMERA_YAW_OFFSET_DEGREES` is 0 (front). If the camera ever moves to the rear, set it to 180 and the
  same arithmetic works unchanged.

**The SDK 12.0 upgrade is not on the critical path.** The old docs deferred it "before any AprilTag
work", because SDK 12.0 is where the new `AprilTagClusterDetection` API lives. That reasoning does not
apply to this robot: **the tags are read by the Limelight's own pipeline and arrive over USB-Ethernet as
JSON**, never through the SDK's `AprilTagProcessor`. Nothing in the confirmed hardware needs SDK 12.0.
Upgrade when there is a reason and time to re-test — not before an event, and not as a prerequisite for
anything currently written.

## 17. Verification: what there is, and what there is not

**There is no test suite, and none is planned.** The JVM suite — 305 tests against fakes — was deleted
in commit `0efa320`, along with the seams that existed to serve it: `PathFollower` /
`PedroPathFollower`, `Clock` / `SystemClock`, the `(DcMotorEx, Clock)` terminal constructors, and the
composition constructor on `Robot`. The code is roughly 40% of its previous size and reads as one thing
instead of two; the price is that nothing catches a regression before the robot does.

So verification is two things, and both are on the robot:

1. **The compile gate.** `./gradlew :TeamCode:compileDebugJavaWithJavac` must be clean and
   `assembleDebug` must build. Because every device lookup now throws rather than degrading, a
   large class of wiring mistake also fails loudly at OpMode init rather than silently at the match.
2. **The on-robot sequence** in HANDOFF §8: TestHardware, `SelfTest`, AutoTune, `Bench: Intake`,
   `Bench: Shooter`, teleop on the practice field, then autonomous with the 30 s clock. Every constant
   in HANDOFF §9 is measured by a named step in that list.

The benches were rebased with the rest of the tree and the whole build is green: both APKs build, and
the competition APK contains **zero** AutoTune or FTC Dashboard references across all nineteen dex
files (verified by scanning the dex strings, not by trusting the Gradle exclusion).

**What each bench is for.** `Bench: Intake` proves the subsystem: each mode earns a PASS once it turns
the way its power says, and the peak current is held for the stall threshold. `Bench: Shooter` puts
every `Shooter` static in Dashboard's config panel and graphs target, velocity, error and power, so kS,
kV and kP are tuned live instead of one redeploy per number; Dashboard edits are not saved to source,
so the card prints paste-ready values. `Bench: Drive` runs Teleop's own drive default and shows each
stick raw → shaped → slow-scaled while the deadband and expo are changed on the pad. `Bench: Limelight`
takes the alliance on the dpad in init, keeps only that alliance's tags, and reports the CELL's floor
distance and bearing from the tag's 3D pose, the flywheel speed the table gives for that distance, the
turn to aim the rear shooter, and a field pose worked back from the CELL's `Field` position — a test
number only, since the CELL moves when the HIVE tips (§16). It drives nothing.

**No bench writes a motor from `onStop()`.** The SDK rejects motor writes from an iterative OpMode's
`stop()` and zeroes the motors itself (§1). Benches that change a `public static` in the competition APK
(`Intake.ANTI_JAM_ENABLED`, `DriveScaling`, `Drivetrain.HEADING_HOLD_ENABLED`) put it back in `onStop()`,
so a bench session cannot carry into a match on the same Robot Controller; `Bench: Shooter` leaves its
edits in place because its APK never goes to a match.

**The benches hold no motor handles of their own.** This is worth stating because the first attempt at
them did, and it was wrong in a way that is easy to repeat. A bench that calls
`hardwareMap.get(DcMotorEx.class, "shooter_left")` gets a *second* handle to a port the subsystem
already holds — and therefore a second `CachedMotor`-style power cache. The two disagree, and worse,
`Robot.stopMechanisms()` goes through the subsystem, so it can no longer stop a wheel that was written
behind the other cache: the bench can leave a flywheel spinning after the OpMode ends. The fix is that
`Shooter` exposes what a bench needs — `setOpenLoopPower` (an explicit mode, so `update()` stays the
only writer), `getSecondVelocity`, `getLastWritten`, `tableTargetFor`, `applySecondMotorDirection` —
and every bench drives the subsystem. Same rule as §7's single `Mecanum`, for the same reason.

**`SelfTest` does not claim to check wheel direction.** The SDK applies `Direction` to the commanded
power *and* to the reported velocity together (`DcMotorImplEx.adjustAngularRate`), so a motor wired
backwards still reports a positive velocity when driven forwards. It catches a dead motor, lead,
gearbox or encoder cable, and it says on the card that direction is not checked. Directions are the
SDK's **Utility → TestHardware** job, or the Mecanum Tuner's.

## 18. Practical notes

Things that are correct in the code and will still surprise someone standing next to the robot.

- **Nothing counts pieces.** Shoot One is one tunnel pulse; Shoot All is four. `getShotsFired()` is a
  pulse count. The operator is the only thing that knows whether the robot is empty, and the only thing
  that enforces G407's limit of four.
- **Feeding the shooter is running the intake.** One motor drives the roller and the tunnel, so the
  intake is *inside* every shot: `commands/Shoot` owns it and pulses it. If the operator is holding the
  intake on when a shot starts, the shot preempts it and it resumes afterwards, still on.
- **The camera faces front and the shooter fires out the rear.** The tags are in view only while the
  robot faces the HIVE. Drive up facing it, pull the aim lock, *then* turn: the card says
  `tag-corrected` while the correction is in force, and the correction expires after
  `AIM_BIAS_MAX_AGE_MS` or the moment anything rewrites the pose.
- **MANUAL takes odometry, the table and the aim law out of the loop in one press,** and the aim lock
  stays off for as long as it is engaged. That is what makes it an override rather than a partial one.
  It is the right response to a hard collision, a CELL that has moved, or a shot from somewhere nobody
  measured.
- **`followLazyCommand` reports "arrived" the moment anything else changes the follower's mode**, because
  `atParametricEnd()` is true whenever the follower is not in FOLLOW. A `hold` or `manual` issued by
  another command mid-path ends the path command after `MIN_PATH_MS`, holding at an end pose it never
  reached. **Keep paths and turns sequential, never parallel.**
- **A zero flywheel target makes `atTarget()` true.** `commands/Shoot` guards against it; anything new
  that gates on `atTarget()` must check `isArmed()` too.
- **The intake current reads NaN unless the roller is pulling.** The ADC read happens only then. NaN
  means "not sampled", not a fault.
- **Anti-jam gives up after `MAX_UNJAM_ATTEMPTS` (3)** and buzzes the operator pad three times. After
  that a human reverses it; nothing retries by itself, on purpose, because a hard jam would otherwise
  cook the motor all match.
- **Tunables are `public static` and live until the Robot Controller app restarts.** A value nudged on a
  bench is what teleop then runs with, and **nothing reports it any more** — the `Tunables` snapshot and
  the `!! TUNED THIS SESSION` card line were deleted. Restart the RC app after a tuning session.
- **A non-legal APK does not announce itself.** `BuildFlavor` is gone, so R704 compliance is a human
  check: strip AutoTune and Dashboard before an event (HANDOFF §4). The giveaway is `Bench: Shooter`: if it is in the
  OpMode list, the APK carries AutoTune and FTC Dashboard and is not match legal.
- **Loop rate is the first line of every card** because it explains most of what else looks wrong. There
  is no p95 or spike histogram any more — `LoopTimer` went with the diagnostics package — just the
  average over ten loops.

## 19. Build and run

```bash
# Gradle 9.1 may reject a newer system JDK; the Android Studio JBR always works.
JAVA_HOME="/Applications/Android Studio.app/Contents/jbr/Contents/Home" \
  ./gradlew :TeamCode:compileDebugJavaWithJavac --console=plain

./gradlew :TeamCode:assembleDebug   # the one APK — AutoTune + Dashboard; NOT match legal until stripped
```

AutoTune's web UI is `http://192.168.43.1:10158` and FTC Dashboard's is `http://192.168.43.1:8080/dash`
while connected to the robot's Wi-Fi; AutoTune has no OpMode, Dashboard is used by `Bench: Shooter`. The repo is on FTC SDK **11.2.1** from the Pedro Quickstart and that is fine (§16). Expect
`BUILD SUCCESSFUL` plus javac warnings about the Java 8 source level.
