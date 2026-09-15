# fixthese.md — every issue found in the 25702 BIOBUZZ code (review of 2026-09-14)

Scope: every file under `TeamCode/src/main/java/org/firstinspires/ftc/teamcode/` except the untouched
Quickstart `pedro/procedures/*`, plus the tests, docs and HANDOFF. Verified against the JVM test
results from this morning (295 tests, 0 failures, no source changed since).

Severity key:
- **BLOCKER** — the robot cannot compete, or a rule is broken, or work is at risk.
- **BUG** — wrong behaviour that will show up on the robot.
- **RISK** — will probably bite once hardware/tuning exists; fix before relying on the feature.
- **SMELL** — correctness is fine, but it will mislead someone or rot.

Items are ordered by severity, then by where they live. Each has the file, the line-level cause,
and what to do about it.

---

## Status (2026-09-14, after the fixes)

Every code-level item below is done and committed with a JVM test unless this table says otherwise;
HANDOFF §8.1 maps each item to its change and its test. Items that need the robot are listed with the
bench that measures them.

| Item | Status |
|---|---|
| A1, A2, A3, A4 | done (commits `3613b9e`, `a5e7124`, `e18cba0`) |
| B1, B3, B4, B6, B7 | done (`86752e4`) |
| B2, C6, C7, C8, C9, D1 | done (`2c206ba`) |
| B5 | done (`3613b9e`) |
| C1, C2, C3, C5, C10, C11 | done (`0fb0ab7`); C2 and C3 ship **off** (`Intake.REJECT_ENABLED`, `Shooter.CUSTOM_PIDF`) until measured |
| C4 | numbers changed to 7 A / 300 ms; **measure** with `Bench: Intake` |
| C12 | done (`86752e4`) |
| C13 | **deferred**: SDK 12.0 is its own task |
| D2, D3, D4, D8 | done (this commit) |
| D5 | done (`9f791de`): `opmodes/test/SelfTest` on `BenchOpMode`; `ConceptCommands` deleted |
| D6 | **on the robot**: TestHardware or the Mecanum Tuner, then `Constants.drivetrainConfig` |
| D7, D11, D12 | done |
| D9, D10, D15 | known, no action; noted in HANDOFF §7 |
| D13, D14 | **on the robot / CAD**: HANDOFF §9 |
| G1–G8 (below) | done (`0c53dc9`) |

---

## A. BLOCKERS

### A1. Teleop cannot drive until Pedro is fully tuned
- **Where:** `opmodes/teleop/Teleop.java` `onInit()`; `subsystems/Drivetrain.java` `drive()`;
  `pedro/Constants.java` `create()`.
- **Cause:** `Constants.create()` returns `null`, so `Drivetrain.follower` is `null` and
  `Drivetrain.drive()` returns on its first line. `Teleop.onInit()` schedules
  `driverControlCommand` on that dead drivetrain and never references `OpenLoopDrive`. The sticks
  do nothing. `TeleopTest` passes only because it injects a `FakePathFollower`.
- **Impact:** at the first event, before Pinpoint + Foresight are tuned, the robot does not move in
  teleop. The HANDOFF's "everything else still runs" is wrong on this point.
- **Fix:** in `Teleop.onInit()`, if `!robot.drivetrain.isAvailable() && robot.openLoopDrive.isAvailable()`,
  schedule an `OpenLoopDrive` default command that reads the same shaped sticks and calls
  `openLoopDrive.drive(f, s, t)` every loop (robot-centric, no heading hold). Add a
  `TeleopTest` case that builds the robot with `new Drivetrain((PathFollower) null, clock)` and a
  `FakePedroDrivetrain`, pushes the stick, and asserts `motors.lastPowers.forward() > 0`.

### A2. `pedro/Tuning.java` is empty, so AutoTune has nothing to run
- **Where:** `pedro/Tuning.java` (three unused imports, no methods).
- **Cause:** AutoTune discovers procedures via `@Tuner public static Procedure x()` factories
  (docs/01 §A.7, with the exact code to paste). None exist.
- **Impact:** HANDOFF §8 step 1 ("Run the Mecanum Tuner") is impossible as the repo stands; the
  web UI at `:10158` will list zero tuners. Every later tuning step depends on it.
- **Fix:** paste the four factories from docs/01 §A.7 into `Tuning.java`. `foresightTuner` and
  `tests` need `Constants.localizerConfig` / `foresightConfig` fields that do not exist yet; add
  them as `null` placeholders or register only `mecanumTuner` + `pinpointTuner` for now.

### A3. Nothing is committed
- **Where:** `git status` — HEAD is Quickstart `b431238`; `Robot.java`, `commands/`, `game/`,
  `opmodes/`, `subsystems/`, `util/`, `src/test/`, `docs/`, `HANDOFF.md` are all untracked.
- **Impact:** one "revert", one clone, one accidental `git checkout .` erases everything.
- **Fix:** commit now. Add `.claude/` to `.gitignore` first (see D11).

### A4. `com.pedropathing:tuning` ships in every build — R704 exposure
- **Where:** `build.dependencies.gradle` line 20.
- **Cause:** AutoTune binds its HTTP (10158) and WebSocket (12649) servers whenever it is on the
  classpath (docs/01 §A.9 #15). R704 prohibits streaming/dashboard tools during matches. The docs
  say "remove for competition builds"; nothing in the build does it.
- **Fix:** move it to a `debugImplementation`-style flavour, or gate it behind a Gradle property
  (`-Ptuning`) so the competition APK is built without it by default. Put the rule number in a
  comment on the dependency line.

---

## B. BUGS (wrong behaviour, will show on the robot)

### B1. `Robot.updateLocalization()` kills the heading hold and hammers the localizer every loop
- **Where:** `Robot.java` `updateLocalization()` → `drivetrain.setPose(corrected)`;
  `Drivetrain.setPose()` → `releaseHeadingHold()`; `PoseFusion.update()`.
- **Cause:** `PoseFusion.update()` returns a non-null pose on every call, including the
  odometry-only path (`fused(odometry)`). So `Drivetrain.setPose()` runs every loop, which
  (a) calls `follower.setPose()` — with a `PinpointLocalizer` that is an I2C write to the Pinpoint
  every loop — and (b) calls `releaseHeadingHold()`, setting `heldHeading = null`.
- **Effect on the heading hold:** in `applyHeadingHold`, `heldHeading == null` → capture the current
  heading, `return 0`. Every loop. The hold never produces a correction; the robot yaws freely.
- **Effect on the aim lock:** `heldHeading == null` → `resetHeadingController()` every loop, so the
  D term and any I term are wiped each tick; the lock is P-only with no damping.
- **Why tests miss it:** `DrivetrainCommandTest` exercises the hold on `Drivetrain` alone;
  `TeleopTest` asserts `isAimLocked()` / `heldHeading` after `execute()`, which re-sets it inside
  the same loop.
- **Fix:** only write back when fusion actually changed something:
  `if (poseFusion.getLastResult() == ACCEPTED) drivetrain.setPose(corrected);` (or have
  `PoseFusion.update` return `null` for `ODOMETRY_ONLY`). Add a `RobotTest` that runs 20 loops with
  the fake follower, no vision, a fixed pose and a centred stick, and asserts the follower saw a
  non-zero `turn` after the pose heading is perturbed.

### B2. Sensorless shooting runs the storage for 2.5 s per shot into a stopped transfer
- **Where:** `commands/Macros.java` `feedOneCore()`; `subsystems/Storage.java`
  `advanceOneCommand()`; `subsystems/Transfer.java` `liftOneCommand()` / `feedCommand()`.
- **Cause:** without an exit sensor `exitEvents` never rises, so `advanceOneCommand` runs until
  `ADVANCE_TIMEOUT_MS` (2500 ms) every time. During those 2.5 s the transfer is `IDLE` under
  `BRAKE`, so the side wheels push all remaining pieces against a stationary transfer. Then
  `liftOne` (600 ms) and `feed` (300 ms) are the same motor at two speeds — the "staging" is
  fictional.
- **Impact:** high jam probability on the first shot; 4 × 3.4 s + up to 3 s spin-up = ~16.6 s of a
  30 s auto spent mostly stalling the storage; `shotsFired` counts pulses, not shots.
- **Fix:** for the first event, replace the sensorless branch with a timed dump: spin up, then run
  storage + transfer together at feed speed for `DUMP_MS`, with the flywheel held. Keep the
  per-piece state machine behind `storage.hasExitSensor() && transfer.hasFeedSensor()`. If you keep
  `advanceOne` sensorless, make it a short pulse (`ADVANCE_PULSE_MS`, like `LIFT_PULSE_MS`), not a
  run-to-timeout.

### B3. Field-centric drive is backwards for BLUE once the pose comes from the real field frame
- **Where:** `subsystems/Drivetrain.java` `drive()` (uses `pose.heading()` directly);
  `opmodes/teleop/Teleop.java` (no alliance offset anywhere in the drive path).
- **Cause:** the field frame's +X points toward column F, i.e. **at** the blue driver. Field-centric
  stick-forward = +X. With a true-frame pose (inherited from auto, or after the Pinpoint is tuned
  and seeded from `FieldPoses.startPose`), a blue driver's "forward" drives toward themselves. Red
  is correct by luck. `RESET_HEADING` (Y) masks it only while the pose is not inherited.
- **Fix:** add a driver-frame offset to `Drivetrain` (`setDriverHeadingOffset(alliance == BLUE ? π : 0)`)
  applied in `drive()` before `ManualDrive.fieldCentric`. Keep the aim lock and macros on the true
  frame. Test: blue pose at heading π, stick forward, assert follower forward is +1.

### B4. Stale alliance from a previous run cannot be overridden in Teleop
- **Where:** `util/field/PoseStorage.java` (static, lives for the RC app process);
  `Teleop.onInitLoop()` gates the dpad on `!allianceFromAuto`.
- **Cause:** any earlier auto run in the same RC process (practice field, previous match, a test)
  leaves `alliance` set; Teleop then shows "(from auto)" and ignores the dpad.
- **Impact:** wrong CELL for the aim lock and wrong rotated poses for `DRIVE_TO_SHOOT` / `PARK`,
  with no way to fix it at the DS.
- **Fix:** always allow the dpad to flip the alliance in `onInitLoop`; show the source as
  information only. Consider clearing `PoseStorage` in `MatchOpMode.stop()` of Teleop.

### B5. `Teleop` never schedules an `OpenLoopDrive` default, and `OpenLoopDrive` shares motors with the follower
- **Where:** `Teleop.onInit()`; `subsystems/OpenLoopDrive.java`.
- **Cause:** `OpenLoopDrive` is built for every OpMode (`Robot` constructor) but only auto schedules
  its `defaultStopCommand`. Today this is harmless (it writes only on intent change and never gets
  an intent in teleop). Once A1 is fixed it needs a proper hand-off: when
  `drivetrain.isAvailable()` becomes true, `OpenLoopDrive` must never be commanded again, or two
  `Mecanum` objects fight over the same four motors.
- **Fix:** make the stopgap default exclusive: schedule exactly one of the two drive defaults in
  `Teleop.onInit()` based on `drivetrain.isAvailable()`, and assert in `Robot` that they are never
  both non-idle (`stopMechanisms` already stops `openLoopDrive`).

### B6. `MatchClock` reports an ENDGAME that does not exist in BIOBUZZ
- **Where:** `util/time/MatchClock.java` `ENDGAME_MS = 60_000`, `forTeleop()`.
- **Cause:** BIOBUZZ has no endgame (docs/04 §5.1). The DS "Time" line and the log `phase` column
  will read `ENDGAME 59.9s` for the last minute; `isEndgame()` is unused.
- **Fix:** `forTeleop()` with `endgameMs = 0`, or rename the phase to `FLOWER_WINDOW` if you want
  the 1:00 unlock visible.

### B7. `PinpointLocalizer` IMU calibration is not handled in the OpMode lifecycle
- **Where:** `Robot.java` constructor → `Drivetrain(hardwareMap, clock)` → `Constants.create()`;
  `MatchOpMode.start()`.
- **Cause:** docs/01 §A.9 #6: constructing a `PinpointLocalizer` starts IMU calibration; you must
  sleep ≥ 1 s or re-zero in `start()`. `Robot` is built in `init()` (good) but nothing re-zeros
  heading in `start()`, and `Teleop.onInit()` seeds the inherited pose before calibration finishes.
- **Impact:** once `create()` is real, the first second of pose/heading data is garbage, and a fast
  INIT→START (drivers do this) starts the match with a wrong heading.
- **Fix:** in `MatchOpMode.start()`, or `Drivetrain.startTeleop()`, re-apply the intended pose
  (`setPose(PoseStorage.getPose())` / start pose) after calibration; or block START on a
  `follower.localizer().isReady()`-style check with telemetry.

---

## C. RISKS (will bite once the hardware exists)

### C1. Every "piece present" sensor is a hue match against placeholder hues
- **Where:** `Robot.java` `pieceAtStorageEntrance/storageFullSensorSees/pieceInTransfer/pieceAtShooterFeed`
  → `PieceType.anyAtSensor`; `game/PieceType.java` hues `{55}`, `{0, 220}` ± 20°,
  `MIN_SATURATION 0.35`, `MIN_VALUE 0.15`.
- **Impact:** the storage-full interlock (G407), the transfer interlock and the feed interlock are
  all false whenever lighting, gain, or the real ball colour lands outside an unmeasured window.
  They fail silently to "no piece", which the code treats as "keep intaking".
- **Fix:** presence sensors (`storageFull`, `transfer`, `shooterFeed`) should use
  `ColorSensor.getDistance()` (REV V3 exposes it; the wrapper already has `hasDistance()`), with
  a threshold measured on the bench. Reserve hue matching for the storage-entrance G408 check.
  Add a distance-based predicate in `Robot.wireSuppliers()`.

### C2. No G408 opponent-NECTAR rejection exists
- **Where:** `Intake.java` (`capturedSupplier` only), `PieceType.classify()` returns
  `POLLEN`/`NECTAR`, never red-vs-blue.
- **Impact:** intaking the opponent's NECTAR is a foul per piece; V1's only defence is a colour
  sensor at the storage entrance, which the code does not use for rejection.
- **Fix:** `PieceType.NECTAR_RED` / `NECTAR_BLUE`; `Intake.setRejectSupplier(...)` that reverses
  the roller for `EJECT_MS` when the entrance sees the opponent colour; wire in `Robot` from the
  alliance. Needs the alliance to reach `Robot` (today only the OpMode knows it).

### C3. Flywheel `atSpeed` is unlikely to latch with stock velocity control
- **Where:** `subsystems/Shooter.java` `TICKS_PER_REV = 28`, `SHOOT_RPM = 3000`,
  `AT_SPEED_TOLERANCE_RPM = 100`; `templates/VelocityMotor.java` (`RUN_USING_ENCODER`, no PIDF set).
- **Cause:** 3000 RPM on a 28 CPR encoder = 1400 ticks/s; ±100 RPM = ±47 ticks/s through the hub's
  velocity estimate and the SDK's default PIDF (whose F depends on the motor type chosen in the
  RC configuration). The reading will oscillate outside that band.
- **Impact:** `spinUpCommand` waits the full `SPINUP_TIMEOUT_MS` (3 s) every time, then fires anyway.
  Not a crash; a 3 s tax per shot macro and a shooter that "reports ready" when it isn't.
- **Fix:** set `setVelocityPIDFCoefficients` explicitly (measure F = 32767 / maxTicksPerSec),
  widen the tolerance to ~5 % of target, and/or require N consecutive in-band loops.

### C4. Intake stall threshold vs. motor stall current
- **Where:** `Intake.java` `STALL_CURRENT_AMPS = 5.0`, `STALL_TIMEOUT_MS = 200`.
- **Cause:** a 435 RPM goBILDA 5203 stalls near 9 A; a compliant roller pulling a ball in can sit
  at 5–6 A for 200 ms legitimately.
- **Impact:** anti-jam reverses the roller mid-capture and spits the piece.
- **Fix:** measure; expect ~7 A / 300 ms. Note the HANDOFF already lists it as "to measure".

### C5. `JamDetector.MAX_UNJAM_ATTEMPTS` is effectively unbounded
- **Where:** `util/control/JamDetector.java` `update()` — `attempts = 0` whenever one reading is
  below threshold.
- **Cause:** after each 150 ms reverse pulse the current briefly drops as the motor re-accelerates,
  which resets `attempts`. The "give up after 3" guard only holds if current never dips once.
- **Impact:** a hard jam cycles forward/reverse indefinitely (the "cook the motor all match" case the
  Javadoc claims to prevent).
- **Fix:** reset `attempts` only after `N` ms of healthy running, not on one sample.

### C6. `Waits.bounded(shootAll, 20 s)` cuts `shootAll` mid-group and leaves `Outcome.RUNNING`
- **Where:** `opmodes/auto/AutoRoutine.java` `build()`; `Macros.shootAll()`.
- **Cause:** the race interrupts the macro before its `finishWith` runs; `AutoRoutine` patches it
  with `markCancelled()`. Works, but every future caller of `bounded(macro, …)` must remember the
  same patch, and `Teleop` does not (an aborted macro relies on `robot.abortMacro()` instead).
- **Fix:** give every macro a `setEnd` on its outer group that calls `markCancelled()` when
  `ec != NATURALLY`, so the outcome is always terminal without the caller knowing.

### C7. `Shooter.spinUpCommand` and `holdSpeedCommand` run as siblings requiring the same resource
- **Where:** `Macros.shootOne()` / `shootAllCore()`: `deadline(sequential(spinUp, …), holdSpeed)`.
- **Cause:** both children require `shooter`. Ivy allows it inside a group (the group is the
  scheduled unit) and both set the same target, so it is harmless today — but any future change
  where the two set different targets (e.g. `IDLE_RPM > 0`) makes the last-executed child win
  silently, and the order of children inside a `parallel` is not part of Ivy's contract.
- **Fix:** drop `spinUpCommand` from the sequence and use `waitUntil(shooter::atSpeed)` raced with
  the spin-up timeout while `holdSpeedCommand` owns the shooter.

### C8. `Macros.intakeUntilFull` runs the storage transport continuously while intaking
- **Where:** `Macros.java` `INTAKE_RUNS_STORAGE = true` → `storage.advanceUntilCommand(isFull, 8 s)`.
- **Cause:** the side wheels push every stored piece rearward into a stopped transfer for up to
  8 s. Same mechanism concern as B2; the HANDOFF lists the question but defaults to the risky side.
- **Fix:** default `false` until the mechanism proves it needs it, or run the transport in short
  pulses on each entrance edge.

### C9. Auto timing has ~2 s of margin under `SHOOT_BUDGET_MS` and none is measured
- **Where:** `AutoRoutine.java` `SHOOT_BUDGET_MS = 20000`; per-shot ≈ 2.5 + 0.6 + 0.3 s + 3 s spin-up.
- **Impact:** 4 × 3.4 + 3 = 16.6 s of the 20 s budget; any `ADVANCE_TIMEOUT_MS` bump pushes shots
  4 out of the budget, and the budget interrupt then reports CANCELLED with the last piece still in
  the lift. Fixing B2 removes most of this.

### C10. Vision blob pipeline geometry uses all-placeholder mount constants
- **Where:** `Limelight.java` `CAMERA_HEIGHT_INCHES = 12`, `CAMERA_PITCH_DEGREES = 20`,
  `CAMERA_FORWARD_OFFSET_INCHES = 6`, `CAMERA_YAW_OFFSET_DEGREES = 0`, `PICKUP_STANDOFF_INCHES = 8`.
- **Impact:** `collectPiece` / `alignToPiece` will drive to the wrong spot until measured. Fail-soft
  (times out), but a driver pressing A on a bad estimate gets 4 s of the robot driving somewhere
  unexpected before the abort.
- **Fix:** keep `COLLECT` / `ALIGN` unbound in `Controls` until the mount is measured, or gate them
  on a `Limelight.MOUNT_CALIBRATED` flag.

### C11. `Limelight.getBotposeAsPedroPose` frame conversion is a guess
- **Where:** `Limelight.java` — adds `FIELD_CENTER_INCHES` to LL `x`/`y` and uses LL yaw directly.
- **Cause:** the Limelight/FTC field frame (centre origin, X along the red wall, Y away from it) is
  not the repo's corner-origin frame (+X toward column F). A real botpose would land rotated.
- **Impact:** none this season (tags move; returns null). Flag it so nobody trusts it later.

### C12. `Teleop.onInitLoop` calls `tryLocalizeFromAprilTag()` every init loop
- **Where:** `Teleop.java` `onInitLoop()`; `Robot.tryLocalizeFromAprilTag()`.
- **Cause:** harmless now (pipeline index already 0, botpose null). If a future pipeline layout
  puts blobs at index 0 this becomes a pipeline switch every 20 ms.
- **Fix:** call once at init, or only when `limelight.isAvailable()` and a fix is plausible.

### C13. SDK 11.2.1 vs. season SDK 12.0
- **Where:** `FtcRobotController/` (Quickstart's 11.2.1).
- **Impact:** no inspection minimum this season, but the AprilTag cluster API (needed for tag
  aiming with `percentClusterFound`, and the correct cluster-centre `ftcPose`) is a breaking change
  in 12.0. Every tag-related line in `Limelight` / `Macros.aimHeading` is written against the
  Limelight's own fiducial results, which is fine — but the moment you add SDK vision you must
  upgrade first.

---

## D. SMELLS (fine today, will mislead or rot)

### D1. `AutoRoutineTest` line 125 is a tautology
- `assertEquals("leave went backward at LEAVE_POWER", -0.3, motors.lastPowers == null ? 0 : -0.3, EPS);`
  can only compare −0.3 with −0.3. It never checks the sign of the leave. `OpenLoopDriveTest` does
  check it, so coverage is not lost, but the assertion reads like a real check. Replace with
  `assertEquals(-0.3, lastMovingPowers.forward(), EPS)` captured during the run.

### D2. `HANDOFF.md` claims "everything else still runs" without a follower
- §8 step 4. False for teleop driving (A1). Fix the sentence when A1 is fixed.

### D3. `HANDOFF.md` references machine-specific paths
- `~/Downloads/BIOBUZZ_V1_Robot_Physical_Architecture.md`, `~/Documents/FTC_Guide`,
  `~/.claude/projects/...`. Anyone else on the team cannot follow them. Copy the two spec files into
  `docs/` (or link the shared drive) and drop the session-memory path.

### D4. Dead code in the V1 build
- `subsystems/templates/PositionalMotor`, `PositionalServo`, `util/diagnostics/RateLimiter`,
  `util/field/PoseFusion` (always odometry-only this season), `Limelight.seesTag`,
  `estimateBlobDistanceInches`, `MatchClock.hasTimeFor/isFlowerUnlocked/isEndgame`,
  `PoseStorage.getStartPosition`, `Intake.runForMs`. All tested, none referenced by production
  code. Not harmful; just be honest in the HANDOFF table that "implemented" ≠ "used".

### D5. `opmodes/SelfTest.java` and `ConceptCommands.java` are empty classes
- Not `LinearOpMode`s, not annotated, no Javadoc "contract" despite HANDOFF §3 saying "each stub's
  Javadoc header is the agreed contract". Either write the contract or delete the files.

### D6. `Constants.drivetrainConfig` directions are a guess
- `frontLeft/backLeft REVERSE`, right `FORWARD`. Correct for one common wiring only. Requires A2
  (Mecanum Tuner) before the first `driveForMsCommand` — a reversed wheel turns LEAVE into a spin.

### D7. `HardwareNames.PINPOINT = "pinpoint"` / `IMU = "imu"` are declared but nothing binds them
- `Constants` has no `localizerConfig` yet. Fine, but `reportMissingHardware()` will not list a
  misnamed Pinpoint until `create()` uses it. Add the names to the config in the same commit as A2.

### D8. `MatchLogger` never prunes
- One CSV per OpMode run in `FIRST/data`, forever. A season of practice is thousands of files.
  Add "keep the newest N" in the constructor.

### D9. `Robot.logCells` does ~12 `String.format` calls per loop
- `MatchLogger.formatCell` formats every `Double`. ~0.3–0.5 ms per loop on a Control Hub. Acceptable;
  measure with `loopStats` once real hardware I/O is in the loop, and drop `%.4f` to `%.2f` if
  `p95` climbs.

### D10. `DriveScaling.shape` applies expo per axis
- Forward and strafe are shaped independently, so a 45° stick yields a different magnitude than a
  straight push. Standard FTC practice, but worth knowing when drivers say diagonals feel slow.

### D11. `.claude/settings.local.json` is untracked and not ignored
- Add `.claude/` to `.gitignore` before A3.

### D12. `pedro/Tuning.java` has three unused imports
- Harmless; will be replaced by A2.

### D13. `Field.HIVE_LATERAL_OFFSET_INCHES = 12.4`, `UP_CELL_OPENING_HEIGHT_INCHES = 53`, `FLOWER_INSET_INCHES = 3`, LOADING ZONE / GARDEN placement within their tiles
- All `[INFERRED]` in docs/04 and correctly flagged there; every aim-lock heading and every
  `FieldPoses` value depends on them. Read the Onshape CAD before the first event and measure at
  the venue. (Listed here so the list is complete; not a code defect.)

### D14. `FieldPoses.BLUE_START_FACING_HIVE` assumes an 8.75 in half-length and no bumper/pre-load protrusion
- `ROBOT_HALF_LENGTH_INCHES = 8.75` is the spec footprint; pre-loads may protrude (R102) and the
  wall-touching point may not be the chassis edge. Measure on the built robot.

### D15. `Teleop` help card is 25 lines of `telemetry.addLine` every init loop
- Fine on the DS, but with `TELEMETRY_INTERVAL_MS = 100` it is the largest telemetry packet in the
  codebase. No action unless init_loop timing shows up.

---

## E. What is NOT wrong (so nobody "fixes" it)

- The `final loop()` in `MatchOpMode` and the read → decide → execute → write split: keep it.
- Default commands at priority −1 / `SUSPEND` / `QUEUE` with logic in `setExecute`: correct for Ivy 1.1.1.
- `shootAllCore` unrolled instead of `repeat()`: correct; the NPE is real.
- `Waits` on the injected clock instead of Ivy `waitMs`: correct; it is why the auto test runs in fake time.
- `Hardware.get` fail-soft + `reportMissingHardware()`: correct; keep the "MISSING" lines on the match card.
- `FieldConstants.SYMMETRY = ROTATE_180`: correct for BIOBUZZ.
- `OpenLoopDrive` writing only on intent change: correct.
- `Controls.read()` one snapshot per loop and `resetEdgeDetection()` in `start()`: correct.
- `Robot.stop()` making no motor writes: correct (SDK rejects them from `stop()`).

---

## F. Suggested order of work

1. A3 (commit) → D11.
2. A1 (teleop stick fallback) with its test.
3. A2 (`Tuning.java`) so the Mecanum Tuner can run; then D6.
4. B1 (`updateLocalization` write-back) with its test.
5. B2 (sensorless dump mode) — this is the auto you will actually run.
6. B4 (alliance override), B6 (endgame phase) — ten-minute fixes.
7. A4 (tuning dependency gating) before the first event build.
8. B3, B7 when `Constants.create()` becomes real.
9. C1–C5 as the sensors and motors are fitted and measured.
10. Everything in D as time allows.

---

## G. Found reviewing the plan against the code (2026-09-14)

Practical issues the original list missed, found by reading every subsystem, macro and OpMode against
how the robot is actually used. All fixed in `0c53dc9` unless noted.

### G1. Teleop could not shoot on a sensorless robot
- **Where:** `Macros.piecesOnBoard()`; `Storage.count()`; `Teleop.onInit()`.
- **Cause:** the count only rose on a storage-entrance sensor edge or `setCount`, which only auto
  called. `shootOne`/`shootAll` gate on `piecesOnBoard() > 0`, so with no entrance sensor the
  operator's buttons reported `NO_TARGET` and never ran the transfer. The first-event robot is that robot.
- **Fix:** `Storage.hasEntranceSensor()`; without one a zero count is "unknown" and `piecesOnBoard()`
  returns `CAPACITY`: Shoot One fires one pulse, Shoot All fires four (X stops early). The card shows `?`.

### G2. The storage count was not handed from auto to teleop
- **Fix:** `PoseStorage.save(pose, alliance, start, pieceCount)`; `MainAuto` writes it every loop,
  `Teleop.onInit` applies it. A cut auto no longer starts teleop believing the robot is empty.

### G3. The heading hold fought small deliberate turns
- **Cause:** `HEADING_HOLD_STICK_DEADBAND = 0.05` was compared against the stick *after*
  `DriveScaling.shape` (0.07 deadband, square expo): raw deflections up to ~0.28 (0.45 in slow mode)
  were discarded and replaced by the hold's correction.
- **Fix:** 0.001; the shaped stick is already noise-free, so any non-zero value is intent.

### G4. `aimAndShootAll` had no timeout
- **Cause:** it called `aimCore` raw; every other caller wraps it in `bounded`. It is the planned
  Pedro-auto opener. **Fix:** `bounded(aimCore, AIM_TIMEOUT_MS)`; after the timeout it shoots anyway.

### G5. A 5th piece made `shootAll` report failure
- **Cause:** `shootAllCore` built `CAPACITY` feed steps but `piecesOnBoard()` can be 5 (4 stored + 1
  in the lift). **Fix:** `CAPACITY + 1` steps.

### G6. The flywheel-recovery wait ran after the last shot
- **Cause:** `flywheelRecovery()` was a fixed step of `feedOneCore`: up to 1.65 s of held resources
  doing nothing after every `shootOne` and at the end of every `shootAll`.
- **Fix:** `afterShot(morePieces)`: full recovery only when another piece follows, a 150 ms dwell after the last.

### G7. Second motors were hard-wired FORWARD
- **Fix:** `Shooter.SECOND_MOTOR_DIRECTION`, `Storage.SECOND_MOTOR_DIRECTION` (REVERSE for an opposed pair).

### G8. `markCancelled()` blanked the active name outside its guard
- **Fix:** both writes inside `if (outcome == RUNNING)`.

### Noted, not code (HANDOFF §7, docs/03 §20)
- Camera facing decides whether tag-aiming exists (front camera = odometry aim).
- `followLazyCommand` reports "arrived" the moment anything else changes the follower's mode.
- `Macros.SENSORLESS_FEED_PULSE_MS` is the pulse the robot uses; `Transfer.*_PULSE_MS` are not.
- `INTAKE_RUNS_STORAGE = true` is an unprotected stall (no storage jam detection).
- Shot counts and `SUCCESS` are pulse counts without sensors.
- Four fitted colour sensors ≈ four to eight I2C transactions per loop; watch the loop time.
- Entrance edge counting samples at loop rate; `Bench: Storage` shows the in-view loop count.
- `AutoSelector`'s A never gated START; the card now says "lock".
- The SDK's TestHardware utility covers wiring checks with no code; `DriveBench` was therefore not built.
