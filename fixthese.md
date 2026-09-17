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

---

# Round 2 — review of the fixed code (2026-09-15)

Read: every changed file in commits `e18cba0..5662182` (Drivetrain, Robot, Macros, Shooter, Storage,
Transfer, Intake, VelocityMotor, JamDetector, ColorSensor, PieceType, Teleop, MatchOpMode, MainAuto,
AutoRoutine, OpenLoopDrive, Constants, Tuning, build files, all `opmodes/test/*`). Verified: 346 JVM
tests pass on the current source; the plain `assembleDebug` APK builds without `Tuning.class`
(so the R704 exclusion works). All eight round-1 fix commits hold up: A1, A2, A4, B1, B2, B3, B4,
B7 are correctly fixed and tested. Nothing below is a regression; these are the next layer.

## Round 2 status (2026-09-15, after the fixes)

Every claim below was checked against the code at line level before anything changed. Where the
review was partly wrong, the verdict says so and why; the body of each item is left as written.
368 JVM tests, 0 failures; both APK variants build.

| Item | Verdict on the claim | Change | Proof |
|---|---|---|---|
| R2-A1 | confirmed (also: the intake current was read even when anti-jam was ineligible; MainAuto never showed loop stats) | `Intake` samples current once per `update()`; `VelocityMotor.write` sends only on change, refreshing an unchanged value every `REFRESH_EVERY_N_WRITES` (12) loops so a reset hub recovers; `Robot.readSensors` reads the entrance every loop and the fitted presence sensors one per loop in rotation; loop stats on the auto card | `IntakeCommandTest.currentIsReadOnceALoop`, `VelocityMotorTest.anUnchangedVelocityIsNotResentEveryLoop`, `RobotTest.presenceSensorsAreReadInRotationAndTheEntranceEveryLoop`, `MainAutoTest.runsTheRoutineAndHandsTheAllianceToTeleop` (commit `213770d`) |
| R2-A2 | partly: `isFull()` also ORs the storage-full sensor, so the exposure is "no entrance **and** no full sensor" | `Storage.hasFullSensor()` / `canDetectFull()`; the full supplier is null when unfitted; operator dpad up = count 4 (holds the roller), dpad left = count 0; HANDOFF §8.2 step 0 fits the full sensor first | `RobotTest.aFullSensorIsKnownToTheStorage`, `StorageTest.fullSensorFittedIsRecorded`, `TeleopTest.operatorCanMarkTheRobotFullWithoutASensor` (`06e0566`) |
| R2-A3 | conclusion right, mechanism wrong: the assume-4 fallback existed but was keyed on config presence, and the entrance predicate was hue-only | the entrance counts by presence (`Robot.pieceEnteringStorage` via `pieceNear`: distance first); `PieceType.HUES_CALIBRATED = false`; `Robot.wireSuppliers` trusts a hue-only entrance sensor, and wires the G408 reject, only once the hues are measured | `RobotTest.entranceCountsByDistanceBeforeTheHuesAreMeasured`, `aHueOnlyEntranceSensorIsNotTrustedUntilCalibrated` (`06e0566`) |
| R2-A4 | confirmed, and worse: even a full sensor that ended the run correctly reported TIMED_OUT | outcome is SUCCESS when full or the count rose; on a robot that cannot detect full the timer ending is SUCCESS under the name `intake (timed)`; TIMED_OUT only when a sensor could have seen a piece | `MacrosTest.intakeUntilFullIsATimedRunWithNoWayToKnowFull`, `intakeUntilFullSucceedsWhenTheFullSensorTrips`, `intakeUntilFullTimesOutAndStopsEverything` (`06e0566`) |
| R2-A5 | partly: four statics at five sites (`ANTI_JAM_ENABLED`, `SENSORLESS_FEED_PULSE_MS` twice, `SHOOT_RPM`, `CUSTOM_PIDF`); `STALL_CURRENT_AMPS` was never written and sensor gain is instance state | `util/diagnostics/Tunables` snapshots every tunable (class list in `opmodes/RobotTunables`) at the first init; Teleop and Auto init cards list what differs; bench footers list it and BACK restores. Deliberately no restore on stop: tune-then-drive is the pit workflow | `TunablesTest` (2), `TeleopTest.initCardListsValuesTunedOnABench`, `BenchOpModesTest.benchBackRestoresTheTunablesAndTheCardListsThem` (`aaeb3ce`) |
| R2-A6 | confirmed | `VelocityMotor` reads the hub's PIDF at construction; `Shooter.applyPidf()` restores it when custom is off; the bench card shows what the hub holds | `ShooterTest.turningCustomPidfOffRestoresTheSdkCoefficients`, `BenchOpModesTest.shooterBenchFlipsTheSecondMotorLiveAndTogglesPidfBothWays` (`aaeb3ce`) |
| R2-A7 | confirmed | `util/diagnostics/BuildFlavor.isTuningBuild()` looks for the AutoTune `Tuner` class; every match init card leads with the R704 warning | `BuildFlavorTest` (`aaeb3ce`) |
| R2-A8 | confirmed, and deliberate | kept: START still runs unlocked (this routine is alliance-safe); the init card blinks `NOT LOCKED`, the running card says `started UNLOCKED`; `AutoRoutine` Javadoc sets the rule that the path auto drives only when confirmed | `MainAutoTest.startingUnlockedWarnsButRuns` (`c1179ce`) |
| R2-A9 | confirmed | `VelocityMotor.setDirection` (writes only on change); `Shooter.update()` / `Storage.update()` apply `SECOND_MOTOR_DIRECTION` live; both benches flip it on dpad right and show both motors' velocities | `ShooterTest.secondFlywheelDirectionFollowsTheStaticAtRuntime`, `BenchOpModesTest.shooterBenchFlipsTheSecondMotorLiveAndTogglesPidfBothWays` (`aaeb3ce`) |
| R2-A10 | mostly refuted: `piecesLine()` already printed `?` for exactly the assume-4 case, never `4/4`. What was real: in the R2-A3 configuration the card showed `0/4` as fact | fixed through R2-A3 (that configuration no longer exists) and the operator count buttons; the `?` line now says `assumes 4` and names the button | `TeleopTest.operatorCanMarkTheRobotFullWithoutASensor` (`06e0566`) |
| R2-B1 | confirmed | the trust decision now lives in `Robot.wireSuppliers()` alone, with the policy in its Javadoc, and `Robot.sensingSummary()` prints the outcome on every init card and bench footer. No `SensorSet` object: the subsystems' `has*Sensor()` already say what is wired, and a second object would be a second truth | `RobotTest.transferFallsBackToTimedPulsesWithoutAFeedSensor` (summary string) (`06e0566`) |
| R2-B2 | confirmed | `Macros.ASSUME_FULL_WHEN_UNCOUNTED = true`, on the card as "assumes 4" | `MacrosTest.assumeFullCanBeTurnedOffByName` (`06e0566`) |
| R2-B3 | confirmed | **deferred** (no field effect): extract a shared `RobotOpMode` after the first event | — |
| R2-B4 | confirmed | `Controls.Needs { NOTHING, DRIVETRAIN, CAMERA }`; `Teleop` gates with `Snapshot.anyPressed(Controls::requiresDrivetrain / requiresCamera)`; the camera card line is built from the enum | `TeleopTest.everyDriveMacroIsRefusedWithoutAFollower` (table-driven over the enum) (`c1179ce`) |
| R2-B5 | confirmed | **deferred** with B3 | — |
| R2-B6 | confirmed | `pieceAtStorageEntrance` renamed `pieceEnteringStorage`: presence first, then classification | via R2-A3 tests |
| extra | "storage full" and "final seconds" were both two blips on both pads | final seconds is one 600 ms rumble | `TeleopTest.finalSecondsGiveOneLongRumbleOnBothPads` (`c1179ce`) |

## R2-A. Will it perform on the robot

### R2-A1. Loop time with sensors fitted (RISK, measure first)
- **Where:** `ColorSensor.update()` (colour + distance = 2 I2C reads per fitted sensor);
  `Intake.update()` and `Robot.logCells()` each call `getCurrentAmps()` (a `LynxGetADCCommand`,
  **not** in the bulk cache) — two ADC round-trips per loop; `VelocityMotor.update()` sends
  `setVelocity` every loop for every velocity motor (up to 6) whether or not the target changed.
- **Impact:** with four colour sensors fitted that is ~8 I2C reads (2–4 ms each on a Control Hub)
  plus ~8 bus writes/reads per loop. Expect 25–40 ms loops, i.e. the anti-jam 300 ms window is
  ~8 samples and the aim lock's D term is noise.
- **Fix:** (1) `Intake` reads current once per loop into a field; `logCells` reads that field.
  (2) `VelocityMotor.write` skips the bus when the value equals the last written one (keep the
  anti-jam path unconditional). (3) `Robot.readSensors()` round-robins the colour sensors (one or
  two per loop) — the presence interlocks tolerate 40 ms latency, the loop does not. Confirm
  with `loopStats` in `Bench: Color sensors` before and after.

### R2-A2. Sensorless robot has no G407 protection at all (RISK)
- **Where:** `Storage.isFull()` (count never rises without an entrance sensor), `Robot.wireSuppliers`.
- **Impact:** the intake never stops at four; the only guard is mechanical. G407 explicitly asks
  for "systems to prevent active pickup of more than 4".
- **Fix:** the storage-full distance sensor is the first sensor to fit — it is one config name and
  the interlock already exists. Say so in HANDOFF §8.2 as step 0.

### R2-A3. A fitted-but-uncalibrated entrance sensor makes the robot refuse to shoot (BUG-in-waiting)
- **Where:** `Robot.wireSuppliers` (`entrance = storageEntranceSensor.isAvailable()`),
  `Storage.hasEntranceSensor()`, `Macros.piecesOnBoard()`.
- **Cause:** plugging the entrance sensor in flips `hasEntranceSensor()` to true. Until the hue
  windows in `PieceType` are measured, no edge is ever counted, `count` stays 0, `piecesOnBoard()`
  returns 0 (the "assume 4" fallback is off because a sensor "exists"), and `shootOne`/`shootAll`
  report NO_TARGET without firing. A sensor is trusted because it is in the config, not because
  it works.
- **Fix:** `PieceType.HUES_CALIBRATED = false` (same pattern as `Limelight.MOUNT_CALIBRATED`);
  `Robot` wires the entrance supplier only when calibrated, and the init card says why. Also give
  the operator a "count = 4 / count = 0" control for the sensorless weeks.

### R2-A4. `intakeUntilFull` on a sensorless robot is an 8-second timer that rumbles "failure"
- **Where:** `Macros.intakeUntilFull()` outcome, `Teleop.updateHaptics()`.
- **Impact:** operator presses Y, intake runs 8 s, three failure blips. They will report it as broken.
- **Fix:** when `!storage.hasEntranceSensor() && !fullSupplier`, report `SUCCESS` after the timer
  (or hide the binding via the help card), and shorten the timer.

### R2-A5. Bench edits to `public static` tunables silently carry into the match OpModes (TRAP)
- **Where:** every `Bench: *` OpMode adjusts `Shooter.SHOOT_RPM`, `Intake.STALL_CURRENT_AMPS`, etc.
  Statics live for the RC app process, so a value bumped on the bench is what Teleop runs with
  until the app restarts, and nothing shows the divergence.
- **Fix:** each bench restores the values it touched in `stop()` unless the user pressed a "keep"
  button; and/or `MatchOpMode.init()` prints any tunable that differs from its compiled default
  (snapshot the defaults in a static initializer).

### R2-A6. `ShooterBench` "custom PIDF off" does not restore the SDK PIDF
- **Where:** `ShooterBench` X toggle → `Shooter.applyPidf()` returns early when `CUSTOM_PIDF` is
  false; the motor keeps the custom coefficients. The card then says "SDK default" — false.
- **Fix:** read `getPIDFCoefficients(RUN_USING_ENCODER)` in `VelocityMotor`'s constructor and
  restore it when custom is switched off.

### R2-A7. The `-Ptuning` guard is build-time only (RISK, R704)
- **Where:** `build.dependencies.gradle`, `TeamCode/build.gradle`.
- **Cause:** one `tuning=true` line in `gradle.properties` (someone will add it so Android Studio
  stops showing red files) puts the web server in every APK from then on, and nothing at runtime
  tells you.
- **Fix:** in `MatchOpMode.init()`: `try { Class.forName("com.pedropathing.tuning.autotune.Tuner"); telemetry.addLine("!! TUNING BUILD — not legal in a match (R704)"); } catch (ClassNotFoundException ignored) {}`.

### R2-A8. Auto starts without the selector being confirmed
- **Where:** `MainAuto.onStart()` ignores `selector.isConfirmed()`; alliance defaults to BLUE.
- **Impact:** harmless with the hardcoded auto; the moment paths exist, an unconfirmed START
  drives the blue route on the red side. Fix now while it is cheap: refuse to schedule the routine
  and show a full-screen "PRESS A TO CONFIRM" until confirmed, or blink the card every loop.

### R2-A9. Second-motor directions are read once at construction
- **Where:** `Shooter.SECOND_MOTOR_DIRECTION`, `Storage.SECOND_MOTOR_DIRECTION` are consumed in the
  constructor. A bench that flips them has no effect until re-INIT. Document on the bench card or
  apply in `update()` on change.

### R2-A10. Sensorless count drift is visible to the operator only in debug telemetry
- **Where:** `Macros.piecesOnBoard()` (assume 4 when 0 and uncounted), `Storage.markExited` after
  each pulse. After `shootOne` the count is 3; intake two more (uncounted); `shootAll` fires 3
  pulses with 4 aboard. Bounded and safe, but the "Pieces 3/4" line on the match card is fiction.
- **Fix:** show "Pieces: unknown (no sensor)" when `!hasEntranceSensor()`; pair with the
  operator count control from R2-A3.

## R2-B. Structure

### R2-B1. Sensor-configuration policy is spread across three classes
- `Storage.hasEntranceSensor()/hasExitSensor()`, `Transfer.hasFeedSensor()`, and
  `Macros.piecesOnBoard()/feedOneCore()/transferLeg()` all branch on "which sensors exist"; the
  fallback rules (assume 4, timed pulse, dead-reckon the count) are decided in three places and
  R2-A3 is a direct consequence. Introduce one value object built in `Robot.wireSuppliers()` —
  `SensorSet { canCountEntries, canConfirmExit, canConfirmFeed, presenceIsDistance }` — and have
  every macro ask it. The subsystems keep their suppliers; the *policy* lives once.

### R2-B2. `Macros.piecesOnBoard()` hides a strategy decision in a getter
- "Assume the robot is full when nothing is known" is a game decision. Name it
  (`Macros.ASSUME_FULL_WHEN_UNCOUNTED`), show it on the card, and test it by name.

### R2-B3. `MatchOpMode` and `BenchOpMode` are the two copies of the lifecycle that
`MatchOpMode`'s own Javadoc says must not exist
- `init()`/`init_loop()`/`start()`/`loop()`/`stop()`, the loop timer, `nowMs`, missing-hardware
  reporting are duplicated. `BenchOpMode` should extend `MatchOpMode` with `matchPeriod()` = a
  new `Period.BENCH` and a hook that skips `Scheduler.execute()`, or both should share a base.

### R2-B4. `Teleop.handleDriver` gate chains grow with every macro
- The "needs the drivetrain" and "needs the camera" lists are hand-maintained `||` chains. Put
  `requiresDrivetrain` / `requiresCamera` on the `Controls` enum (it already carries pad, label,
  description) and gate generically.

### R2-B5. Three homes for every rule (Javadoc, docs/03, HANDOFF)
- The round-1 "everything still runs" error was a drift between HANDOFF and the code. `Drivetrain`
  is 26 KB with roughly 40 % Javadoc restating docs/03. Pick one home per rule: the Javadoc owns
  "what this class guarantees", docs/03 owns cross-class contracts, HANDOFF owns status. Delete
  the restatements.

### R2-B6. Presence and classification share a name
- `Robot.pieceAtStorageEntrance()` (hue classification) and `Robot.pieceNear()` (distance) are
  both "is there a piece". Rename the entrance one `classifiedPieceAtEntrance()` so the next person
  does not wire a distance-only sensor into a hue predicate.

## R2-C. Verified fixed (no action)
- A1 open-loop stick fallback + `stickForwardDrivesOpenLoopWhenPedroIsNotTuned`.
- A2 `Tuning.java` factories with `NotReady`; A4 `-Ptuning` exclusion of `Tuning.java` and
  `procedures/**` — plain build confirmed to compile without them.
- B1 `updateLocalization` writes back only on `ACCEPTED`; `headingHoldCorrectsThroughTheFullLoop`.
- B2 sensorless feed is now storage+transfer pulse in parallel with flywheel recovery wait.
- B3 `driverHeadingOffset` (π for blue) in `drive()` and `resetHeading()`; `redForwardIsFieldPlusX`.
- B4 dpad overrides the auto's alliance; B6 no endgame; B7 pose re-applied after the 1 s settle.
- C6/C7 `reporting()` wrapper marks CANCELLED on interrupt; `waitForSpeedCommand` has no requirement
  so it no longer competes with `holdSpeedCommand`.
- D1 the tautological assertion is gone (`lastMoving[0].forward()`).


---

# Round 3 — Pedro 3.0.0 / Ivy 1.1.1 usage audit (2026-09-16)

Asked: check every Pedro and Ivy call in the repo against the official docs (the pedropathing.com
docs source in `Docs-master/`) and confirm the code is correct for real use.

Method: all 60 doc pages read; the Ivy 1.0.0 sources read and the Ivy core 1.1.1 bytecode diffed
against them class by class (only change: `waitMs` now on `System.currentTimeMillis`); the real
`ivy:pedro:1.1.1` source read; Pedro core 3.0.0 disassembled (`Follower`, `PathTracker`,
`Foresight`, `ManualDrive`, `PIDController`, `Angle`, `Vector2D`, `Paths`/`Path`/`AtomicPath`,
`KalmanFilter`); the revhub 3.0.0 sources (`Mecanum`, `CachedMotor`, `PinpointLocalizer`, configs)
and the AutoTune sources (`TunerScanner`, `Tuner`, `Hooks`, `Procedure`) read; then every repo file
that imports the libraries (37 main sources), the fakes and the seven tests that pin library
behaviour checked call by call. 368 tests, 0 failures on the source as reviewed.

## Verdict

**No incorrect Pedro or Ivy usage.** Every call exists with the signature used, and every semantic
the code depends on is real in the shipped artifacts: the follower's mode machine and
`atParametricEnd` (true whenever not FOLLOW), `isBusy` cleared only in a hold, `hold(Pose)` unscaled,
`manual` latched; the scheduler's inline interrupt on `schedule()`, suspend-and-resume without
`start()`, `cancel` always INTERRUPTED; `Deadline`/`Parallel` forwarding their own end condition and
ending losers INTERRUPTED on a natural finish; `Race` checking done before execute; `Repeat.end`
dereferencing a list built in `start` (hence `shootAllCore` unrolled); `Lazy` with no requirements;
`unless` never finishing; `Mecanum`'s +strafe = left and +turn = CCW (so `Controls` negates all three
sticks, as the Quickstart's own `Tests.java` does); `ManualDrive.fieldCentric` rotating by minus the
heading; `PIDController.calculate(0, error)` with Pedro's target-minus-current sign; the Pinpoint
constructor's IMU recalibration; `Path.endPose()` throwing without an interpolator (caught);
`KalmanFilter.update(dx, meas)` giving exactly zero correction on the odometry-only path; `@Tuner`
factory rules; AutoTune's servers bound whenever the library is present.

What cannot be checked off the robot, none of it a library question: motor directions in
`Constants.drivetrainConfig`, Pinpoint pod directions and offsets, the Foresight numbers, and
HANDOFF §9.

## Where the official docs are wrong (recorded in docs/01; do not "fix" the code toward them)

| Page | Says | Library |
|---|---|---|
| `pathing/guide/teleop-usage.mdx` | `manual(-left_stick_y, left_stick_x, right_stick_x)` | +strafe is left and +turn is CCW, so strafe and turn must be negated; the Quickstart's `Tests.java` and `Controls` do |
| `ivy/pedro-commands.mdx`, `pathing/guide/path-following.mdx` | `follow()` finishes when "no longer busy" / `following()` false | `setDone(follower::atParametricEnd)`: 97.5 % of the last segment, still in FOLLOW |
| `pathing/custom/drivetrain.mdx` | the 2.x abstract class (`calculateDrive`, `runDrive`) | the 3.0.0 `Drivetrain` interface (`drive`, `maxScaling`, `stop`, `debug`, `interpolateVelocity`) |

## Round 3 status

| Item | What | Change | Proof |
|---|---|---|---|
| R3-1 | An armed flywheel was told to stop for one loop when a shot macro started: Ivy's OVERRIDE ended Teleop's hold (`idle()`) at `schedule()` and the macro's own hold started two hand-off ticks later, resetting the at-speed latch. About 100 to 300 ms per shot; not a bug | `Macros.reporting(..., Command... alongside)` and `heldFlywheel()`: the hold starts inside `Scheduler.schedule()` for `shootOne`, `shootAll`, `aimAndShootAll` (where it now overlaps the aim); the inner `deadline(..., holdSpeedCommand())` groups are gone; NO_TARGET still never touches the shooter | `MacrosTest.shootOneKeepsAnArmedFlywheelSpinning` |
| R3-2 | `driveTo` from a pose equal to its target built a zero-length line (degenerate inside Pedro: NaN powers, which `CachedMotor` ignores, leaving the last written power). Exact equality is practically impossible on hardware; theoretical | `Macros.MIN_PATH_INCHES = 0.5`; `lineTo` and `approachPath` return null inside it, so `followLazyCommand` finishes at once and the macro's position check reports | `MacrosTest.driveToAlreadyThereFinishesWithoutAPath` |
| R3-3 | The docs discrepancies above would tempt the next reader to flip `Controls` or "simplify" `followLazyCommand` | docs/01 A.4, A.6, B.1 and HANDOFF §7 | — |


# Round 4 — simplify and speed up (2026-09-16)

The team asked for the code to be simpler and more efficient without losing anything that scores.
Four decisions taken with the team first: the Limelight is front-mounted and for AprilTags only; the
AprilTag field-localisation stack goes; the shooting cycle becomes one state-machine command;
`Docs-master/` is ignored, not committed. Three read-only sweeps of the tree (loop cost, the command
and OpMode layer, dead code) produced the evidence; every deletion was checked for callers in
`main/` and `test/`.

## What was costing points

- **Ivy hand-off loops in the shooting cycle.** Ivy's `Sequential` (library source, 1.1.1 identical to
  1.0.0) hands off one child per `execute()` and never executes the child it has just started, so
  every child boundary costs a whole loop, an `instant` costs one by itself, and nesting compounds.
  The shooting tree was nine levels deep: 7 idle loops between "flywheel recovered" and "next pulse
  commanded", ~6 on entry, ~8 on exit, 35 to 43 per four-piece run by two independent counts.
  Measured on the JVM (`MacrosTest`, sensorless four-piece `shootAll`, 20 ms ticks): **3,960 ms →
  3,260 ms**, 198 → 163 loops, the 35 loops the count predicted.
- **Bus transactions for values nobody read.** The intake motor current (not in the bulk cache) was
  read every loop, idle or not; every REV V3 colour sensor paid a colour read *and* a distance read
  while presence was judged by distance alone; the entrance sensor was read even when
  `wireSuppliers()` had decided not to trust it. Two to three transactions per loop on today's
  sensorless robot, two to three on the sensor-fitted one, roughly 3 to 8 ms of a 20 ms loop.
  Plus 16 `String.format` calls per loop for the CSV row.
- **An armed flywheel zeroed on the way out of every shot.** `holdSpeedCommand`'s end idled the
  wheel one loop before Teleop re-armed the operator's hold (the mirror image of Round 3's entry
  fix): one `setVelocity(0)` and a reset at-speed latch per armed shot.
- **Dead weight.** 1,582 lines of Quickstart tuners for localizers the robot does not own; 376 lines of
  positional templates nothing implements; a Kalman filter that never received a measurement
  (`PoseFusion`, gated to null all season); a colour-blob stack the camera decision rules out; 59
  unused public members in `Field`; `Scoring`, `RateLimiter`, ENDGAME machinery in `MatchClock`,
  three `HardwareNames` for hardware V1 does not have, and a tail of members with no caller.

## Status

| Item | What changed | Proof |
|---|---|---|
| R4-1 | Deleted: `OTOSTuner`, `OctoQuadTuner`, `ThreeWheelTuner`, `ThreeWheelIMUTuner`, `TwoWheelTuner`; `Mechanism`, `PositionalMotor`, `PositionalServo`; `RateLimiter`; `Scoring`; `Field`'s FLOWER / GARDEN / inventory / TIP members and the `Zone` class (the LOADING ZONE is one centre pose now); `MatchClock`'s ENDGAME, FLOWER window, `hasTimeFor`; `HardwareNames.IMU / SHOOTER_FEED_SERVO / SENSOR_INTAKE_ENTRANCE`; `PathFollower.isBusy/stop`; `Shooter.spinUpCommand/idleCommand`, `Intake.runForMs`, `Storage.advanceCommand/reverseCommand`, `Transfer.liftCommand/reverseCommand`, `Macros.isRunning/atHeading`, `AutoRoutine.log`, `FieldPoses.BLUE_GARDEN_APPROACH/all`, the stock `readme.md`. `.gitignore` gets `Docs-master/`. | `6ef83ee`; both APKs build; 344 tests |
| R4-2 | Deleted the AprilTag field-localisation stack: `PoseFusion`, `Robot.updateLocalization/tryLocalizeFromAprilTag`, the `MatchOpMode` call, Teleop's init-loop attempt, `Macros.relocalize`, `Limelight`'s botpose members, the `localization` log column. | `3434607`; 329 tests |
| R4-3 | Deleted the colour-blob stack: `collectPiece`, `alignToPiece`, the blob pipeline, `VisionMath`, `MedianFilter`, `Angles.headingToward`, the intake capture flag, `PieceType` sizes, the mount constants and `MOUNT_CALIBRATED`, `Controls.COLLECT/ALIGN` and `Needs.CAMERA`. `Limelight` is now 120 lines: the tag list and `getTagTx`. | `5a22392`; 295 tests |
| R4-4 | `Intake.update` samples current only while pulling (NaN otherwise); `ColorSensor.update(boolean colour)` and `Robot.readSensors` read only what `wireSuppliers` consumes, benches read everything (`setReadAllSensorData`); `MatchOpMode.LOG_EVERY_N_LOOPS = 2`; `Teleop.retarget()` computes the CELL pose and tag range on change. | `674cde8`; 298 tests; `IntakeCommandTest.currentIsReadOnceALoopWhilePullingAndNeverWhenIdle`, `ColorSensorTest.colourIsSkippedWhenNotAsked`, `RobotTest.anUntrustedEntranceSensorIsNotReadAtAll`, `benchesReadEverySensorInFull` |
| R4-5 | `commands/ShootCycle`: one `Command.build()` state machine (`SPIN_UP → [ADVANCE →] [LIFT →] FEED → RECOVER → …`), every sensor variant, requires storage + transfer only; `shootOne` / `shootAll` / `aimAndShootAll` compose it; `shootAllCore`, `feedOneCore`, `transferLeg`, `afterShot`, `flywheelRecovery`, `pieceWasShot` gone from `Macros`. `Shooter.AT_SPEED_HOLD_MS` replaces the loop count; `holdSpeedCommand` has no end action; `AutoRoutine` idles the wheel before the leave. | `62174cf`; 302 tests; `MacrosTest.theNextPulseStartsTheLoopTheFlywheelRecovers`, `anArmedWheelFeedsWithinTwoLoopsOfThePress`, `sensorlessShootAllOfFourFinishesInsideTheSumOfItsTimers`; `TeleopTest.anArmedFlywheelIsNeverStoppedAcrossAShot`; `ShooterTest.holdSpeedLeavesTheTargetAndTheDefaultIdlesItNextLoop` |
| R4-6 | `Macros.aimHeading` remembers the tags' disagreement with the odometry bearing while a tag is visible and applies it while no tag is (`AIM_BIAS_MAX_AGE_MS`, dropped on `Drivetrain.getPoseWrites()` change); `TagBearingSource` seam for tests; the teleop card shows `tag-corrected`. | `e9ab335`; 305 tests; `MacrosTest.aimHeadingKeepsTheTagCorrectionAfterTheTagLeavesView`, `theAimCorrectionExpires`, `theAimCorrectionIsDroppedWhenThePoseIsRewritten` |
| R4-7 | HANDOFF, this section, docs/01..04 | this commit |

Numbers: main code 13,070 → 9828 lines, tests 7,060 → 6345 lines, JVM suite 370 → 305 tests (the deleted
features took their tests with them; eleven new tests pin the gains). Both APKs build after every step.

**Not done, and why:** the `MatchOpMode` / `BenchOpMode` shared base (R2-B3) stays deferred, structure
only; `OpenLoopDrive` stays until Pedro is tuned; the `PathFollower` seam and the fakes are what let
the drivetrain run on the JVM; the per-loop telemetry strings are under a millisecond and the SDK
transmits every 100 ms anyway; teleop's aim lock (the heading-hold PID under stick driving) and
auto's `aimAt` (Pedro's tuned hold, turning in place) stay two controllers on one aim law.
