# BIOBUZZ V1 Robot: Physical Architecture → Software Map

Source of truth for the physical robot is `BIOBUZZ_V1_Robot_Physical_Architecture.md` (the team's
V1 CAD spec). This document restates the parts of it that software must know and maps every
mechanism to a subsystem in `03-software-architecture.md`. The spec says not to infer control logic
or strategy from it; this document only records what each mechanism *is* and what the code therefore
has to *control* and *sense*.

**Scope.** V1 is deliberately limited to collecting, storing (four pieces), transferring, aiming and
launching. It does **not** include Flower scoring, a vertical or horizontal extension, a secondary
intake, an alternate storage path, a diverter, separate Pollen/Nectar storage, a moving storage
carriage, or endgame mechanisms. The earlier final-robot spec (`BIOBUZZ_Robot_Physical_Architecture.md`)
describes those V2 items; nothing in this repo implements them.

> **Collect reliably. Store predictably. Transfer cleanly. Shoot consistently.**

---

## 1. Overall robot

- Footprint ≈ **17.5 in × 17.5 in**, four-wheel **mecanum** drivetrain, compact, low centre of gravity.
- One continuous game-piece path through the centre of the chassis; electronics pack around it.
- Each stage must be testable on its own.
- Three vertical layers:

| Layer | Contents |
|---|---|
| Upper | flywheel shooter (fixed, firing out the rear), top of the vertical transfer |
| Middle | front intake, short ramp, horizontal 4-piece storage, lower vertical transfer |
| Bottom | mecanum chassis, battery, hubs, wiring, structure |

- Weight rules: battery, hubs and drive motors low; funnel, intake and storage walls light. The
  shooter is the only high-mounted mass, and it is what the drivetrain tuning will feel.

Game pieces: **POLLEN ≈ 2.8 in** (yellow, neutral) and **NECTAR ≈ 3.6 in** (red or blue,
alliance-specific; controlling the opponent's is a foul, G408). Every stage must pass either. A robot
may control at most **4** pieces (G407), which is exactly the storage capacity.

## 2. The game-piece path

```
FRONT
  Wide passive funnel (~2 in deep, angled walls, most of the robot width; no Gecko wheels here)
      ↓
  Front intake roller — goBILDA 16 mm × 16 mm, 30A durometer, 8 mm REX bore (NOT Gecko wheels)
      ↓   shaft sits ABOVE the ramp; pulls the piece in and pushes it back onto the ramp
  Short shallow ramp (~1–2 in of travel; not a conveyor)
      ↓
  Low horizontal storage channel — up to 4 pieces, front → rear,
      Gecko/compliant side wheels along both sides advance and control the queue
      ↓
  Rear 90° vertical transfer — opposing Gecko wheels grip the piece from both sides and lift it
      ↓   feeds directly into the shooter; no intermediate conveyors
  Compliant flywheel shooter, fixed to the chassis and firing out the rear
      (one shooter for both POLLEN and NECTAR)  → HIVE
```

Design constraints the software inherits:

- **The shooter is fixed and fires out the rear, so the drivetrain aims.** The robot's heading must
  be the bearing to the target CELL plus the firing offset (`Shooter.HEADING_OFFSET_RAD`, π): the
  driver holds an aim lock on the heading hold while still translating, and autonomous turns in
  place before shooting (`Macros.aimHeading`, `Macros.aimAt`).
- **One intake, one storage system, one path.** No routing decisions exist in V1: every piece goes
  to the shooter.
- **Minimal transfers.** The vertical transfer feeds the shooter directly; there is no buffer to model
  between transfer and flywheel.
- **Storage is a controlled queue of at most four**, not a hopper: the code knows how many pieces are
  in it and where the queue ends.

## 3. Mechanism → subsystem map

| Physical mechanism | What it is | Likely actuators | Likely sensors | Software subsystem |
|---|---|---|---|---|
| Mecanum drivetrain | 4 wheels, low | 4 DC motors | goBILDA Pinpoint + 2 pods (planned), IMU | `subsystems/Drivetrain` via Pedro `Follower` (`Mecanum` + `PinpointLocalizer`) |
| Front funnel | passive polycarbonate / printed guides | none | none | none (geometry only) |
| Front intake roller | 16 mm compliant wheels on one shaft above the ramp | 1 DC motor (velocity) | intake-entrance sensor | `subsystems/Intake` |
| Ramp | passive | none | none | none |
| Horizontal storage | 4-piece channel, Gecko side wheels move the queue rearward | 1 (or 2) DC motors | storage-entrance sensor, storage-full sensor | `subsystems/Storage` |
| Rear 90° vertical transfer | opposing Gecko wheels, lifts one piece into the shooter feed | 1 DC motor | transfer sensor, shooter-feed sensor | `subsystems/Transfer` |
| Flywheel shooter | compliant flywheel(s) fixed to the chassis at the rear, firing rearward; aimed by the drivetrain heading | 1–2 DC motors (velocity), possibly a feed gate servo | shooter-feed sensor (shared with Transfer), flywheel encoders | `subsystems/Shooter` (+ the `Drivetrain` aim lock) |
| Vision (if installed during V1) | Limelight 3A, rigidly mounted | none | the camera | `subsystems/Limelight` (aiming + piece detection; fail-soft when absent) |
| Piece sensors | reserved points (below) | none | colour / distance / beam-break TBD | `subsystems/ColorSensor` instances (or a distance/beam-break wrapper when chosen) |

### Reserved sensor points

The V1 spec lists "easy sensor mounting" as a priority without naming the points. The code reserves a
config name for each stage boundary in `util/hardware/HardwareNames` so that adding a sensor later is
one line, regardless of the sensor type:

| Point | Purpose for software |
|---|---|
| Intake entrance | "a piece is entering" — gates capture logic and jam detection |
| Storage entrance | count pieces entering the queue; identify POLLEN vs NECTAR colour (G408) |
| Storage full | stop the intake when four are stored (G407) |
| Vertical transfer | a piece is in the lift; interlock so the transfer does not double-feed |
| Shooter feed | a piece is staged at the flywheel; interlock the shot on flywheel speed |

Which sensor goes where (REV Color Sensor V3 for piece type, REV 2 m distance, beam-break on a digital
channel) is not decided. The `ColorSensor` wrapper is instantiable per name so several can coexist.

## 4. Vision (optional in V1)

The spec allows "vision hardware if installed during V1". If a Limelight 3A is fitted:

- **Rigid** chassis mount, so `VisionMath.Mount` (height, pitch, offsets, yaw) is a set of constants.
- High enough that intake and storage do not block the view; final position after the shooter CAD.
  Its yaw relative to the robot's forward axis is `Limelight.CAMERA_YAW_OFFSET_DEGREES`, which the
  aim law uses with a tag's tx; a rear-firing shooter with a front camera aims from odometry.
- Uses: AprilTag **aiming** (the clusters on the HIVE CELL give `tx`/`ty`/range to the CELL opening)
  and game-piece detection (yellow POLLEN, red/blue NECTAR). Field localisation from tags is **not
  available in BIOBUZZ** (docs/04 §3); the pose comes from Pinpoint odometry and the IMU.
- Without the camera, `Limelight.isAvailable()` is false and the robot aims from odometry toward
  the known CELL position (`game/Field`), with a wall-referenced fallback.

## 5. Electronics packaging and the motor budget

Control Hub, Expansion Hub if required, battery, wiring and vision hardware go in side cavities beside
the storage, under parts of it, or in low front/rear corners. None of it may cross the path
funnel → intake → ramp → storage → transfer → shooter.

**Rule check (docs/04 §6):** BIOBUZZ R503 allows at most **8 DC motors and 8 servos**. V1 wants
4 drive + intake + storage (1–2) + transfer + shooter (1–2) = **8–10 motors**: a single flywheel and
a single storage motor fit exactly, otherwise one mechanism must share a motor (storage and transfer
geared together). A Control Hub plus one Expansion Hub is the maximum (R701);
bulk-cache reads clear **every** hub (`Robot.readSensors()` iterates all `LynxModule`s).

## 6. Game-piece state as the software will track it

```
  none ──intake──▶ ENTERING ──ramp──▶ STORED[1..4] ──transfer──▶ IN_TRANSFER ──▶ AT_FEED ──shoot──▶ gone
```

| Transition | Owner | Interlocks |
|---|---|---|
| none → ENTERING | `Intake` | not when storage is full; reject opponent NECTAR |
| ENTERING → STORED | `Storage` (count++) | storage-entrance sensor |
| STORED → IN_TRANSFER | `Storage` advance + `Transfer` run | transfer empty, shooter ready |
| IN_TRANSFER → AT_FEED | `Transfer` | feed sensor |
| AT_FEED → shot | `Shooter` (feed gate / transfer pulse) | flywheel at speed, robot aimed (heading hold on the CELL) |

These are the sequences `commands/Macros` composes ("intake until full", "shoot one", "shoot all");
the subsystems only expose single-mechanism commands.

## 7. V1 physical priorities, restated for code review

1. Reliable intake → capture is sensor-confirmed, jam detection on the intake motor.
2. Reliable storage → the count is authoritative and hard-stops at four.
3. Reliable horizontal-to-vertical transfer → transfer never runs into an occupied feed.
4. Reliable shooter feeding → a shot is only released at speed, with the robot aimed.
5. Low jam rate → every stage has a timeout and a reverse.
6. Easy maintenance / sensor mounting → names reserved now, wrappers fail-soft when absent.
7. Low centre of gravity, simple packaging, fast iteration → retune the drivetrain after the
   shooter is fitted.

## 8. Deferred to V2 (not in this repo)

Flower scoring mechanism, vertical and horizontal extensions, secondary intake, alternate storage
path, separate Pollen/Nectar storage, routing diverter, complicated endgame mechanisms, moving storage
carriage, elevators. The game rules that only those mechanisms would exercise (FLOWER scoring, G410's
NECTAR-into-FLOWER window) are recorded in docs/04 for when V2 starts; `MatchClock.isFlowerUnlocked()`
already exposes the timing.

## 9. Open hardware questions the code cannot resolve

| Question | Affects |
|---|---|
| Drive motor model and gearing | Pedro tuning only (AutoTune measures it) |
| Intake / storage / transfer / shooter motor models (ticks per rev, free speed) | velocity targets and stall-current thresholds |
| Which mechanism shares a motor to fit R503's 8 | `HardwareNames` optional second-motor names; subsystem ownership of a shared motor |
| Shooter firing direction (out the rear is assumed) and the camera's yaw relative to it | `Shooter.HEADING_OFFSET_RAD`, `Limelight.CAMERA_YAW_OFFSET_DEGREES` |
| Flywheel: one or two motors, feed gate or transfer pulse, hood or fixed angle | `Shooter` |
| Sensor type at each of the five points | which wrapper each `HardwareNames` entry resolves to |
| POLLEN / NECTAR colour signatures under match lighting | `game/PieceType` thresholds |
| Whether a Limelight is fitted in V1, and where | `Limelight` mount constants; aiming fallback |
| Hub count and port assignment | `HardwareNames` only |

Treat this V1 architecture as the baseline unless the CAD spec is explicitly changed.
