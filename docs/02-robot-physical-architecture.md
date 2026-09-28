# BIOBUZZ Robot: Physical Architecture → Software Map

What the robot actually is, and which subsystem owns each mechanism. Read from the CAD
(`Non wheels full assembly.glb`, 2026-09-27) and confirmed by the team the same day. The two files in
`docs/specs/` are the team's earlier written specs: they are the design *intent* and are superseded by
this document wherever the two disagree, because the CAD is what is being built.

**The CAD is a parts-layout file, not a positioned assembly.** It tells us which mechanisms exist and
which parts are in each of them. It does **not** tell us where anything sits on the chassis, so this
document defines mechanisms and hardware and deliberately states **no geometry**. Every pose,
standoff, camera height and shooter angle in the code is still a placeholder — see HANDOFF §9.

> **Seven motors. One camera. No piece sensors. The drivetrain aims.**

---

## 1. Overall robot

- Four-wheel **mecanum** drivetrain, compact, low centre of gravity. Footprint ≈ **17.5 in × 17.5 in**
  from the V1 spec; unconfirmed against the built chassis, and `FieldPoses.ROBOT_HALF_LENGTH_INCHES`
  is derived from it.
- One continuous game-piece path front to back: funnel, roller, tunnel, flywheel.
- The shooter is **bolted to the chassis and fires out the rear**. There is no turret. Aiming is
  therefore the drivetrain's heading, and that single fact shapes more of the software than anything
  else on this page (`Shooter.HEADING_OFFSET_RAD` = π).
- The heaviest and highest mass is the shooter, and it is at the back. The drivetrain will want
  re-tuning after it is fitted.

Game pieces: **POLLEN ≈ 2.8 in** (yellow, neutral) and **NECTAR ≈ 3.6 in** (red or blue,
alliance-specific — controlling the opponent's is a foul, G408). Every stage must pass either. A robot
may control at most **4** pieces (G407).

## 2. The mechanisms, and the motors

| Mechanism | Motors | Hardware |
|---|---|---|
| Drivetrain | 4 × goBILDA **5203-2402-0014** (13.7:1, **435 RPM**) | mecanum + **goBILDA Pinpoint odometry computer with 2 pods** |
| Intake — roller **and** tunnel, **one motor** | 1 × goBILDA **5203-2402-0051** (50.9:1, **117 RPM**) | roller: 10 × 16 mm × 16 mm 30A compliant wheels on a 288 mm REX shaft. Tunnel: 8 × Gecko 32 mm, 4 × Gecko 72 mm, 3 × Gecko 48 mm, 5 sprockets and chain |
| Shooter | 2 × non-goBILDA "bare" motors | two **opposed 2.9 in** flywheels, 1.26 in wide; per motor GT2 40T → GT2 50T + HTD-3mm-43T on a 97T belt |
| Vision | — | **Limelight 3A**, front-mounted, AprilTag aiming only |

**Seven motors.** R503 allows eight, so there is exactly one spare port and no room for a mechanism
that wants two (§6).

Two consequences the software inherits directly from this table:

- **The roller and the tunnel are one motor**, geared together through the sprocket chain. They cannot
  be commanded separately, so they are one subsystem with one mode, and **feeding the shooter is
  running the intake forward**. See §3.
- **The shooter motors are not goBILDA parts and nobody has counted their encoder ticks per
  revolution.** So `Shooter` works entirely in **encoder ticks per second** — the unit
  `DcMotorEx.getVelocity()` actually returns — and there is no RPM and no `TICKS_PER_REV` anywhere in
  it. A measured ticks-per-second figure cannot be wrong about itself; a guessed conversion would make
  every number on the bench card and in the distance table a lie.

## 3. The game-piece path

```
FRONT
  Wide passive funnel (angled walls, most of the robot's width; no powered parts)
      ↓
  Roller — 10 × 16 mm × 16 mm 30A compliant wheels, 288 mm REX shaft
      ↓   ONE MOTOR drives this and everything below it
  Tunnel — opposed Gecko wheels (32 / 48 / 72 mm) on a sprocket chain, carrying the piece rearward
      ↓   no gate, no indexer, no sensor: a piece arrives because the tunnel ran long enough
  Two opposed 2.9 in flywheels, fixed to the chassis, firing out the rear   → HIVE CELL
```

Design constraints the software inherits:

- **The shooter is fixed and fires out the rear, so the drivetrain aims.** The robot's heading must be
  the bearing to the target CELL minus the firing offset (`Shooter.HEADING_OFFSET_RAD`, π). In teleop
  that is an aim-lock setpoint inside the heading hold, so the driver keeps translating while the robot
  points the shooter (`Macros.aimHeading`); in autonomous it is a turn in place (`Macros.aimAt`).
- **Nothing in the path can be counted.** There is no sensor between the funnel and the flywheel, so no
  code anywhere claims to know how many pieces are aboard. "Shoot one" is one tunnel pulse; "shoot all"
  is `Macros.PIECES_PER_LOAD` (4) pulses; `Macros.getShotsFired()` is a count of pulses. **The operator
  decides when the robot is empty**, and the telemetry says "pulses", never "pieces".
- **G407 is a human interlock.** Nothing stops the intake at four, because nothing knows it is at four.
  The operator stops it.
- **G408 is a human interlock too.** There is no colour sensing on the robot, so the code cannot refuse
  the opponent's NECTAR. The drivers must not intake it. The funnel is wide and passive, which makes
  that easier to get wrong than it sounds.
- **One path, no routing.** Every piece that enters goes to the flywheel. There is no diverter, no
  second storage and no decision to make.

## 4. Mechanism → subsystem map

| Physical mechanism | Actuators | Sensors | Software subsystem |
|---|---|---|---|
| Mecanum drivetrain | 4 × `DcMotorEx` | goBILDA Pinpoint + 2 pods (the Pinpoint's own IMU supplies heading) | `subsystems/Drivetrain` — one Pedro `Mecanum`, plus a `Follower` once AutoTune has run |
| Front funnel | none | none | none (geometry only) |
| Roller **and** tunnel | 1 × `DcMotorEx`, open-loop power | motor current only, and only while pulling | `subsystems/Intake` |
| Flywheel pair | 2 × `DcMotorEx`, open-loop power | one flywheel's encoder velocity | `subsystems/Shooter` (+ the `Drivetrain` aim lock) |
| Limelight 3A | none | the camera | `subsystems/Limelight` |
| — | — | **no piece sensors of any kind** | — |

`util/hardware/HardwareNames` holds every config string and nothing else: four drive motors,
`pinpoint`, `intake`, `shooter_left`, `shooter_right`, `limelight`. A name there that is not in the
Robot Controller configuration throws at OpMode init and says which device it was — that is deliberate
(docs/03 §14).

## 5. Dead design iterations in the CAD — do not re-add

The CAD file also contains **`Assembly 1`** and **`Assembly 2`**, each two 6000 RPM motors driving Gecko
wheels. **Both are abandoned design iterations. Neither is on the robot and neither has any code.**
They are recorded here so that the next person to open the CAD does not read them as mechanisms that
software forgot.

Two of them would also be illegal: seven motors plus either assembly's two is nine, and R503 caps a
robot at eight. If one of them is ever revived, something else loses its motor first, and that is a
hardware decision before it is a software one.

## 6. Electronics packaging and the motor budget

**Rule check (docs/04 §6):** R503 allows at most **8 DC motors and 8 servos**. The robot uses
**7 motors and 0 servos**:

| Ports | Mechanism |
|---|---|
| 4 | drivetrain |
| 1 | intake (roller + tunnel) |
| 2 | shooter |
| **1 free** | — |

A Control Hub plus at most one Expansion Hub (R701). `Robot.readSensors()` clears the bulk cache on
**every** `LynxModule`, so a second hub costs nothing in correctness. The Pinpoint and the Limelight are
both external computers on their own links (I2C and USB-Ethernet respectively), which is why the
Limelight is the one device whose calls are wrapped in try/catch (docs/03 §14).

Nothing may cross the path funnel → roller → tunnel → flywheel.

## 7. What the robot can and cannot sense

| It knows | From |
|---|---|
| where it is, and which way it faces | Pinpoint odometry, once AutoTune has produced `localizerConfig` |
| how far off the HIVE's AprilTags say its heading is | Limelight 3A, front-mounted, while the tags are in view |
| how fast a flywheel is turning | one shooter encoder, ticks/sec |
| whether the roller is jammed | intake motor current, sampled only while pulling |
| battery volts, loop rate | `Robot` |

| It does not know | Consequence |
|---|---|
| how many pieces are aboard | the operator counts; `shootAll` fires four pulses regardless |
| whether a piece is at the flywheel | a shot is a timed tunnel pulse, `Shoot.FEED_PULSE_MS` |
| what colour a piece is | G408 is a driver responsibility |
| whether a shot scored | nothing on the robot closes that loop; the drivers watch the CELL |

Adding a sensor is not "one line" any more: the reserved sensor-name block, the fail-soft lookup and
the trust policy that used to make it cheap were all deleted with the mechanisms that needed them
(HANDOFF §6). Fitting one means writing the subsystem member, the read in `Robot.readSensors()` and
the interlock that consumes it. That is the right price for a sensor nobody has chosen yet.

## 8. Geometry: all of it is still unmeasured

Because the CAD is a parts layout, **every one of these is a placeholder**:

| Number | Lives in | How it gets real |
|---|---|---|
| the field frame's HIVE and CELL positions | `game/Field` (INFERRED, from docs/04) | Onshape field CAD, then a tape measure at the first event |
| start pose, shooting spot, park | `game/FieldPoses` | drive the real field; correct the BLUE value and red follows |
| robot half-length, wall standoffs | `FieldPoses.ROBOT_HALF_LENGTH_INCHES` | measure the built chassis with pre-loads in |
| flywheel speed per distance | `Shooter`'s distance table | `Bench: Shooter`, four distances, HANDOFF §8 |
| camera yaw, and its height above the tiles | `Limelight.CAMERA_YAW_OFFSET_DEGREES` (0 = front) | a tag dead ahead must read tx ≈ 0 |
| which way "off the wall" is in autonomous | `Auto.LEAVE_POWER`, `LEAVE_MS` | the practice field |

Treat any path, autonomous or distance-derived flywheel speed as untested until these are measured,
because it is.

## 9. Open hardware questions the code cannot resolve

| Question | Affects |
|---|---|
| Which motors the flywheels actually are (make, encoder ticks/rev, free speed) | nothing in the code — that is the point of working in ticks/sec — but it decides whether 1300 t/s is near the ceiling or a third of it |
| Where the Limelight ends up, and how high | `Limelight.CAMERA_YAW_OFFSET_DEGREES`, and whether the tunnel or the shooter blocks the view of a CELL |
| Whether the rear-firing assumption survives the built robot | `Shooter.HEADING_OFFSET_RAD` (π) |
| Whether one tunnel pulse moves exactly one piece, at what power | `Shoot.FEED_PULSE_MS`, `Intake.IN` |
| Whether the tunnel holds a load on a slope without power | `Intake.IDLE` (0.35, borrowed from the reference robot) |
| Flywheel hood, exit angle, and the height of the CELL opening | the distance table's shape, and whether a shot from 96 in is possible at all |
| Whether the two flywheels are geared identically | `Shooter.SECOND_MOTOR_REVERSED` is geometry, not a tuning knob; unequal gearing would need two powers |
| Hub count and port assignment | `HardwareNames` only |

Whatever the answers, the software's shape does not change: seven motors, nothing counted, the
drivetrain aims.
