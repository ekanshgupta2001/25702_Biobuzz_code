# BIOBUZZ Robot Physical Architecture Specification

## Purpose

This file defines the **physical robot architecture only** for the 2026–2027 FTC BIOBUZZ robot.

It is meant to give Claude a clean understanding of:
- what physical subsystems exist,
- where those subsystems are located,
- how game pieces move through the robot,
- and how the major mechanisms are packaged.

Do **not** infer software architecture, autonomous routines, control logic, or strategy from this file.

---

## 1. Overall Robot

**Target footprint:** approximately **17.5 in × 17.5 in**

**Drivetrain:** four-wheel mecanum.

The robot should remain compact, low, and balanced. The central chassis volume is reserved primarily for the game-piece path. Battery, hubs, motors, and support electronics should be packaged around or underneath that path wherever practical.

### Main game-piece path

```text
FRONT

Wide Funnel
    ↓
Front Intake Roller
    ↓
Short Ramp
    ↓
Low Horizontal Storage Compartment
    ↓
Rear 90° Vertical Transfer
    ↓
Turret / Shooter

Additional routing may divert selected game pieces toward the Flower scoring mechanism.
```

The storage system is **not** a steep diagonal conveyor and **not** a tall hopper.

---

## 2. Front Funnel

The robot begins with a wide passive funnel.

### Geometry
- spans most of the robot width,
- extends roughly **2 inches** backward,
- uses angled walls,
- guides loose game pieces toward the center.

Approximate top view:

```text
FRONT

┌──────────────────────────────┐
│ \                          / │
│  \                        /  │
│   \                      /   │
│    \______        ______/    │
│           \______/           │
│              ↓               │
│           INTAKE             │
└──────────────────────────────┘
```

Likely construction:
- thin polycarbonate,
- lightweight printed guides,
- or similar low-mass panels.

The funnel itself should stay mechanically simple.

---

## 3. Front Intake

Immediately after the funnel is the main powered intake.

### Intended intake wheel

**goBILDA Intake Roller Wheel**
- 8 mm REX bore
- 16 mm diameter
- 16 mm width
- 30A durometer

These are the **small compliant intake roller wheels**.

The front intake should **not** use the larger Gecko transport wheels.

### Position

The intake roller shaft is located **above the ramp**.

Approximate side view:

```text
FRONT → BACK

      Intake Roller
   ○ ○ ○ ○ ○ ○ ○ ○
          ●
          ↓
        /
      /    Ramp
_____/________________ → Storage
```

The roller pulls the game piece inward and pushes it backward/up the ramp.

---

## 4. Intake Ramp

Directly beneath the intake is a short shallow ramp.

### Purpose
- supports the ball beneath the intake roller,
- guides it rearward,
- transitions it into the horizontal storage compartment.

### Packaging
- roughly another **1–2 inches** of effective front-to-back travel,
- shallow angle,
- compact,
- not intended to act as a long conveyor.

---

## 5. Horizontal Storage Compartment

This is the main storage system.

### Function
The storage compartment:
- receives game pieces from the intake/ramp,
- stores up to **4 game pieces**,
- moves them from front-middle toward the rear,
- feeds the rear vertical transfer.

### Geometry
The compartment is:
- long,
- low,
- essentially horizontal,
- centrally located,
- enclosed or semi-enclosed like a shallow tunnel/channel.

Approximate top view:

```text
FRONT
        ↓

┌─────────────────────┐
│ ○                 ○ │
│                     │
│ ○       ●         ○ │
│                     │
│ ○       ●         ○ │
│                     │
│ ○       ●         ○ │
│                     │
│ ○       ●         ○ │
└──────────┬──────────┘
           ↓
          BACK
```

Legend:
- `●` = game piece
- `○` = Gecko/compliant transport wheel

### Capacity
Maximum intended controlled capacity:
- **4 game pieces**

Approximate BIOBUZZ game-piece sizes:
- Pollen: ~2.8 in diameter
- Nectar: ~3.6 in diameter

The channel needs sufficient compliance and clearance for either.

### Transport
Larger Gecko-style compliant wheels/rollers are positioned along the sides of the storage compartment.

Their purpose is to:
- maintain contact with the game pieces,
- move them rearward,
- keep the queue controlled,
- prevent loose uncontrolled movement.

The front intake rollers and the storage Gecko wheels are two separate mechanisms.

---

## 6. Rear 90-Degree Vertical Transfer

At the back of storage, the game-piece path turns upward approximately 90 degrees.

This forms an L-shaped path.

```text
                         UP TO SHOOTER
                              ↑
                         ○    ●    ○
                         ○    ↑    ○
                         ○    ●    ○
                         ○    ↑    ○
                         ○    ●    ○
                         ○    ↑    ○
                              │
● → ● → ● → ● ───────────────┘
   HORIZONTAL STORAGE
```

### Construction intent
Use opposing compliant Gecko wheels or similar compliant rollers.

The transfer should:
- accept a game piece directly from storage,
- grip it from both sides,
- move it upward,
- feed it directly into the upper scoring assembly.

Avoid unnecessary extra conveyors between storage and shooter.

---

## 7. Turret

The shooter is mounted on or integrated into a compact rotating turret.

### Preferred location
- rear-center of the robot,
- above the rear portion of the storage/vertical transfer,
- positioned to maintain reasonable balance.

### Physical purpose
The turret provides horizontal aiming while allowing the drivetrain to remain independently oriented.

### Packaging requirements
The turret should:
- stay compact,
- avoid unnecessary height,
- avoid hanging far off one side,
- receive game pieces from below through the vertical transfer.

---

## 8. Shooter

The shooter sits above the rear section of the robot.

### General type
A compliant flywheel-style shooter capable of handling both:
- Pollen,
- Nectar.

### Physical feed path

```text
Horizontal Storage
        ↓
Rear 90° Vertical Transfer
        ↓
Turret / Shooter Feed
        ↓
Flywheel Shooter
```

The vertical transfer should feed as directly as possible into the shooter.

Avoid adding extra horizontal transfer stages unless testing proves they are necessary.

---

## 9. Vertical Extension / Flower Mechanism

The robot will also include a **vertical extension mechanism**.

Its main purpose is **Flower scoring**, not HIVE shooting.

The shooter should function independently of the vertical extension.

### Current design intent
The exact Flower mechanism is not fully locked yet.

However, CAD and packaging should reserve:
- vertical travel space,
- mounting points,
- clearance for a compact Flower scoring assembly.

The rest of the robot should not block future Flower mechanism development.

---

## 10. Game-Piece Routing / Diverter

The robot may contain a compact routing actuator/diverter.

Current physical intent:
- one main intake,
- one main storage system,
- selected pieces can be redirected toward either HIVE shooting or Flower scoring.

Do not create a second full intake or second full storage system unless explicitly requested later.

---

## 11. Limelight Mounting

A Limelight camera is planned.

### Preferred physical configuration
- rigidly mounted to the chassis,
- not mounted on a scanning servo,
- positioned high enough to avoid the intake/storage blocking its view,
- placed so the turret/shooter does not permanently obstruct it.

Potential mounting regions:
- upper front-center,
- upper-center,
- rear-center with a clear forward view.

The final mounting position should be chosen after turret/shooter CAD is established.

---

## 12. Sensors and Reserved Space

Physical room should be reserved for sensors around:
- intake entrance,
- storage entrance,
- storage/full detection,
- vertical transfer,
- shooter feed.

Do not package the robot so tightly that sensors cannot be added later.

---

## 13. Electronics Packaging

Likely components:
- Control Hub,
- Expansion Hub if required,
- battery,
- wiring distribution,
- vision hardware.

Preferred placement:
- side cavities next to the storage,
- beneath portions of storage,
- low front or rear corners depending on motor placement.

The electronics should not block the main mechanism path:

```text
Funnel
→ Intake
→ Ramp
→ Storage
→ Vertical Transfer
→ Shooter
```

---

## 14. Weight Distribution

Higher-mounted mechanisms include:
- turret,
- shooter,
- vertical Flower hardware.

Therefore:
- battery should remain low,
- hubs should remain low,
- drivetrain motors should remain low,
- funnel and intake should stay lightweight,
- storage walls should stay lightweight.

Avoid unnecessary high-mounted mass.

---

## 15. Approximate Vertical Packaging

Think of the robot as three layers.

### Upper layer
- turret,
- shooter,
- upper vertical transfer,
- Flower scoring extension/mechanism.

### Middle / low mechanism layer
- intake,
- ramp,
- horizontal four-piece storage,
- lower vertical transfer.

### Bottom layer
- mecanum drivetrain,
- battery,
- hubs,
- wiring,
- support structure.

Approximate side view:

```text
              ┌─────────────────┐
              │ TURRET/SHOOTER  │
              └────────┬────────┘
                       ↑
                 VERTICAL FEED
                       ↑
────────────────────────────────
        HORIZONTAL STORAGE
────────────────────────────────
       ELECTRONICS / BATTERY
════════════════════════════════
         MECANUM CHASSIS
════════════════════════════════
```

---

## 16. Physical Design Priorities

When evaluating physical designs, prioritize:

1. Reliable game-piece path
2. Minimal jam points
3. Low center of gravity
4. Compact packaging
5. Easy maintenance
6. Mechanism accessibility
7. Room for sensors
8. Room for Flower-mechanism refinement
9. Minimal unnecessary transfers
10. Low moving mass

---

## 17. Designs to Avoid

Do not reinterpret the robot as:
- a giant hopper robot,
- a steep diagonal conveyor robot,
- a tall elevator-based storage robot,
- a dual-intake robot,
- a robot with separate full storage systems for every scoring objective,
- a robot where the entire storage assembly moves vertically without a clear reason,
- a robot whose shooter requires the drivetrain to aim,
- a robot with mechanisms added simply because empty space exists.

---

## 18. Final Architecture Summary

```text
FRONT

[ Wide ~2-inch Funnel ]
          ↓
[ 16 mm 30A Intake Roller Wheels ]
          ↓
[ Short Ramp ]
          ↓
[ Low Horizontal 4-Game-Piece Storage ]
          ↓
[ Rear 90° Gecko Vertical Transfer ]
          ↓
[ Compact Rear-Center Turret ]
          ↓
[ Compliant Flywheel Shooter ]

Additional:
- vertical Flower scoring extension,
- compact routing/diverter mechanism,
- fixed Limelight,
- mecanum drivetrain,
- low-mounted electronics and battery.
```

Treat this physical architecture as the baseline robot configuration unless it is explicitly changed later.
