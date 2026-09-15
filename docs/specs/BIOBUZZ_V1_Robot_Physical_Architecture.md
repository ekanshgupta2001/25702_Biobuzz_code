# BIOBUZZ V1 Robot Physical Architecture

## Purpose

This document defines the **V1 physical robot architecture only**.

V1 is intentionally limited to the systems required to:

1. collect game pieces,
2. store up to four game pieces,
3. transfer them to the shooter,
4. aim and launch them.

This file does **not** include:
- Flower scoring,
- vertical extension,
- horizontal extension,
- secondary scoring mechanisms,
- diverters for alternate destinations,
- autonomous strategy,
- software architecture,
- control logic,
- or future V2 mechanisms.

The goal of V1 is to create a simple, reliable, testable base robot before additional mechanisms are added.

---

# 1. Overall V1 Robot

## Target Size

Approximate starting footprint:

- **17.5 in × 17.5 in**

## Drivetrain

- Four-wheel mecanum drivetrain
- Compact FTC-style chassis
- Low center of gravity
- Central mechanism path kept open

The physical layout should be built around one continuous game-piece path:

```text
FRONT

Funnel
  ↓
Intake
  ↓
Short Ramp
  ↓
Horizontal Storage
  ↓
Rear 90° Vertical Transfer
  ↓
Turret
  ↓
Flywheel Shooter
```

V1 should be mechanically simple enough that each stage can be tested independently.

---

# 2. Front Funnel

The robot begins with a passive funnel across most of the front width.

## Geometry

- approximately **2 inches deep**
- angled side walls
- funnels game pieces toward the center intake
- lightweight and low-profile

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

## Construction Intent

Likely materials:

- thin polycarbonate,
- lightweight printed guides,
- lightweight sheet material.

The funnel is passive and should not contain the larger Gecko transport wheels.

---

# 3. Front Intake

Immediately after the funnel is the powered intake.

## Intake Wheel Specification

Use:

**goBILDA Intake Roller Wheel**
- 8 mm REX bore
- 16 mm diameter
- 16 mm width
- 30A durometer

These are the small compliant intake wheels.

Do **not** use the larger Gecko wheels for the front intake.

## Placement

The intake roller sits above the ramp.

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

The intake roller should pull the ball inward and push it backward onto the ramp.

---

# 4. Short Ramp

A short ramp sits directly underneath the intake.

## Purpose

The ramp:

- supports the ball under the intake roller,
- guides the game piece rearward,
- transitions the game piece into the storage compartment.

## Geometry

- short,
- shallow,
- compact,
- approximately **1–2 inches** of effective front-to-back travel after the funnel.

The ramp should not become a long inclined conveyor.

---

# 5. Horizontal Storage Compartment

The main V1 storage system is a low horizontal container.

## Function

The storage compartment:

- receives game pieces from the intake/ramp,
- stores up to **4 game pieces**,
- moves the queue from front to rear,
- feeds the rear vertical transfer.

## Geometry

The storage should be:

- long,
- low,
- essentially horizontal,
- enclosed or semi-enclosed,
- centrally located.

It should look like a shallow tunnel or channel, not a hopper.

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
- `○` = compliant transport wheel

## Capacity

Maximum intended V1 capacity:

- **4 game pieces**

The channel must accommodate both game-piece sizes:

- Pollen: approximately **2.8 in**
- Nectar: approximately **3.6 in**

## Transport Wheels

Use larger compliant Gecko-style wheels or similar compliant rollers along the sides.

Their role is to:

- grip game pieces,
- advance them toward the rear,
- maintain control of the queue,
- reduce random movement inside the storage compartment.

The storage wheels are separate from the front intake roller.

---

# 6. Rear 90-Degree Vertical Transfer

At the rear of the storage compartment, the game-piece path turns upward.

This forms an L-shaped transport path.

Approximate side view:

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

## Construction Intent

Use opposing compliant Gecko wheels or similar rollers.

The vertical transfer should:

- accept a game piece directly from storage,
- grip it from both sides,
- move it upward,
- feed it directly into the shooter/turret assembly.

The horizontal-to-vertical transition must be compact and smooth.

Avoid unnecessary intermediate conveyors.

---

# 7. Turret

The shooter is mounted on a compact rotating turret.

## Location

Preferred location:

- rear-center of the robot,
- above the rear storage/vertical-transfer section.

## Purpose

The turret allows the shooter to rotate independently of the drivetrain.

## Packaging Requirements

The turret should:

- remain compact,
- stay reasonably centered,
- avoid excessive height,
- avoid large off-center mass,
- receive the game piece from below.

The turret should not require the entire storage system to move.

---

# 8. Flywheel Shooter

The shooter sits on top of the turret.

## General Design

The shooter should be a compliant flywheel-style system capable of launching both:

- Pollen,
- Nectar.

The same shooter should be used for both game-piece types.

## Physical Feed Path

```text
Horizontal Storage
        ↓
Rear 90° Vertical Transfer
        ↓
Turret Feed
        ↓
Flywheel Shooter
```

The vertical transfer should feed as directly as possible into the shooter.

Avoid unnecessary extra transfer stages.

## Design Intent

The shooter should be:

- compact,
- mechanically rigid,
- compatible with variable game-piece size,
- easy to access for tuning and wheel changes,
- mounted securely to the turret.

---

# 9. V1 Electronics Packaging

Electronics should be placed around the mechanism path rather than through it.

Likely components include:

- Control Hub,
- Expansion Hub if required,
- battery,
- motor and servo wiring,
- vision hardware if installed during V1.

Preferred locations:

- side cavities beside storage,
- underneath portions of storage,
- low front/rear corners.

The main rule is:

```text
DO NOT BLOCK:

Funnel
→ Intake
→ Ramp
→ Storage
→ Vertical Transfer
→ Shooter
```

---

# 10. V1 Weight Distribution

Higher-mounted V1 mechanisms:

- turret,
- shooter.

Therefore heavier support components should remain low.

Preferred placement:

- battery low,
- hubs low,
- drivetrain motors low,
- funnel lightweight,
- intake lightweight,
- storage side walls lightweight.

Keep the center of gravity as low and central as practical.

---

# 11. V1 Mechanism Layers

## Upper Layer

- turret,
- flywheel shooter,
- top of vertical transfer.

## Middle / Low Mechanism Layer

- front intake,
- short ramp,
- horizontal four-game-piece storage,
- lower vertical transfer.

## Bottom Layer

- drivetrain,
- battery,
- electronics,
- chassis structure.

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

# 12. V1 Physical Priorities

V1 should prioritize:

1. Reliable intake
2. Reliable storage
3. Reliable horizontal-to-vertical transfer
4. Reliable shooter feeding
5. Low jam rate
6. Easy maintenance
7. Easy sensor mounting
8. Low center of gravity
9. Simple packaging
10. Fast iteration

---

# 13. What V1 Should NOT Include

Do not include in V1:

- Flower scoring mechanism,
- vertical extension,
- horizontal extension,
- secondary intake,
- alternate storage path,
- separate Nectar storage,
- separate Pollen storage,
- alternate scoring diverter,
- complicated endgame mechanisms,
- moving storage carriage,
- large elevator systems.

V1 should remain focused on proving the core scoring architecture.

---

# 14. Final V1 Architecture Summary

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
```

## V1 Design Philosophy

> **Collect reliably. Store predictably. Transfer cleanly. Shoot consistently.**

Everything else belongs in later robot versions.
