# BIOBUZZ Season Analysis (FTC 2026-27)

Compiled 2026-09-13, one day after kickoff, from the Competition Manual V1, the Event Field Setup
Guide V1.0, Team Update 00, the SDK v12.0 release notes, vendor pages and the first Chief Delphi
threads. Every fact is tagged:

- **[OFFICIAL]** verbatim or read from a FIRST-hosted document or figure.
- **[COMMUNITY]** vendor sites, forums, third-party repos.
- **[INFERRED]** our own derivation, with the evidence chain.

Numbers read off manual figures rather than body text carry a ±1 in caveat; the Onshape CAD is the
official geometry authority (manual §9.1). Re-check this document against Team Updates and the
Q&A (opens 2026-09-28). The pre-kickoff "BIOBUZZ (Forecast)" tab in the team's strategic compendium
spreadsheet is superseded by everything here.

---

## 1. Identity and timeline

| Item | Value |
|---|---|
| Game | **BIOBUZZ presented by RTX**, part of the FIRST CANOPY season (FLL BIOGLOW, FRC BIOCORE) [OFFICIAL] |
| Kickoff | 2026-09-12; SDK v12.0 released the same day [OFFICIAL] |
| Manual | **Competition Manual V1** (single 16-section document; the Part 1 / Part 2 split is retired). Relevant: §8 Overview, §9 ARENA, §10 Game Details, §11 Game Rules (G), §12 Robot Rules (R), §13 Tournament, §16 Glossary [OFFICIAL] |
| Team Updates | TU00 (2026-09-12): Competition Integrity Contract, new **STRATEGIC** term (§10.6), **R503 servos reduced to 8**, R105 expansion re-dimensioned, R801 reworded, old R903 gamepad rule removed [OFFICIAL] |
| Q&A opens | 2026-09-28 12:00 ET [OFFICIAL] |
| ftc-docs AprilTag-Clusters tech tip | scheduled 2026-09-14 (stub today) [OFFICIAL] |
| Earliest League Meet / Qualifier | 2026-10-19; FTC Live minimum event software version 2026-10-15 [OFFICIAL] |
| Regionals | from 2026-11-30; last FCMP-advancing date 2027-03-21 [OFFICIAL] |
| FIRST Championship | 2027-04-28 to 05-01 (venue not stated) [OFFICIAL] |
| Media | Game animation https://youtu.be/sUH3z5a5S9I; field tour https://youtu.be/47X9sYnPijw; manual chatbot https://ftc-cmchatbot.firstinspires.org/ [OFFICIAL] |

## 2. Field

### 2.1 Dimensions [OFFICIAL §9.2]
144 in × 144 in inside the perimeter, 36 tiles of 24 in in a 6 × 6 grid (AndyMark am-5850_Full,
tiles am-2499, perimeter am-0481). Tolerance ±1 in on every nominal dimension. Some events raise the
field on platforms.

### 2.2 Tile grid [OFFICIAL §9.4, Setup Guide §6, Figure 9-5]
Columns **A–F left to right from the audience**, rows **1–6 from the audience wall to the far wall**.
A1 is the audience-side red corner, F6 the far-side blue corner. The **red ALLIANCE AREA is on the
left** from the audience (column A wall); blue on the right (column F wall). During AUTO the red side
is columns A/B/C and the blue side D/E/F (§9.5, G304, G402).

### 2.3 Elements [OFFICIAL §9.2, §9.6, §9.7]
Only two kinds of fixed element: one **HIVE structure** and four **FLOWERS**.

**HIVE structure** at field centre (tiles C3, C4, D3, D4). Frame 49.46 in wide × 38.95 in deep,
pivot axis **43.95 in above the tiles**, two triangular metal sides joined by a crossbar. Each alliance
has one **HIVE** = two **CELLS** on a bi-stable pivot; the CELLS are 18.8 in apart. **CELL opening
≈ 20 in wide × 14 in tall × 12 in deep**; the bottom face of each CELL carries an AprilTag cluster.
Red HIVE on the red half (column C), blue on column D. [INFERRED] Up-CELL opening centre ≈ 53 in
above the tiles (pivot 43.95 in + ~9.4 in); confirm in the Onshape CAD before fixing shooter geometry.

**FLOWER** (four, one per perimeter wall): top opening ≈ 4 in diameter at ≈ 21.5 in above the tiles
with a 1.25 in backstop; bottom **retrieval opening ≈ 3.55 in tall × 3.57 in deep** (only POLLEN,
2.8 in, fits through; NECTAR, 3.6 in, does not); floor ring 0.4 in tall with a 2.79 in hole that seats
the bottom POLLEN. Scoring volume = between the top ring and the middle ring (separate CAD).
Positions read from the top-view figures (±1 in) [INFERRED numbers, OFFICIAL "one per wall"]:

| FLOWER | Wall | Position |
|---|---|---|
| Far | row-6 wall | B/C tile seam, ≈ 48 in from the red wall |
| Blue | column-F wall | row 4/5 seam, ≈ 96 in from the audience wall |
| Audience | row-1 wall | D/E seam, ≈ 96 in from the red wall |
| Red | column-A wall | row 2/3 seam, ≈ 48 in from the audience wall |

### 2.4 Zones [OFFICIAL §9.3, Setup Guide §8]

| Zone | Size | Red | Blue |
|---|---|---|---|
| ALLIANCE AREA (outside the field) | 97 in wide × 54 in deep | column-A wall | column-F wall |
| LOADING ZONE | 23 in × 11 in | tile **A5** | tile **F2** |
| GARDEN | 23 in × 2 in strip | tile **A1** | tile **F6** |

No ramps, barriers, climb or ascent zones. PARK = at least partially in your own LOADING ZONE.

### 2.5 Symmetry: 180° rotation, not a mirror [OFFICIAL Setup Guide §8.3/8.4, Figures 9-2, 9-5, 10-2]

| Element | Red | Blue | Mirror across the C/D line would give | 180° rotation gives |
|---|---|---|---|---|
| GARDEN | A1 | F6 | F1 ✗ | F6 ✓ |
| LOADING ZONE | A5 | F2 | F5 ✗ | F2 ✓ |
| Wall FLOWERS | far @ B/C, red wall @ 2/3 | audience @ D/E, blue wall @ 4/5 | ✗ | ✓ |
| Starting up-CELL | audience-side | far-side | ✗ | ✓ |

Code consequence: `FieldConstants.SYMMETRY = ROTATE_180`, i.e. `(x, y, h) → (144 − x, 144 − y, h + π)`
in corner-origin Pedro coordinates. A reflection would put every blue auto in the wrong place.

## 3. AprilTags

**Every tag is on a moving HIVE CELL. The SDK v12.0 notes say: "since BIOBUZZ AprilTags move, they
are not suitable for absolute Field Localization."** [OFFICIAL] There is no tag position table, no
Limelight `.fmap`, and there will not be one. Tags are **aiming targets only**.

- Family 36h11, **3.25 in** square, arranged as **clusters of four** on one sticker on the bottom face
  of each CELL, facing down, bottom edge toward field centre [OFFICIAL §9.9].
- IDs (Figure 9-17): red scoring-side CELL **33, 32, 31, 30**; red audience-side **34, 35, 36, 37**;
  blue audience-side **38, 39, 40, 41**; blue scoring-side **45, 44, 43, 42**. Valid range 30–45.
- Cluster geometry (Figure 9-15): inner tags ±2.75 in, outer ±6.5 in from the cluster centreline;
  reference holes ±7.0 in; centreline 2.75 in above the holes; front of CELL to hole centreline
  9.938 in.
- **Cluster origin = centre of the CELL opening** (SDK notes), so a cluster detection's
  `ftcPose` range/bearing/elevation points straight at the mouth you are shooting into.
- Which CELL starts up (Figure 10-2): **red audience-side (tags 34–37)**, **blue far-side (tags
  42–45)**. After each TIP the target flips (red 34–37 ↔ 30–33, blue 42–45 ↔ 38–41).
- SDK v12.0 **breaks legacy AprilTag OpModes**: detections are now `AprilTagSingleDetection` or
  `AprilTagClusterDetection` (`metadata.name`, `percentClusterFound`), and a cluster gives full 6-DOF
  pose from a single visible member tag [OFFICIAL].
- Limelight: no BIOBUZZ field map on the downloads page; use its AprilTag pipeline for tx/ty/range
  to the cluster, or a colour pipeline for game pieces [OFFICIAL by absence].
- The generic FTC field frame (origin at field centre, X along the red wall, Y away from it) is
  unchanged but unused for localisation this season.

## 4. Game pieces [OFFICIAL §9.8; masses COMMUNITY (AndyMark)]

| | POLLEN | NECTAR |
|---|---|---|
| Diameter | **≈ 2.8 in (7.1 cm)** | **≈ 3.6 in (9.1 cm)** |
| Colour | yellow, neutral | red or blue, **alliance-specific** |
| Material | perforated Gopher ResisDent polyethylene ball | same |
| Part | am-5851_yellow | am-5852_red / am-5852_blue |
| Mass | ≈ 25 g | ≈ 41 g |
| Count | 40 | 8 red + 8 blue |

"POLLEN and NECTAR are not perfectly spherical and may vary in size" [OFFICIAL]. Colour (yellow vs
red vs blue) and diameter each classify a piece on their own; no markings.

Start of match [OFFICIAL §10.3.1]: 4 POLLEN in each FLOWER, 4 in each GARDEN, **4 pre-loaded in each
robot**; 3 NECTAR of the alliance colour in each up-facing CELL; 5 NECTAR per ALLIANCE AREA.
During the match [OFFICIAL §10.1, G426, G427]: NECTAR enters from the ALLIANCE AREA by a drive-team
member, one per own-colour HIVE TIP, or all remaining once ≤ 60 s remain, and must contact a tile in
the LOADING ZONE before touching a robot. POLLEN never re-enters through humans. Whether TIP-unlocked
NECTAR may be banked is an open Q&A question [COMMUNITY].

## 5. Match structure and scoring

### 5.1 Timing [OFFICIAL §10.1, §10.4, Table 9-1]
AUTO 0:30 → transition 0:08 (**no powered movement, G403**) → TELEOP 2:00. Displayed timer runs
2:30 → 0:00. **No endgame period.** Timed events: **1:00 remaining = FLOWER ownership unlocked**
(NECTAR may enter a FLOWER, G410; all remaining NECTAR may be loaded, G426); **0:20 = final
warning** (train whistle). The visual field timer is authoritative over audio (§9.11).
Code: `MatchClock` (AUTONOMOUS_MS 30 s, TELEOP_MS 120 s, FINAL_WARNING_MS 20 s; `isFinalSeconds()`,
`isExpired()`). The FLOWER window has no code until a FLOWER mechanism exists.

### 5.2 Scoring [OFFICIAL Table 10-2]

| Action | AUTO | TELEOP |
|---|---|---|
| LEAVE (no longer touching the perimeter wall) | 3 | — |
| PARK (partially in own LOADING ZONE) | 5 | 5 |
| **HIVE TIP** | **20** | **20** |
| POLLEN / NECTAR remaining in the up-facing CELL at end | — | 2 each |
| FLOWER: bottom NECTAR bonus (your colour lowest) | — | 5 |
| FLOWER: each element in a FLOWER you own | — | 2 each |
| GARDEN: each element in your GARDEN | — | 1 each |

Ranking points: WIN 3, TIE 1, **SWARM** (LEAVE + PARK points ≥ 16 at ordinary events), **POLLINATOR 1**
(≥ 4 TIPS), **POLLINATOR 2** (≥ 7 TIPS); championship thresholds TBA [OFFICIAL Table 10-3]. Ranking
sort: RP average, then match points excluding fouls, then average TIPS, then average AUTO points.
Penalties: MINOR FOUL 5, MAJOR FOUL 20 to the opponent [OFFICIAL Table 10-4].

### 5.3 HIVE TIP mechanics [OFFICIAL §10.5.1, Setup Guide §12]
A HIVE TIPS when the up-CELL becomes the down-CELL and the damper contacts the frame. Field staff
calibrate every HIVE to tip on **8 POLLEN**, or **3 POLLEN + 3 NECTAR** (no tip on 2 POLLEN + 3
NECTAR or 7 POLLEN). Only launching into the up-facing CELL may cause a TIP (G417); shooting at the
down-CELL while a HIVE is tipping may prevent the TIP.

[INFERRED, high value] The up-CELL starts with 3 NECTAR and each robot pre-loads 4 POLLEN, so **the
first TIP needs only 3 POLLEN**: an AUTO that makes three shots scores 20 + LEAVE 3 + PARK 5 = 28.
Every later TIP starts from an empty CELL and needs 8 POLLEN (or a NECTAR mix). POLLINATOR 1 (4 TIPS)
≈ 3 + 3 × 8 = 27 POLLEN-equivalents per alliance; POLLINATOR 2 (7 TIPS) ≈ 51, more than the 40 POLLEN
on the field, so 7 TIPS requires recycling POLLEN out of tipped CELLS and FLOWER bottoms and mixing in
NECTAR.

### 5.4 FLOWER and GARDEN [OFFICIAL §10.5.2, §10.5.3]
An element scores when partially inside the FLOWER scoring volume. The alliance whose colour has the
**top-most** scored NECTAR **owns** the FLOWER and earns 2 per element in it regardless of who placed
them; the **bottom-most** NECTAR colour earns 5. Elements enter only through the top and only POLLEN
leaves through the bottom (G418). GARDEN scoring is by zone colour, 1 per element, unprotected.
[INFERRED] Ownership is decided by the last NECTAR in, so filling FLOWERS with POLLEN early and losing
the cap gifts the points; cap late.

### 5.5 Rules that constrain software [OFFICIAL §11]

| Rule | Constraint | Code consequence |
|---|---|---|
| **G407** | **CONTROL at most 4 SCORING ELEMENTS at a time**; teams are told to build guards and "systems to prevent active pickup/intaking of more than 4" | nothing on the robot can count, so the **operator is the interlock** (§9 item 1); a momentary 5th reversed out is "likely not STRATEGIC" |
| **G408** | may not CONTROL the opponent's NECTAR | colour classification at the intake or storage entrance, or a 3.6 in reject gate |
| **G409** | may not catch or deflect elements released by a tipping HIVE | do not park under the HIVE; no open-top hopper |
| **G410** | **no NECTAR into a FLOWER until ≤ 60 s remain**, MAJOR FOUL per NECTAR | V1 has no Flower mechanism; a V2 macro would gate on 60 s remaining (`MatchClock.getRemainingMs()`) |
| G411 | no hoarding to deny the opponent | — |
| G415 | no grabbing/entangling field elements; a concave alignment shape around a FLOWER is allowed | FLOWER alignment guide is legal |
| G416 / R105 | expansion limits must be **physically** constrained; software limits do not satisfy R105 | soft limits are a convenience only |
| G417 | no manipulating HIVE motion except by launching into the up-CELL | pause the shooter during a TIP |
| G402 | during AUTO, priority on own side; crossing "may be seen as STRATEGIC" | keep AUTO on own columns |
| G403 / G404 | no powered movement during the 8 s transition or after TELEOP ends | delay TeleOp INIT if init moves a servo |
| Launching | **no launch zone, no velocity or height cap**; flywheels explicitly legal (R801); do not eject elements from the field (G405) | shoot from anywhere |

## 6. Robot rules [OFFICIAL §12, TU00]

| Rule | Requirement | Our spec |
|---|---|---|
| R102 | starting configuration inside an 18 in cube; pre-loads may protrude | ✓ 17.5 in footprint |
| R104 | **no weight limit** | — |
| R105 | after start, within 18 × 24 × 29 in tall, physically constrained | the shooter must stay inside |
| **R503** | **max 8 DC motors and 8 servos** across all configurations | V1 needs 8–10 motors: drivetrain 4, intake, storage (1–2), transfer, shooter (1–2). **A single flywheel and a single storage motor fit exactly**; otherwise storage + transfer must share a motor |
| R701 | one Control Hub (or phone + Expansion Hub) plus at most one Expansion Hub | ✓ |
| **R702** | **Limelight 3A is the only permitted programmable vision coprocessor**; Limelight 3G, OAK-1, OpenMV banned | ✓ Limelight 3A |
| **R704** | no continuous video stream; **FTC Dashboard, FTControl Panels and similar streaming tools are prohibited during matches** | no Panels, no FTC Dashboard; AutoTune (binds a web server whenever present) is **in every build** for now, so the everyday APK is not match legal: before an event it is removed by hand along with the sources that import it (HANDOFF §4). Nothing on the card warns |
| R708 | single-sensor UVC webcams only; no stereo | — |
| R801 | no pneumatics, blowers, vacuums; flywheels/rollers fine | ✓ |
| SDK | **v12.0** (2026-09-12), Android Studio Narwhal 3 Feature Drop+; no minimum version mandated for inspection | repo is on SDK 11.2.1 via the Pedro Quickstart; **fine**, because the Limelight reads the tags, not the SDK's `AprilTagProcessor` (§9 item 13) |

## 7. Strategy signals [COMMUNITY, 24 h after kickoff]
- All four vendor StarterBots (goBILDA, AndyMark, REV, Studica) are **4-piece indexed magazine +
  flywheel shooter** robots; goBILDA's carries and launches "up to four POLLEN in quick succession"
  and pulls POLLEN from FLOWERS. This is our architecture.
- Chief Delphi: the CELL opening is perpendicular to the audience/far walls, so a **wall-referenced
  heading is a vision-free aiming fallback** (auto-aiming thread); defence is "viable" given limited
  contact rules and a small scoring area; strong pushback on the subjective STRATEGIC framework.
- FUN Robotics Ri30H content: catapults, turrets, Flower mechanisms, colour vision for POLLEN and
  NECTAR (videos: hive calibration https://youtu.be/U5TY2FS0CRE, strategies https://youtu.be/yk3qig3L3Zg).
- No credible pieces-per-match numbers yet. TIPS are the currency; see §5.3 arithmetic.
- [INFERRED] A turret is optional if the drive heading controller is good, but a turret lets the
  drivetrain keep cycling while the shooter tracks the CELL, which suits a 4-piece cycle robot.

## 8. Software resources
| Resource | Status |
|---|---|
| FTC SDK v12.0 | `AprilTagGameDatabase.getCurrentGameTagLibrary()` returns BIOBUZZ tags; new cluster detection types; `ColorBlobLocatorProcessor` available; **no BIOBUZZ colour swatch constants** (tune HSV ourselves) [OFFICIAL] |
| ftc-docs | cluster tech tip stub (full 2026-09-14); AprilTag ID/metadata and field-frame pages not yet updated [OFFICIAL] |
| Limelight | no BIOBUZZ `.fmap`; 3A legal, 3G banned [OFFICIAL] |
| Pedro Pathing / Road Runner | no BIOBUZZ field image or presets yet (visualizer still DECODE) [COMMUNITY] |
| Official field assets | Initial Field Element Assembly Guide V1.0, Event Field Setup Guide V1.0, Onshape field CAD, FLOWER scoring-volume CAD, STEP export; printable AprilTag PDF and dimensioned 2-D drawing "coming soon" [OFFICIAL] |

## 9. Implications for our robot and code

Rewritten 2026-09-27 against the robot in the CAD (see `HANDOFF.md` §1 and `docs/02`). Sections 1–8 above
are sourced season facts and are unchanged; this section is the only part that tracks our own build.

| # | Item | Verdict for this robot |
|---|---|---|
| 1 | G407: at most 4 controlled scoring elements | **Nothing on the robot enforces it.** There is no sensor in the roller or tunnel, so nothing counts pieces. The operator is the interlock; Shoot All fires four pulses (`Macros.PIECES_PER_LOAD`). Physical guards are the only other protection. |
| 2 | POLLEN ≈ 2.8 in, NECTAR ≈ 3.6 in | Verified. The tunnel must pass either. Nothing in software distinguishes them. |
| 3 | G408: never control the opponent's NECTAR | **Also the operator's job.** There is no colour sensing on this robot, so the code cannot reject a piece; the driver must not intake the wrong NECTAR. Reversing the roller (operator LB) is the only way to shed one. |
| 4 | Fixed flywheel firing out the rear, aimed by the drivetrain heading | Legal; no launch zone, no velocity or height cap, flywheels explicitly allowed (R801). Target is the up-CELL opening, 20 × 14 × 12 in. TIP on 8 POLLEN or 3 POLLEN + 3 NECTAR, so the **first TIP needs only 3 POLLEN** — which is why autonomous spends its budget shooting. `Shooter.HEADING_OFFSET_RAD` = π. |
| 5 | FLOWER scoring | **No mechanism, no code.** A FLOWER matters only as a POLLEN source: the roller can pull POLLEN from the 3.55 in bottom retrieval opening, which G418 allows. |
| 6 | AprilTag localisation | **Not available this season**: every tag rides a moving HIVE CELL. The pose is the Pinpoint's alone. Tags refine the *aim* only, through `Macros.aimHeading`, which keeps the tags' disagreement with odometry for `AIM_BIAS_MAX_AGE_MS` after they leave view — necessary because the camera faces front and the shooter fires rearward. |
| 7 | Alliance pose conversion | **180° rotation.** `FieldConstants.forAlliance` rotates; the three-armed `Symmetry` enum was deleted, since a run-time selector over a fact the field cannot change is only somewhere for a wrong value to hide. |
| 8 | Match clock | 30 s AUTO, 8 s transition, 120 s TELEOP, no endgame. `MatchClock` models the 0:20 warning (one long rumble) and `isExpired()` drives the autonomous stop (G403). The 1:00 FLOWER unlock has no code because there is no FLOWER mechanism. |
| 9 | Start positions | G304: own side, touching the wall, outside the LOADING ZONE and FLOWER volumes, holding 4 POLLEN. Placed **rear (shooter) toward the up-facing CELL**. One start pose per alliance (`FieldPoses.BLUE_START_FACING_HIVE`); the two-option `StartPosition` enum was deleted with the selector. First target: red tags 34–37, blue 42–45. |
| 10 | Motors and servos | **7 motors, 0 servos**, against R503's limit of 8 and 8: four drive, one for the roller *and* tunnel together, two flywheels. One spare port. |
| 11 | Electronics | One Control Hub; Limelight 3A (R702's only legal coprocessor); **no Panels or Dashboard** (R704). AutoTune is in every build and must be stripped by hand before an event (HANDOFF §4). |
| 12 | Size | 18 in cube at start; 18 × 24 × 29 in during play, physically constrained; no weight limit. The CAD is a parts layout, so the packaged footprint is still unverified. |
| 13 | SDK version | **11.2.1 is fine.** The v12.0 upgrade was previously listed as a prerequisite for AprilTag work, but the tags are read by the *Limelight's own pipeline* over USB-Ethernet — the SDK's `AprilTagProcessor` and its new cluster API are never used. The upgrade is a future task, not a blocker. |
| 14 | Geometry | **Everything is still a placeholder.** The CAD gave mechanisms, not positions, so `game/Field` and `game/FieldPoses` need the Onshape field CAD (§10) and a tape measure at the first event. |

### Not found / open
AprilTag field coordinates (will not exist); Limelight BIOBUZZ fmap; Pedro/Road Runner field assets;
SDK colour swatches for POLLEN/NECTAR; championship RP thresholds (TBA); the 1:00 audio cue (TBD);
Championship venue; printable tag PDF and dimensioned field drawing; expected pieces per match.
Follow-ups: read the Onshape CAD for the up-CELL height and FLOWER offsets; read ftc-docs on
2026-09-14 for clusters; ask in Q&A (2026-09-28) whether TIP-unlocked NECTAR can be banked and whether
a shot that bounces out of a CELL counts.

## 10. Sources
1. https://www.firstinspires.org/programs/ftc/game-and-season
2. https://ftc-resources.firstinspires.org/ftc/game (materials index); `/ftc/game/manual` (PDF); `/ftc/game/cm-html` (HTML, primary source); `/ftc/game/manual-09` … `manual-12`
3. https://ftc-resources.firstinspires.org/ftc/game/tu-00 (Team Update 00)
4. https://ftc-resources.firstinspires.org/ftc/field; `/ftc/field/eventfieldguide` (Event Field Setup Guide V1.0); `/ftc/field/initialfieldguide`
5. Onshape field CAD https://cad.onshape.com/documents/a355e772e3d24813de7852ee/w/f106353168f1f92100b81259/e/95d1e1e442b4138cccaf2d73 and FLOWER scoring volume `.../e/8f89b946e295be9c543faa08`
6. https://ftc-resources.firstinspires.org/ftc/archive/2027/event/season-dates (Key Season Dates V26-27.1)
7. https://www.firstinspires.org/hubfs/web/program/ftc/biobuzz-gamepostcard.pdf
8. https://github.com/FIRST-Tech-Challenge/FtcRobotController/releases/tag/v12.0 and the repo README (v12.0 notes); `.../samples/ConceptAprilTag.java`
9. https://ftc-docs.firstinspires.org/apriltag-clusters; ftc-docs AprilTag ID / metadata / field-coordinate-system pages
10. https://andymark.com/products/biobuzz-scoring-elements; https://www.gobilda.com/ftc-starter-bot-resource-guide-2026-2027-season/; https://andymark.com/products/robits-biobuzz-starterbot; REV and Studica StarterBot pages
11. Chief Delphi threads 524081 (auto-aiming), 524071 (defence), 524075 (STRATEGIC), 524050 (FUN Ri30H), 523993 (GT Ri30H), 519858 (predictions)
12. https://docs.limelightvision.io/docs/resources/downloads (no BIOBUZZ fmap); https://pedropathing.com/ (no BIOBUZZ assets)
13. Manual figure images: Fig 9-2 (zones), 9-5 (tile grid), 9-15 (cluster dimensions), 9-17 (tag IDs), 10-2 (element staging)
