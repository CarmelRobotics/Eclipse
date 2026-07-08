# Eclipse Robot Software — Complete Technical Report

**FRC Team 2035 — 2026 Season (*REBUILT*)**
Repository: `CarmelRobotics/Eclipse`, branch `summer` · Java 17, WPILib 2026 command-based, GradleRIO
Major dependencies: CTRE Phoenix 6 (26.1.0), PathPlanner (2026.1.2), DogLog (2026.5.0)
Last updated: 2026-07-06 (post field-calibration session of 2026-07-04; before second field session)

---

## Table of Contents

1. [Executive Summary](#1-executive-summary)
2. [Game Context: What REBUILT Demands of the Software](#2-game-context)
3. [Architecture, Conventions, and Project Layout](#3-architecture-conventions-and-project-layout)
4. [Drivetrain](#4-drivetrain)
5. [Localization (Vision-Fused Odometry)](#5-localization)
6. [Targeting and Shot Selection](#6-targeting-and-shot-selection)
7. [Shoot-on-the-Move Solver](#7-shoot-on-the-move-solver)
8. [Shooter](#8-shooter)
9. [Lintake (Intake)](#9-lintake-intake)
10. [Driver Assists and Operator Interface](#10-driver-assists-and-operator-interface)
11. [Autonomous](#11-autonomous)
12. [Diagnostics: Systems Check, Fault Monitor, Motor Test](#12-diagnostics)
13. [Telemetry and Tuning Infrastructure](#13-telemetry-and-tuning-infrastructure)
14. [Failure History and Lessons Learned](#14-failure-history-and-lessons-learned)
15. [Known Limitations and Roadmap](#15-known-limitations-and-roadmap)
- [Appendix A: Controller Map](#appendix-a-controller-map)
- [Appendix B: Dashboard Key Catalog](#appendix-b-dashboard-key-catalog)
- [Appendix C: Live-Tunable Parameter Catalog](#appendix-c-live-tunable-parameter-catalog)
- [Appendix D: CAN ID Map](#appendix-d-can-id-map)

---

## 1. Executive Summary

Eclipse is a swerve-drive robot for the 2026 FIRST Robotics Competition game *REBUILT*. Robots collect FUEL (5.91-inch foam balls) from the carpet and score it through the top opening of a Hub whose front lip sits 72 inches above the floor, while the two alliances' hubs alternate active/inactive scoring windows ("SHIFTS") through teleop. The robot's mechanical concept is a high-capacity hopper (~30 balls), a ground intake on rack-and-pinion arms, and a four-motor flywheel drum feeding through an adjustable-angle pivot — a volume shooter designed to dump its entire hopper in a few seconds.

The software is a WPILib command-based project organized around one consistent pattern: **commands set enum states; each subsystem's `periodic()` continuously drives its hardware toward the current state.** On top of that foundation sit:

- **Dual-mode vision localization** — hard pose snap when far off, Kalman fusion with distance-scaled trust when tracking, pure odometry when blind.
- **Field-calibrated ballistics** — interpolated shot tables re-anchored on real field data over a minimum-energy physics model.
- **Context-aware shooting** — a single driver trigger that latches into either a hub shot or a ferry pass based on range, alliance-zone position, and the live SHIFTS hub-active state.
- **Shoot-on-the-move** — a cached per-loop shot solution with filtered velocity lead and fixed-point time-of-flight iteration.
- **Driver assists** — geofenced heading/translation aids for the trench corridors and tower, an X-lock defense brake, and controller rumble that marks the exact firing window.
- **A diagnostics suite** — continuous CAN/camera fault monitoring, a read-only pit systems check, and an active motor self-test that verifies every motor mechanically responds.
- **Self-collecting calibration data** — every detected shot writes a snapshot of its full firing solution and the live trim offsets in effect.
- **Brownout defense in depth** — supply current caps on every motor group, open-loop voltage ramping, load shedding on the flywheel idle, and a lowered RIO brownout threshold.

Autonomous routines are built from team 2910's published PathPlanner paths (imported byte-for-byte after hand-adapted versions missed the fuel lines), with single- and double-cycle trench variants for both field sides. `FIELD_TESTING.md` in the repository root is the companion field-session script; this document is the design reference.

---

## 2. Game Context

Software decisions below repeatedly reference these game facts, so they are collected once here.

**The field** (blue-origin coordinates, from the official `2026-rebuilt-welded` AprilTag layout): 16.541 m long (X) by 8.069 m wide (Y). Blue's driver stations are at x = 0, red's at x = 16.541.

**The hubs**: each alliance has a hub embedded in a barrier line that crosses the field. Hub centers are at (4.625, 4.035) for blue and (11.925, 4.035) for red. The scoring opening's front lip is 72 in (1.829 m) above the carpet, and the opening is roughly 2.5 ft across — balls must *descend* into it, which drives the entire ballistics design toward steep, minimum-energy arcs rather than flat shots.

**The barrier and trenches**: each alliance's barrier line (guardrail–trench–bump–hub–bump–trench–guardrail) is 47 in deep in X; the hub-face AprilTags fix the spans at x = 4.022–5.229 (blue) and 11.312–12.519 (red). The outer 65.65 in (1.668 m) of each line, nearest each guardrail, is a drive-under trench corridor. Driving through a trench requires the shooter stowed (a height bar) and rewards a straight line — the genesis of the trench assist.

**The towers**: one per alliance wall between driver stations 2 and 3, 49.25 in wide, base reaching 39 in onto the field, centered at y = 3.962 (blue wall) / y = 4.108 (red wall).

**The alliance zone**: hub shots only score from inside the shooter's own alliance zone. The hub sits *on* the zone line, so the zone is everything between the alliance wall and the hub's X coordinate.

**SHIFTS**: teleop is 2:20 (a 140-second countdown). The first 10 seconds are a transition with both hubs active; then four 25-second shifts alternate which alliance's hub scores; the final 30 seconds (endgame) are both-active. The shift order is delivered in the game-specific message — a single character, `'R'` or `'B'`, naming the alliance whose hub goes inactive *first* — sent by FMS roughly 3 seconds after auto ends, based on which alliance scored more in auto. There is no climb or endgame mechanism in this game; endgame is simply open scoring.

**Match time plumbing**: `DriverStation.getMatchTime()` counts down the current period when connected to FMS or when the DS is in Practice mode, and returns −1 in ordinary standalone teleop. The SHIFTS logic exploits this: −1 falls into the always-active window, so shop driving is never gated.

---

## 3. Architecture, Conventions, and Project Layout

### 3.1 Layout

```
src/main/java/frc/robot/
├── Main.java                  — WPILib entry point (unmodified)
├── Robot.java                 — logging init, tuning-mode switch, brownout threshold, battery traces
├── Constants.java             — kMaxSpeed (= drivetrain rated speed at 12 V), kMaxAngularRate (1 rot/s)
├── RobotContainer.java        — subsystem construction, all bindings, assist logic, shot command
│                                factories, named commands, auto chooser, sim pose setter
├── util/
│   └── LoggedTunableNumber.java   — live-tunable dashboard values with a global lockout
└── subsystems/
    ├── drive/
    │   ├── TunerConstants.java            — CTRE-generated swerve constants + brownout hardening
    │   └── CommandSwerveDrivetrain.java   — drivetrain subsystem; vision, targeting, aim commands
    ├── shooter/
    │   ├── Shooter.java                   — flywheel/pivot/indexer control, readiness, detection
    │   └── ShooterConstants.java          — IDs, gains, tables, physics derivation, enums
    ├── lintake/
    │   ├── Lintake.java                   — pinion + roller control
    │   └── LintakeConstants.java          — IDs, gains, compliance current limits, enums
    ├── diagnostics/
    │   └── SystemsCheck.java              — fault monitor, pit check, active motor test
    └── localisation/
        ├── LimelightHelpers.java          — vendor helper (unmodified)
        ├── LimelightInfo.java             — camera name + mount pose record
        └── LocalisationConstants.java     — camera roster
src/main/deploy/pathplanner/               — autos and paths (2910-derived + legacy)
FIELD_TESTING.md                           — ordered field-session checklist
```

### 3.2 The state-machine convention

Every mechanism subsystem exposes small enums (`ShooterState`, `PivotState`, `IndexerState`, `PinionState`, `RollerState`). Commands — bindings, autos, assists — only ever *set states*. The subsystem's `periodic()` translates the current state into hardware control calls every 20 ms loop, unconditionally. This has three consequences that shaped the season:

1. **Commands are cheap and composable.** A shot sequence is "set states, wait for readiness, set the feed state" — no command owns a control loop.
2. **Cleanup is trivial and reliable.** `finallyDo(() -> stopShooter())` sets three states to safe values; `periodic()` does the rest. Every shot command in the codebase ends this way, so no interruption — driver release, auto timeout, e-stop — can leave a motor running.
3. **A returned-but-unscheduled `Command` does nothing, silently.** Twice this season a `Command`-returning helper was called for its side effect and the returned command discarded — an intake stow, then the vision pose seed. Both were invisible no-ops. The convention now: any operation that must happen when called (vision resets, assist state sets) has an *imperative* method; `Command`-returning wrappers exist only for binding use. `Lintake` exposes both forms explicitly (`setState(PinionState)` returning a Command for bindings, `setPinionState(PinionState)` imperative for lambdas).

### 3.3 Coordinate and unit conventions

- **All field math is blue-origin.** The vision pipeline always reads the `wpiBlue` botpose; hub positions, assist zones, pass corners, and PathPlanner paths are all blue-origin. Nothing flips the *pose* by alliance — code that needs alliance awareness (hub selection, zone gate, pass end) branches explicitly on `DriverStation.getAlliance()`. The only alliance flip in the system is the *operator perspective* (which way "forward" points on the driver's stick) and PathPlanner's own path mirroring.
- **Pivot positions are output rotations** (after the 42:1 reduction); conversion to motor rotations happens in exactly one place (`setPivotPosition`). Flywheel speeds are rotor RPS. Distances are meters; the shot tables are keyed in meters.

---

## 4. Drivetrain

### 4.1 Hardware and generated configuration

The drivetrain is a CTRE Tuner X-generated swerve project: four modules of TalonFX drive + TalonFX steer + CANcoder, a Pigeon 2 (ID 1), all on a dedicated CANivore bus named **`drivetrain`**. That bus name is load-bearing: the robot's first field session was lost to motors silently not responding because the constant was an empty string — the swerve library found no devices and threw no error.

| Module | Drive | Steer | Encoder | Position (X, Y) |
|---|---|---|---|---|
| Front-left | 3 | 4 | 33 | (+11 in, +11 in) |
| Front-right | 22 | 9 | 47 | (+11 in, −11 in) |
| Back-left | 55 | 61 | 62 | (−11 in, +11 in) |
| Back-right | 19 | 54 | 46 | (−11 in, −11 in) |

Key mechanical constants: 2.05 in wheel radius, 5.2734 drive ratio, 26.0909 steer ratio, 3.375 couple ratio, 120 A slip current (the stator level where wheels break traction), 6.00 m/s rated speed at 12 V (this value *is* `Constants.kMaxSpeed`). Steer feedback is `FusedCANcoder`; left side uninverted, right side inverted.

Closed-loop gains, both voltage-output: steer kP 33 / kD 0.5 / kS 0.1 / kV 2.49 (static feedforward using the closed-loop sign); drive kP 0.1 / kV 0.124. The steer gains were retuned during bring-up (kP 33 with a touch of kD holds module angle crisply without audible chatter).

### 4.2 Brownout defense (drivetrain's share)

The robot's power story is "four drive Krakens plus four flywheel Krakens plus everything else on one battery," and it produced field-visible sag. The defenses are layered across subsystems; the drivetrain's layer, applied in `TunerConstants`:

- **Drive supply current limit, 40 A per motor** (160 A drivetrain total). The slip current (120 A stator) bounds *traction torque* but not *battery draw* — four unlimited drive Krakens demand 300+ A on a hard launch, which alone sags a good battery toward the brownout threshold before the flywheel even spins up. The comment in code marks this as the one power cut with a real performance cost (sustained pushing force); the revert knob is documented inline: if the robot loses pushing battles, raise toward 50 A — launch acceleration barely changes because it is traction-limited anyway.
- **Open-loop voltage ramp, 0.15 s.** Teleop drive is `OpenLoopVoltage`, so a full stick slam is otherwise a 0→12 V step and the single biggest di/dt spike in normal driving. Ramping flattens the spike with no steady-state cost; auto path following is closed-loop velocity and untouched.
- **Steer limits: 60 A stator, 20 A supply.** Azimuth needs torque headroom but never sustained battery draw.

The other layers, described in their own sections: RIO brownout threshold lowered to 6.0 V (`Robot`), flywheel supply caps (35 A × 4), flywheel idle shedding below 7.0 V, mechanism supply caps (30 A), and — on top of all of them — the dynamic **PowerManager** (§4.5) that reallocates the drive/flywheel budget by activity instead of budgeting worst-case-everywhere.

### 4.5 Dynamic power management (PowerManager)

The static caps above must each assume the worst case — the drive limit assumes a full launch, the flywheel limit assumes a full volley — even though those never happen at the same instant. `PowerManager` (in `subsystems/power`, modeled on team 581's) exploits that: it switches between named `PowerMode`s that hand the battery to whatever the robot is doing right now.

| Mode | Drive A/motor | Flywheel A/motor | When |
|---|---|---|---|
| IDLE | 40 | 35 | Default — equals the static compiled limits |
| SCORING | 25 | 50 | Aiming/shooting: flywheel headroom to hold speed, drive (holding a heading) yields |
| SPRINT | 65 | 20 | Full-stick translation, no aim: drive pulls hard, flywheel only idling |
| AUTO | 55 | 45 | Autonomous collect-and-score cycles |

The desired mode is computed each loop in `RobotContainer.desiredPowerMode()` from controller and DS state; `PowerManager` (a self-scheduling `SubsystemBase`) applies the mode's supply limits **only on change**, and only after the mode has held ~100 ms (a stability filter so a stick hovering at the SPRINT threshold can't churn config writes). Because applying a current-limit config over CAN blocks ~1 ms per motor, the eight applies run on a **single background executor thread** rather than stalling the 20 ms loop. Each subsystem exposes the runtime setter (`applyDriveSupplyCurrentLimit`, `applyFlywheelSupplyCurrentLimit`) that preserves its other config (the drive stays supply-only; the flywheel keeps its 100 A stator headroom).

The whole system is gated by `FeatureFlags.POWER_MANAGER`, **default off**: when off the mode is pinned to IDLE, whose limits equal the compiled static defaults, so disabling the flag restores exactly the known-good static behavior. It is opt-in at the field precisely because it changes brownout behavior and must be validated against the battery traces before it is trusted in a match.

### 4.3 Swerve requests in use

All drivetrain control flows through Phoenix 6 `SwerveRequest` objects applied by `applyRequest(Supplier<SwerveRequest>)` (a `run()` command requiring the drivetrain):

| Request | Use |
|---|---|
| `FieldCentric` (10% deadbands, open-loop voltage) | Default teleop drive |
| `FieldCentricFacingAngle` (`snapRequest`, P 8 / D 0.2, continuous input) | Hub aim and pass aim: heading locked, driver translates |
| `FieldCentricFacingAngle` (`assistSnapRequest`, same PID) | Trench/tower assists |
| `SwerveDriveBrake` | X-lock defense brake |
| `Idle` | Applied while disabled (`ignoringDisable`) |
| `ApplyRobotSpeeds` | PathPlanner's output, with wheel-force feedforwards |
| `PointWheelsAt` / `RobotCentric` | Motor self-test steer and drive steps |
| SysId translation/steer/rotation | Characterization routines |

The two facing-angle requests use P = 8, D = 0.2. The D term exists because P-only at 8 visibly overshot and hunted around the hub bearing; 0.2 damps the approach. (The *original* oscillation had a deeper cause — §14.)

### 4.4 SysId and simulation

The drivetrain retains the three generated SysId routines (translation, steer, rotation — selected by editing `m_sysIdRoutineToApply`) with SignalLogger output. Simulation runs a 4 ms `Notifier` loop feeding `updateSimState` with battery voltage, so swerve control gains behave plausibly in desktop sim. The dashboard pose setter (§10.5) exists specifically to exploit this sim without external tools.

---

## 5. Localization

### 5.1 Pipeline

One Limelight, `limelight-four`, mounted 0.35 m forward, 0.15 m up, pitched 20° (per `LocalisationConstants`; the mount pose lives in the camera's web UI — code does not push it). Each `periodic()` loop, for each camera in the roster:

1. **Push orientation:** `SetRobotOrientation(name, currentHeadingDeg, yawRateDps, 0…)` — required by MegaTag2 if it is ever enabled, harmless for the MegaTag1 read used today.
2. **Read the blue-origin botpose:** `getBotPoseEstimate_wpiBlue(name)`. If the estimate is null or has zero tags, the camera contributes nothing this loop and odometry carries the pose.
3. **Branch on the delta** between the vision translation and the current pose estimate.

### 5.2 Snap vs. fuse

The branch threshold is `kMaxVisionCorrectionMeters` = 1.0 m:

- **Delta > 1.0 m — snap.** The pose is simply wrong: startup, the robot was carried, or odometry ran blind too long. The code calls `resetPose(vision XY, current gyro heading)` — translation from vision, rotation from the Pigeon, because single-tag vision heading jitters degrees while the Pigeon drifts ~0.1°/s. Fusing from a meter away would take seconds to converge; snapping is correct. A counter (`vision/pose resets`) increments on every snap — a steadily climbing counter while tags are visible is the diagnostic signature of localization fighting itself.
- **Delta ≤ 1.0 m — fuse.** Normal tracking. The estimate feeds `addVisionMeasurement` with translation standard deviation **σ = 0.05 + 0.02·d²** meters (d = average tag distance): 0.07 m at 1 m, 0.13 m at 2 m, 0.37 m at 4 m. Heading σ is `Double.MAX_VALUE` — vision never corrects rotation; the Pigeon owns it. Fusion (rather than the earlier per-frame hard reset with a deadband) does two jobs: it smooths single-tag botpose jitter so the aim target doesn't vibrate, and it absorbs the garbage frames cameras emit as a tag exits the field of view (edge distortion plus motion blur while rotating). One of those garbage frames once hard-reset the pose *wrong at the exact moment vision went blind* — precisely when odometry needed a good starting point to carry the aim.
- **No tags — carry.** Wheel odometry propagates the pose at cm/s drift, so aiming keeps working through dropouts (a hard requirement discovered on the field: the tag leaves the camera's FOV during close-range aiming).

**Heading field-alignment.** The Pigeon owns *fine* rotation (the fusion above is translation-only, infinite heading σ), but it must be *field-aligned* in the first place — otherwise every aim is off by the misalignment forever, with no other heading correction. So the loop seeds that alignment from **multi-tag** botpose heading (`tagCount ≥ 2`, which is unambiguous — a single tag's heading can flip): it adopts the vision heading once (`vision/heading seeded` → true), and re-adopts only if the gyro has since diverged by more than `kVisionHeadingSeedToleranceDegrees` (10°). Small disagreements are left to the Pigeon, so normal jitter/drift never re-aligns and the aim target doesn't chatter; a genuine divergence — an unseeded gyro, a bad auto seed, or a mid-match bump — snaps back on the next two-tag look. This closes the single-point-of-failure where a wrong gyro heading silently ruined every shot (§14, incident 8).

Timestamps pass through `Utils.fpgaToCurrentTime()` for correct latency compensation in the underlying pose estimator.

### 5.3 The imperative-reset rule

The single most expensive bug of the season: `setPose()` was a `Command`-returning wrapper, the vision code called it and discarded the result, and 12,481 consecutive vision measurements were "applied" as silent no-ops while the dashboard showed a frozen pose. The rule, now enforced by comment at the call site and by the absence of the wrapper: **pose changes go through `resetPose()` / `addVisionMeasurement()` imperatively — never through a Command-returning helper.**

### 5.4 Published diagnostics

Every loop: `vision/pose x`, `vision/pose y`, `vision/tag count`, `vision/avg tag dist`, `vision/pose resets`, `hub visible`, plus the Field2d widget. The intended reading: if `dist to hub` disagrees with a tape measure, every downstream number (hood angle, RPS, aim) is being computed for the wrong place — fix pose before tuning anything.

---

## 6. Targeting and Shot Selection

The drivetrain owns all targeting state (it owns the pose), exposed to bindings and the shooter as small query methods.

### 6.1 Which hub

`getHubPosition()`:

- **FMS attached** → the alliance hub, per `DriverStation.getAlliance()`.
- **No FMS (shop/practice)** → whichever hub is *closer* to the robot.

The shop rule exists because practice driver stations routinely sit on the wrong alliance. Before it, a blue-side practice robot with a red DS aimed at the red hub ~9 m away: the robot appeared to "aim forward" from everywhere in the practice half, and the shot tables clamped at max range (max hood, max RPS) — two confusing symptoms with one root cause.

### 6.2 The alliance-zone gate

`isInAllianceZone()` implements the legality constraint positionally: the hub sits on the zone line, so the zone is the wall side of the hub's X. Red: `robotX ≥ 11.925`; blue: `robotX ≤ 4.625`. Which rule applies comes from the **DS alliance** — FMS-assigned in matches, manually selected in the shop (documented in the field checklist: set the DS alliance to the side you practice on, or the right trigger will always ferry).

The gate matters because the geometry is deceptive: the hub is ~4.6 m from the wall and the shot range is 5 m, so the "in range" circle spills past the barrier — without the gate, a robot past the line could take (illegal, non-scoring) hub shots. The first implementation compared against the field midline; it was corrected in review because the pose does *not* flip by alliance, and a red robot at its own hub (x ≈ 11.9) failed an `x < 8.27` test — the check would have blocked every red hub shot. The current form is symmetric by construction.

### 6.3 SHIFTS integration

`computeHubActiveForAlliance(alliance, matchTime, gameData)` is a faithful copy of the WPILib 2026 reference implementation:

| Countdown (140 s teleop) | State |
|---|---|
| > 130 | Transition — both hubs active (10 s) |
| 130–105 | Shift 1 |
| 105–80 | Shift 2 |
| 80–55 | Shift 3 |
| 55–30 | Shift 4 |
| ≤ 30 | Endgame — both hubs active (30 s) |

Game data `'R'` means red's hub is inactive in shifts 1 and 3 (blue's in 2 and 4); `'B'` the reverse. Autonomous returns active; disabled returns inactive; empty game data returns active (the WPILib-recommended safe default — data arrives ~3 s into teleop).

Two local additions:

- **`effectiveGameData()`** — when the real game data is empty *and FMS is not attached*, synthesize `'B'`. DS **Practice mode** supplies a real countdown, so with synthesized data the shifts actually cycle on schedule and drivers can rehearse the active/inactive transitions (set practice teleop to 140 s; type `R`/`B` in the DS Game Data box to choose the order). With FMS attached and data not yet arrived, the empty string passes through and the safe assume-active default holds — never guess the shift order on a real field. Plain teleop reads matchTime = −1, which lands in the always-active window, so ordinary shop driving is unaffected.
- **`isOurHubActive()`** — the per-loop cached result (computed early in `periodic()` so the dashboard mirror can't disagree with the gate), published as `our hub active`, plus an `active hub alliance` string for the dashboard.

### 6.4 The context-aware trigger

The right trigger binds to `Commands.either(hubShotGroup, passGroup, condition)` where the condition is:

```
getShotDistance() ≤ 5 m  AND  isInAllianceZone()  AND  isOurHubActive()
```

If all three hold at the moment of the pull, the driver gets the full hub shot (aim lock + spin-up + feed + rumble on `readyToShoot`). Otherwise the same pull is a smart pass (corner aim + ferry spin-up + feed + rumble on `readyToPass`). **The decision latches at the pull** — `either` evaluates once at initialization — so the mode cannot flip mid-hold at a range boundary or a shift transition; release and re-pull to re-decide. The dashboard shows `aim/auto mode` (the exact live condition, same expression) and `aim/in alliance zone` so the driver always knows what a pull would do.

Rationale for the SHIFTS term: during the opponent's shift a hub shot scores nothing — the balls are donated to the floor at best. Routing the same trigger to a ferry keeps the driver's muscle memory intact ("trigger = do the useful scoring thing") while the strategy shifts underneath.

### 6.5 Pass targeting

`getPassTarget()` returns the alliance-zone corner on the robot's current side of the field: X is the own-alliance end (from the DS alliance) inset by `ShotTuning/PassCornerInsetM`; Y is the near-side wall inset by the same amount. The side selection has **0.5 m of hysteresis** around the field centerline (`m_passHighSide` updates only past ±0.5 m) so the target — and the heading lock chasing it — can't flicker between corners while driving down the middle. Pass aim is a straight bearing (no motion lead: a pass lands in a multi-meter zone, not a 0.6 m opening) plus the same mechanical heading trim as hub aim. `getPassDistance()`, `getPassHeading()`, and `getPassHeadingError()` feed the pass readiness gates.

The inset default is 1.0 m — deliberately deep. The drum imparts heavy backspin; on landing, backspun balls check up and bounce *back toward the field*. Aiming near the corner means the bounce-back settles the ball inside the zone; even a wall hit drops dead at the base. The tuning instruction, preserved in the constant's comment: tune by watching where balls *settle*, not where they first land.

---

## 7. Shoot-on-the-Move Solver

### 7.1 The solution

`updateShotSolution()` runs **once per loop** in drivetrain `periodic()` (after vision, so it uses the freshest pose) and caches two values every consumer reads: `m_shotVector` (the displacement the ball must cover, in field frame) and `m_shotTofSeconds`.

```
robotToHub = hubPosition − robotTranslation
lead       = lowpass(fieldVelocity) × MoveCompGain          // gain default 0.7
tof        = TOF_table(‖robotToHub‖) + TimeOfFlightOffset
repeat 3×:
    shotVector = robotToHub − lead × tof
    tof        = TOF_table(‖shotVector‖) + TimeOfFlightOffset
```

- The **low-pass filter** is a single-pole IIR with a 0.15 s time constant on each velocity axis (20 ms sample time). Raw swerve field velocity is noisy; unfiltered, every module wiggle went straight into the aim target that the P = 8 heading controller was chasing.
- **`MoveCompGain` = 0.7** deliberately under-leads. Full physics lead (1.0) maximizes on-the-move accuracy in principle, but 0.7 makes the target move far less under the driver at a small accuracy cost at max speed. 0 disables lead entirely.
- The **fixed-point iteration** exists because lead depends on TOF and TOF depends on the *led* distance. It converges in 2–3 passes at any legal robot speed. The pre-fix code looked TOF up at the raw hub distance while the tables were consulted at the led distance — the lead itself was computed with the wrong flight time.

`getShotDistance()` is the cached vector's norm clamped to the table domain (0–6 m); `getHubHeading()` is its bearing plus `ShotTuning/HeadingOffsetDeg` (a mechanical-bias trim: if shots land consistently left/right, trim in 1° steps and later bake into the constant).

Historically the solution was re-derived five to six times per loop by different callers, straight from raw module speeds — that inconsistency (each caller seeing a slightly different "solution") plus the noise was the source of the on-the-move twitchiness the cache eliminated.

### 7.2 The removed rotation-compensation term — do not re-add

`getHubHeading()` used to add an "aim ahead while spinning" term: gyro yaw rate × TOF, clamped. It was removed deliberately, and the code comment preserves the two independent reasons:

1. **It closed a positive feedback loop.** The heading controller rotates the robot toward the target; the rotation feeds the gyro-rate term; the term pushes the target further ahead; the error grows; the controller rotates faster. Loop gain ≈ P × TOF ≈ 8 × 0.8 ≈ 6 — far past unity. The robot orbited the hub bearing and could never settle. This presented on the field as "it oscillates when aiming," which looked exactly like a PID tuning problem and wasn't.
2. **Its units were wrong anyway.** `getAngularVelocityZWorld()` returns degrees/second; the term consumed it as radians/second, so any rotation at all pinned the compensation at its ±20° clamp.

The correct observation that killed it for good: `faceHubCommand` drives yaw rate to zero *before* the shot fires (that's what the aim gate is), so a spin term contributes nothing to a settled shot. If spin compensation is ever genuinely needed (e.g., shooting during a deliberate spin), it must be applied once at ball release — never inside the target the controller is chasing.

### 7.3 Heading control

Both aim requests share the tuned heading controller: **P = 8, I = 0, D = 0.2**, continuous input on (−π, π]. P = 8 settles a 90° error in well under a second; D = 0.2 removes the terminal overshoot/hunt. The readiness gate (±5°) is deliberately wider than the controller's steady-state accuracy — it exists to catch *transients*, not to demand precision the mechanism can't use (a 5° error at 5 m moves the impact ~0.44 m; the gate plus the hub opening tolerate it).

While aiming, driver translation is scaled by `ShotTuning/ShootingSpeedScale` (default 0.45): less speed means a smaller velocity lead and less stick noise in the solution. Full stick deflection still works — it's a cap, not a curve change.

---

## 8. Shooter

### 8.1 Hardware and configuration

| Component | Motors (CAN) | Control | Gains / limits |
|---|---|---|---|
| Flywheel drum, 4 in wheels, 1:1 | 7 (left leader), 8 (back-left), 11 (right), 49 (back-right) | `VelocityVoltage`, all four commanded identically | kP 0.3, kI 0, kD 0, kS 0.15, kV 0.125, kA 0.2 · supply 35 A, stator 100 A |
| Pivot, 42:1 | 5 (leader, CW+), 6 (follower, CCW+) | `MotionMagicVoltage` position | kP 4.8, kD 0.1, kS 0.25, kV 0.12, kA 0.01 · MM cruise 80 / accel 160 / jerk 1600 (motor rot) · supply 30 A |
| Indexer/kicker | 53 (Kraken X44) | Direct voltage | ±4.5 V · supply 30 A |

Left and right flywheel pairs carry opposite inverts in their configs, so a single identical velocity command spins both sides of the drum correctly — there is no Phoenix `Follower` setup, by design, so the SysId routine and the self-test can observe each motor independently.

**Flywheel gain rationale** (from the constants file): kS was originally 0, which is physically wrong — every motor needs a static term; 0.15 is a conservative estimate pending SysId. kP was raised 0.125 → 0.3 specifically to fight the ball-passage velocity dip harder; the tuning instruction is to push it toward the edge of oscillation while watching the `Shooter/RpsError` log trace. kD is deliberately 0: on a velocity loop it differentiates an already-noisy signal.

**Flywheel supply cap rationale**: 35 A × 4 = 140 A of battery draw ceiling (down from 200 A at the original 50 A each). The supply cap is what most directly helps `VelocityVoltage` *hold speed*, because it prevents battery sag from eating the voltage headroom the controller is commanding with. Spin-up gets marginally slower; the shot timeouts already cover it. The stator limit stays generous (100 A) so recovery torque mid-volley is never the bottleneck.

**Status-signal management**: the constructor explicitly sets update frequencies for every signal the code reads — leader velocity 100 Hz, all supply currents 50 Hz, leader pivot position 100 Hz, and (for the motor self-test) the three follower flywheel velocities, follower pivot position, and indexer velocity at 50 Hz — *before* calling `optimizeBusUtilization()` on all seven motors. The ordering is load-bearing: optimization disables every signal not given a frequency, and an earlier iteration froze telemetry (readiness, overcurrent, shot detection all read startup values) by optimizing first.

### 8.2 State machines

```
ShooterState: ZERO (smart idle) · SCORE (table RPS) · PASS (ferry table RPS)
              · LOB (40 RPS) · SEND (90 RPS) · REVERSE (−10 RPS)
PivotState:   STOW (0) · SCORE (table angle) · LOB (0.0694 rot ≈ 25°)
              · PASS (tunable, default 0.086) · SHOT_BLOCK (tunable, default 0.5 rot)
IndexerState: ZERO (0 V) · SCORE (−4.5 V, feed) · REVERSE (+4.5 V, back out)
```

`periodic()` translates these every loop: a `switch` drives the pivot to its state's position, a `switch` selects the commanded flywheel RPS (suppressed while a SysId routine holds the motors), and the indexer gets its state's voltage directly.

**Smart idle** (`ZERO` state): the drum pre-spins at 6.7 RPS so shots start from rolling, not static, friction — but only when both (a) within 6.5 m of the nearest hub (beyond that the 4-motor draw is pure waste; 6.5 m leaves room to be fully spun up before entering the 5 m shooting range) and (b) battery above 7.0 V. The battery clause is a brownout shed: when the bus is already sagging, the pre-spin is the nice-to-have that gets dropped to protect the drivetrain — a stalled drive is a lost match; a slower spin-up is not. The check is deliberately coarse (instantaneous voltage, no debounce): the drum barely changes speed in the fraction of a second the bus needs to recover.

### 8.3 Shot physics and the calibrated tables

Four `InterpolatingDoubleTreeMap`s key everything on distance. The *shape* of the curves comes from physics; the *values* are field-calibrated.

**The physics model.** The hub demands descent into a top opening, so the tables target the minimum-energy (slowest-ball) trajectory — slower balls drop more steeply. For launch height ~12 in (Δh = 1.524 m to the lip):

- Optimal launch angle: **θ = 45° + ½·atan(Δh/d)** — steeper close in, approaching 45° at range.
- Required speed: **v² = g·(Δh + √(d² + Δh²))**.
- Ball speed → wheel RPS: 4 in drum (0.638 m/rot circumference) at an assumed 75% surface-to-ball transfer: **RPS ≈ 4.177 · v**.
- TOF from the trajectory: t = d / (v·cos θ).

**The hood model** (empirically established during calibration): at dead stow the shooter launches ≈ 70° from horizontal, and one degree of pivot travel is one degree flatter — so *higher pivot rotations = flatter launch* (LOB at 25° pivot ≈ 45° launch fits the same line).

**Calibration history, which explains every number in the tables:**

1. The original tables were the pure physics values (pivot 0.008–0.045 rot, RPS 25–33), documented as a deliberate baseline expected to trim up 10–25% on the field.
2. During the 2026-07-04 session, the hub-selection bug (§6.1) clamped every shot at the max-range table entry — hood 0.047 rot, 37.3 RPS — and *that combination demonstrably landed from ~2.75 m*. One accidental, perfectly repeatable data point.
3. Re-anchor: the working shot sat 0.018 rot flatter than the physics curve, so the entire pivot curve shifted +0.018 (shape preserved). The RPS curve was recomputed for the flatter arcs with a **×1.30 real-world factor** (drag + transfer losses) calibrated from the same point: vacuum physics said 28.7 RPS where the field needed 37.3. Sanity check: the re-anchored table independently reproduces the anchor (predicts 37.0 at 2.5 m).
4. Field-bake: shots then landed at all tested distances with dashboard trims of +0.02 rot pivot and +4.5 RPS, so both were baked straight into the entries (and the dashboard offsets zeroed — the comment warns that leaving trims non-zero after a bake double-counts them).

**The resulting tables:**

| Distance | Pivot (output rot) | RPS | TOF (s) |
|---|---|---|---|
| 1.5 m | 0.046 | 39.0 | 0.48 |
| 2.5 m | 0.065 | 41.5 | 0.62 |
| 3.5 m | 0.076 | 45.5 | 0.73 |
| 4.5 m | 0.083 | 49.0 | 0.83 |
| 5.0 m | 0.086 | 51.0 | 0.87 |

The RPS entries at 3.5/4.5/5.0 m carry a progressive drag allowance (+0.5/+1/+1.5) on top of the ×1.30 factor, because the factor was calibrated at midrange and drag grows with range — the field observation was that long shots fall short by a *scaling* amount, not a flat offset. Entries past ~3 m sit above the 25° LOB angle; that is fine for SCORE shots (25° is a kicker feed limit heuristic, not a travel limit, and these fed fine — §8.4). The 5.0 m row exists so the table covers the range gate exactly. The 4.5–5.0 m rows remain model-extrapolated pending the next session.

**Pass table** (ferry distance → RPS, at the pass pivot angle): 3 m → 28, 5 m → 37, 7 m → 44, 9 m → 50, 11 m → 56, 13 m → 61. Same calibrated model (×1.30), ground-to-ground ballistics from a 0.305 m release. Untested on the field; trim globally with `ShotTuning/PassRpsOffset`, then re-bake.

### 8.4 The flatter pass pivot (backspin mitigation)

The drum has a single contact surface — no opposing wheels — so backspin and exit speed are mechanically coupled; code cannot reduce spin without reducing speed one-for-one. What code *can* change is the landing geometry. A backspun ball checks up and bounces backward when its surface spin speed exceeds its forward ground speed at impact; a **flatter, faster arrival** keeps forward speed above the spin bite, so the ball rolls out instead of kicking back toward the field.

Passes therefore launch at their own `PivotState.PASS` angle, `ShotTuning/PassPivotRot`, default **0.086 rot** — meaningfully flatter than LOB's 0.0694 (higher = flatter) and equal to the flattest SCORE entry, which is confirmed to feed. Expected side effects, documented for the tuner: pass RPS must come *up* at the flatter angle (less arc = less carry at the same speed — the pass table was modeled at ~45° launch), and if balls start rolling *through* the corner zone, the angle has gone too flat. The manual POV-right ferry deliberately keeps the old LOB angle as a fallback.

### 8.5 Shot lifecycle commands

All shot commands share one factory shape (in `RobotContainer`):

```
deadline(
    sequence(
        prepareShotCommand(pivotState, shooterState)      // runOnce: set both states
        waitUntil(ready).withTimeout(spinupTimeout)        // 1.25 s shots / 2.0 s passes
        run(feed)  [.withTimeout(feedSeconds) in auto]     // indexer → SCORE
    ),
    lintakePumpCommand()                                   // agitation, dies with the shot
).finallyDo(stopShooter)                                   // states → ZERO/ZERO/STOW
```

- **`timedShotCommand(pivot, shooter, feedSeconds)`** — autonomous: feed for a fixed window. `"shoot"` = 1.5 s (a quick ~8-ball volley), `"dump"` = 2.5 s (empties preload + a full collect cycle).
- **`heldShotCommand(pivot, shooter)`** — teleop: feed until the driver releases.
- **`heldPassCommand()`** — teleop pass: gated on `readyToPass()` with the longer 2.0 s timeout (ferry speeds up to ~60 RPS spin up slower from idle).

**The force-feed timeout** is a deliberate design decision, not a compromise: if readiness hasn't cleared in 1.25 s, something is wrong at the hardware level (the normal critical path — spin-up ~0.5 s, pivot ~0.3 s, heading ~0.5 s — fits inside it with slack), and feeding anyway is strictly better than a robot frozen mid-match with its flywheel screaming. The timeout constant's comment walks the arithmetic.

**The agitation pump** (`lintakePumpCommand`) bounces the intake pinion between AGITATE (−7.5 rot) and GROUND (−9.75 rot) every 0.3 s for the duration of any shot. Thirty balls in a hopper pack together fast once the bottom layer drains; the pump keeps the pile flowing into the indexer instead of bridging over it. It is deliberately requirement-free (it only writes the pinion state), so it composes with the roller command and the shot command without cancelling either, and it's the reason the manual X→AGITATE binding could be retired.

### 8.6 Readiness gates

`readyToShoot()` is the AND of four independently published gates:

| Gate | Tolerance | What it protects against |
|---|---|---|
| In range | ≤ 5.0 m | Angle error dominating at range; ~0.44 m miss per 5° at 5 m |
| Aimed | ≤ 5.0° of `getHubHeading()` | Firing mid-rotation |
| Flywheel at speed | ± 3.0 RPS | Inconsistent exit velocity (spin-up ripple ±5–10 RPS, settled ±2) |
| Pivot in position | ± 0.04 output rot (~1.7 motor rot) | Firing mid-move; tolerance sized to sensor noise + 42:1 gear play |

`readyToPass()` swaps in the pass targets: flywheel at the ferry RPS (±3), pivot at the tunable pass angle (±0.04), heading within **10°** of the corner bearing (twice the hub tolerance — a pass lands in a ~2 m zone, not a 0.6 m opening). This is what lets passes fire on actual spin-up instead of the blind timeout.

Every gate is published individually (`ready/…`) and logged, so a stuck shot is diagnosable from the dashboard in seconds: whichever flag is false is the reason. While a shot or pass trigger is held, `rumbleWhenReady` drives steady controller rumble whenever the composite readiness is true — the driver *feels* the firing window without looking at anything. Steady rather than pulsed is intentional: it marks the entire valid window, not just its edge, and `finallyDo` clears it so rumble can never stick on.

### 8.7 Sensor-free shot detection and the per-shot log

There is no beam-break. Instead, ball passage is detected from the flywheel velocity signature — a mechanical fact no sensor wiring can fake:

```
DISARMED → (actual within 2 RPS of commanded) → ARMED
ARMED    → (actual drops > 5 RPS below commanded) → IN_DIP
IN_DIP   → (actual recovers to within 2 RPS)      → shot counted, back to ARMED
```

Noise ripple is ±1 RPS; a >5 RPS dip is real friction (a ball, or a jam — a jam never recovers, so it can't inflate the count). Both thresholds are live-tunable. The counter resets to disarmed whenever the wheel isn't spinning for a shot state.

**Every counted shot writes a snapshot** — the calibration record that removes clipboard-keeping from field sessions:

- `Shooter/ShotEvent/{Number, Mode, DistanceM, CommandedRps, ActualRps, PivotRot, HeadingErrorDeg, RpsOffset, PivotOffset, PassRpsOffset, BatteryVolts}` in the WPILOG (step through `Number` in AdvantageScope to replay a session shot by shot). `DistanceM` is the ferry distance for PASS shots and the hub shot distance otherwise.
- A `last shot` dashboard string, e.g. `#12 SCORE 2.48 m | rps 41.5 | pivot 0.065 | hdg 1.2 deg | trim rps +2.0 piv +0.005` — glance after each ball lands, note the landing next to it.

Logging the **live trim offsets** is the point: a shot that landed perfectly with `RpsOffset = +2` is direct evidence the table is 2 RPS low at that distance. Without the trim in the record, the post-session bake-in is reconstruction from memory.

### 8.8 Protection

Two software overcurrent supervisors run every loop, both with live-tunable thresholds, both implementing the same pattern (current above limit continuously for longer than a window → zero the motor, fall back to a safe state, raise a dashboard flag; any dip below the limit resets the timer):

- **Pivot: 40 A for 0.25 s**, watching `max(leader, follower)` supply current. A jammed MotionMagic pivot integrates into stall (~180 A, ~180 W of heat, thermal shutdown in about a second — which then locks the robot out of shooting for minutes). Normal MotionMagic accelerations peak 15–25 A and never hold 40 A for a quarter second. Using the max of the pair is deliberately conservative: if one motor jams while the other keeps pulling, the pivot racks and something bends.
- **Indexer: 25 A for 0.2 s.** Normal feeding is 5–10 A; stall is ~80 A. Not safety-critical (falls back to ZERO), but it stops a jam from cooking the motor and draining the battery while the driver keeps holding feed.

These *software* trips sit under *hardware* supply caps (30 A on pivot and indexer configs), so the soft trips keep their diagnostic role (a tripped flag names the mechanism) while the hard caps bound what the battery can ever see.

**Jam clearance** (`clearJamCommand`, held on B): flywheel to REVERSE (−10 RPS, slow backout) and indexer to REVERSE (+4.5 V) simultaneously — reversing unwinds the friction holding a stuck ball against the drum. Release restores ZERO/ZERO through the standard `runEnd`.

### 8.9 On-robot characterization

Four dashboard buttons run WPILib SysId voltage profiles (quasistatic and dynamic, each direction) across all four flywheel motors simultaneously — characterizing the drum as the single system it mechanically is. Each routine starts the CTRE SignalLogger, sets `m_characterizing` (which suspends `periodic()`'s velocity commands so the ramp isn't overwritten every loop), and `finallyDo` stops the motors, stops the logger, and returns control. Analysis happens in Tuner X against the `.hoot` (flywheel mechanism, rotations units to match Phoenix), producing kS/kV/kA to replace the hand-estimated values.

---

## 9. Lintake (Intake)

### 9.1 Hardware

| Component | Motor (CAN) | Control |
|---|---|---|
| Pinion arms (leader) | 35, CW+ | MotionMagic position, brake mode |
| Pinion arms (follower) | 2, CCW+ | MotionMagic position, brake mode |
| Roller | 45 | Direct voltage, coast mode |

Pinion gains kP 1.25 / kD 0.15 / kV 0.25 / kA 0.01 (kS 0), MotionMagic cruise 55 / accel 135 / jerk 1600. Positions: STOW −4, AGITATE −7.5, GROUND −9.75 rotations (zeroed at boot). Roller: INTAKE +12 V, EJECT −12 V, ZERO 0 V.

### 9.2 Compliance by current limit

The intake regularly hits things — walls, the trench bar, fuel piles. Rather than a soft gain schedule, compliance is a **stator current limit of 15 A** on the pinions: stator current is proportional to torque, so the cap bounds how hard the arms can ever push, in every state, with one number. The mechanics, worked in the constant's comment: 3:1 belt into a 10-tooth 10DP pinion (1 in pitch diameter) on the rack, Kraken torque ≈ 0.0194 N·m/A → 15 A × 2 motors × 3 ÷ 0.5 in radius ≈ **31 lbf** of hold/resist force. Push the deployed intake into a wall harder than that and it back-drives inward instead of stalling; it re-extends when the wall is gone. Deploy/retract needs only ~5–15 lbf, so motion stays crisp. The knob's direction is documented: raise for firmer hold, lower to yield on lighter contact. Supply is capped at 30 A per motor (brownout budget); the roller carries the same 30 A supply cap and no stator cap (no compliance role).

`periodic()` drives both pinions to the state position and the roller to the state voltage every loop, and publishes state/target/actual for both pinions (the follower encoder is read directly — this subsystem never calls `optimizeBusUtilization`, so all its signals stay live by default).

### 9.3 Interactions

Three other features drive the Lintake: the trench/tower assist deploys it on structure entry and restows on exit (§10.2); every shot command runs the agitation pump against it (§8.5); and the motor self-test exercises roller and both pinions (§12.3). All of them use the imperative `setPinionState()`; the Command-returning `setState(PinionState)` overload exists for the bumper bindings and named commands. This dual API is a direct scar from the season's recurring returned-command-discarded bug.

---

## 10. Driver Assists and Operator Interface

### 10.1 Assist zone geometry

Zones are evaluated every loop from the pose (blue-origin meters), published as `assist/zone`:

- **Trench corridors**: X within either barrier span (4.02–5.23 or 11.31–12.52) AND Y within 1.668 m of either side wall (y ≥ 6.40 or y ≤ 1.668).
- **Towers**: within 0.99 m of either alliance wall in X, and within ±0.63 m of that wall's tower center Y (blue 3.962, red 4.108).
- **NEAR halos**: the same shapes grown by 0.8 m on every side.

Priority: INSIDE beats NEAR; trench beats tower (they cannot physically overlap).

### 10.2 Assist behavior

The assist is a drivetrain command scheduled by a trigger (`assistActive`) that is true when the zone is not NONE **and** no aim button is held **and** X (the brake) is not held:

- **NEAR (approach)** — the assist is a *suggestion*: heading snaps to the nearest 90° increment only while the rotation stick is centered (|right X| ≤ 0.1); any rotation input instantly returns full manual control and clears the lock so it re-snaps fresh on release. Translation is always fully manual.
- **INSIDE** — full assist: heading locks to the nearest 90° (0/90/180/270 — 90s so a robot can line up a shot straight out of a trench), and translation is railed to the facing axis (facing along X zeroes Y velocity and vice versa), so the robot glides through the corridor without drifting into the walls.
- **Snap hysteresis** — once locked, the heading only re-snaps to a different increment after the robot has rotated more than 60° from the current lock. The threshold must exceed 45° or the lock could flicker at the exact boundary between increments.
- **Mechanisms** — on entering INSIDE, the shooter pivot stows (clearing the trench bar) and the intake deploys (driving through a trench sweeps fuel); on exit the intake restows. The shooter stays stowed until the driver next aims — re-deploying it automatically under a bar it just cleared would be worse.

Two trigger-composition details are deliberate and subtle:

- **Aiming outranks the assist** (`aimHeld.negate()` in the trigger) so a driver can pop out of the trench and immediately shoot or ferry without the assist fighting the aim heading.
- **The X-lock brake is folded into the trigger** (`x().negate()`) rather than being allowed to interrupt the assist command. WPILib's `whileTrue` schedules only on a rising edge; a brake tap that merely *interrupted* the assist would leave it dead until the robot exited and re-entered the zone. With X in the trigger expression, releasing the brake *is* a rising edge, and the assist resumes immediately.

### 10.3 Full control map

| Input | Action |
|---|---|
| Left stick | Field-centric translation (6 m/s max, 10% deadband) |
| Right stick X | Rotation (1 rot/s max, 10% deadband) |
| **Right trigger (hold)** | Context shot, latched at pull: HUB (in range + in zone + hub active) else PASS. Translation scaled to 45% while aiming. Rumble = locked. |
| A (hold) | Aim at hub only (heading lock, no spin-up), rumble on readiness |
| POV-up (hold) | Forced smart pass (corner aim + ferry spin-up + feed on readiness) |
| POV-right (hold) | Manual flat ferry: LOB pivot, fixed 90 RPS, no aim assist (fallback) |
| Left trigger (hold) | Roller intake (auto-stops on release) |
| POV-left (hold) | Roller eject |
| LB / RB | Intake deploy / stow |
| **X (hold)** | X-lock defense brake (wheels crossed; release to drive) |
| Y (hold) | Pivot to SHOT_BLOCK (blocker clearance, tunable 0.5 rot) + intake stow; both restore on release |
| B (hold) | Clear jam: flywheel + indexer reverse |
| POV-down | Re-seed field-centric heading |

Automatic behaviors, no button: trench/tower assists, shot agitation pump, smart flywheel idle, disabled-mode drivetrain idle, operator-perspective flip on alliance change.

### 10.4 Velocity plumbing

`driverXVelocity`/`driverYVelocity` map the left stick (negated — stick up is field +X) through `kMaxSpeed`; `driverRotationalRate` maps right-stick X through `kMaxAngularRate`. The aim variants (`shootingXVelocity`/`shootingYVelocity`) multiply by the live `ShotTuning/ShootingSpeedScale` (0.45): capped translation shrinks the shoot-on-the-move lead and the stick noise entering the solution. POV-up's pass aim deliberately uses the *unscaled* driver velocities — ferrying benefits from mobility more than precision.

### 10.5 Simulation pose setter

Three dashboard numbers (`Sim/PoseX` default 4.6, `Sim/PoseY` default 0.75, `Sim/PoseHeadingDeg` default 0) plus a `Sim/ApplyPose` button that calls `resetPose` imperatively. Purpose: test any position-dependent behavior (assist zones, zone gate, range gate, pass side selection) in desktop simulation from the dashboard alone, without the separate sim GUI. On a real robot with a camera in view, vision will immediately correct whatever this sets — it is a sim tool by construction.

---

## 11. Autonomous

### 11.1 Infrastructure

PathPlanner `AutoBuilder`, configured in the drivetrain from the GUI settings file (`RobotConfig.fromGUISettings()`): holonomic controller with translation PID 10/0/0 and rotation PID 7/0/0, `ChassisSpeeds.discretize` at 20 ms, wheel-force feedforwards passed through to the `ApplyRobotSpeeds` request, and automatic path mirroring when the DS alliance is red. A `PathfindingCommand.warmupCommand()` is scheduled at boot so the first pathfinding call doesn't pay JIT/allocation cost mid-match. The auto chooser (`AutoBuilder.buildAutoChooser()`) publishes to `Auto` on the dashboard.

Named commands available to every auto: `intake deploy`, `intake retract`, `intake run`, `intake stop`, `shoot` (timed shot, 1.5 s feed), `dump` (timed shot, 2.5 s feed), `stopshoot`, `zerodrive` (no-op placeholder). The roller commands are `runOnce` lambdas because the roller setter is imperative-only.

### 11.2 Routines

The competitive autos are built from **team 2910's published BLUE paths, imported byte-for-byte**. The first attempt hand-adapted their routes from screenshots; in the PathPlanner simulator the collect legs raked empty carpet, and the fix — after being told twice — was to stop adapting and copy exactly. The score-pose waypoints were subsequently shortened in-house (score at (2.718, 5.504) rather than deep in the corner) to cut cycle time.

- **`2910_trench_left`** — intake deploy + run → preload start (≈ 4.4, 7.3) through the far mid-field fuel line (≈ 8.3, 4.0) → near mid-field (≈ 5.95, 4.0) → trench return (≈ 3.0, 7.3) → score pose → `dump` raced against a 4.0 s wait → intake stop.
- **`2910_trench_right`** — the same routine mirrored to the low-Y trench (PathPlanner flips the BLUE paths for red automatically; the LEFT/RIGHT pair covers both starting sides for blue).
- **`2910_trench_left_double` / `2910_trench_right_double`** — the single cycle, then a second collect cycle through the alternate fuel row (LEFT variant sweeps the y ≈ 5.6 line, RIGHT the y ≈ 2.47 line) and a second dump. Whether both cycles fit the auto period is a stopwatch question for the next field session.

Legacy bring-up routines (`preload`, `testauto`, `doubleswipe`) remain selectable. All autos set `resetOdom: true` — odometry resets to the path's start pose at auto init, and vision then corrects; physical placement accuracy determines how much of the first second is spent converging.

---

## 12. Diagnostics

`SystemsCheck` exists because the season's two worst debugging sessions were failures that produced **no error anywhere** — a wrong CAN bus name (every motor silently absent) and a discarded Command (vision silently never applied). The subsystem provides three escalating layers.

### 12.1 Continuous fault monitor

Every loop, `SystemsCheck.periodic()` publishes a boolean per device under `Faults/…` for all 23 CTRE devices — 12 swerve motors/encoders + Pigeon (harvested from the drivetrain's module accessors) and the shooter's 7 + lintake's 3 (each subsystem contributes its own named device map, so naming stays with the owner) — plus a liveness flag per Limelight, an aggregate `Faults/AnyFault`, a count, and an `Faults/Offline` list naming the missing devices. Camera liveness watches the Limelight `hb` heartbeat: if it hasn't advanced in 1.0 s, the camera is unplugged, unpowered, or crashed. Heartbeat timestamps initialize to zero so cameras read *down* until their first real frame — a momentary false-down at boot is preferred to a false-up.

Deliberately **no rumble** on faults: rumble is reserved exclusively for the shot-ready cue, so its meaning stays unambiguous.

### 12.2 Static systems check

`fullCheckCommand()` — a `runOnce` with `ignoringDisable(true)`, bound to the `SystemsCheck/Run` dashboard button and auto-run on every Test-mode entry. Read-only; safe with hands on the robot. Four phases: every device's `isConnected()`, every camera's heartbeat, gyro signal validity (`getYaw()` status OK and finite), and battery ≥ 12.0 V (meaningful only because nothing is drawing current). Output: per-item booleans under `SystemsCheck/…`, a one-line `SystemsCheck/Result` (PASS or "FAIL: n issues"), and a human-readable `SystemsCheck/Report` listing each problem (`OFFLINE: Shooter/PivotFollower`, `LOW BATTERY: 11.82 V …`).

### 12.3 Active motor test

The static check proves a motor is *on the bus*; it cannot see a stripped gear, sheared shaft, or dead phase. `SystemsCheck/RunMotorTest` actuates every mechanism one at a time and verifies each motor's **own encoder** responds. Total runtime ≈ 9.4 s.

| Phase | Action | Pass criterion |
|---|---|---|
| Baseline | Pivot + pinion forced to STOW, 0.75 s settle | — (known start configuration) |
| Flywheels | All four to LOB (40 RPS), 1.0 s | each motor's \|velocity\| > 5 RPS |
| Indexer | Feed 0.5 s | \|velocity\| > 5 RPS (reads ~45 RPS rotor at −4.5 V) |
| Pivot | To SHOT_BLOCK, 0.9 s, then back | each motor moved > 0.1 output rot *from its own start* |
| Roller | Intake 0.6 s | \|velocity\| > 5 RPS |
| Pinion | Deploy to GROUND 1.2 s, restow 1.0 s | each motor moved > 1.0 rot from its own start |
| Steer | All modules to 90°, 0.9 s, back to 0° | per-module angle error **folded to ±90°** < 10° |
| Drive | Robot-centric 0.4 m/s for 0.5 s, then brake | each wheel \|speed\| > 0.1 m/s |

Design points, several of them code-review products:

- Everything drives through the normal state-based closed-loop paths (MotionMagic, VelocityVoltage) — nothing is commanded open-loop into a hard stop, and `finallyDo` on every sub-test restores safe states even if interrupted mid-step.
- The steer criterion folds the error modulo 180° because Phoenix steer optimization may legally point a module 180° opposite with drive reversed whenever that's the shorter rotation (wheels left at X-lock's ±45° flip when commanded to 90°). A reversed module is a *working* module; the pre-review check would have false-failed it.
- The pivot criterion measures each motor against its own captured start (the pre-review version compared the follower against the leader's start, which could false-pass a dead follower if the encoders had diverged).
- The button is an always-runnable `runOnce(...).ignoringDisable(true)` that *schedules* the real sequence only when enabled, and reports `SKIPPED — enable the robot first` when disabled. A plainly-guarded sequence cannot even start while disabled (the scheduler refuses motion-capable commands), which made the button look broken — the wrapper restores honest feedback. Scheduling an already-running command is a no-op, so double-clicks can't restart a test mid-run.
- Operational requirements, printed in the running message: **on blocks** (a dead drive motor on carpet gets dragged by the other three and can read as moving; free-spinning wheels can't lie) and **no balls loaded** (the indexer step feeds toward a stopped drum; a leftover ball can wedge).

Per-motor results land under `SystemsCheck/{Shooter,Lintake,Drive}/…`, with per-subsystem `Pass` flags rolled into `SystemsCheck/MotorTest/Result`.

---

## 13. Telemetry and Tuning Infrastructure

### 13.1 DogLog

Initialized in the `Robot` constructor: WPILOG file output, live NetworkTables mirroring (`ntPublish`), and DS/joystick capture. The always-on channel set:

- **Per-loop shooter suite**: shot distance, hub heading error, target/actual pivot, target/commanded/actual RPS, RPS error, all readiness gates (hub and pass), average flywheel current, indexer current and volts, all three state enums, shot count, ball-in-flywheel.
- **Battery**: `Battery/Voltage` and `Battery/BrownedOut` every loop from `robotPeriodic` — sag is invisible on a dashboard after the fact; these traces are the first thing to check when shots start missing low.
- **Discrete `Shooter/ShotEvent/*` records** (§8.7) — the per-shot calibration snapshots.

`Robot` also lowers the brownout threshold to 6.0 V (`setBrownoutVoltage`, RIO 2 only, wrapped in a try so a RIO 1 keeps its default) — transient dips during a full-power shot-while-driving stay survivable instead of cutting outputs at the 6.75 V default.

### 13.2 LoggedTunableNumber

A ~60-line utility modeled on the 6328/1678/2910 pattern, backed by SmartDashboard: constructor seeds the dashboard key without clobbering an existing value; `get()` returns the live dashboard number while the global tuning mode is on, and the compiled default when off; `hasChanged()` is true on first call and thereafter only when the value moved, which gates expensive reactions (the flywheel gains re-apply Slot0 configs over CAN only on change, instead of hammering the bus every loop).

**Tuning mode is a single global switch** — `LoggedTunableNumber.setTuningMode(...)` in the `Robot` constructor, currently `true`. The competition checklist item: set it `false` before an event, freezing every knob at its compiled default so a stray dashboard edit can't change gains mid-match. The complete knob catalog is Appendix C.

### 13.4 Feature flags

`FeatureFlags` (in `config`, boolean analog of the tunable numbers, backed by `FeatureFlag`) exposes dashboard toggles under `FeatureFlags/*` for disabling a behavior live without a redeploy — the field escape hatches when something misbehaves mid-session: `VisionFusion` (off → pure odometry), `ShootOnTheMove` (off → aim as if stationary), `ShiftsGate` and `AllianceZoneGate` (off → force a hub-shot offer regardless of SHIFTS state or field position), `IdleSpin`, `DriverAssists`, and `PowerManager`. Unlike tunable numbers these are **always live** (not gated by tuning mode) — intervening during a match is the entire point — and every flag defaults to the normal robot, so an untouched flag changes nothing. Each is wired at the single point where its behavior branches (the vision loop, the shot solver's lead term, the right-trigger condition, the idle-spin computation, the assist trigger, the power manager).

### 13.3 The field workflow

The intended loop, encoded in `FIELD_TESTING.md`: pit health checks (fault monitor → static check → motor test on blocks) → pose/vision sanity (Field2d vs. tape measure — *fix pose before tuning anything*) → aim and zone-gate sanity → the shot-table sweep at 1.5/2.5/3.5/4.5/5.0 m using the `last shot` string and dashboard trims → pass tuning (flatter-pivot iteration) → assist/utility checks → autos in escalating order → failure drills (cover the camera, pull the controller, run a dying battery) → **bake every surviving trim into `ShooterConstants`, zero the dashboard offsets, lock tuning mode.**

---

## 14. Failure History and Lessons Learned

Documented because the fixes only stay fixed if the mechanisms are remembered.

**1. Empty CANivore bus name — total drivetrain silence.**
*Symptom:* code deploys clean, robot enables, nothing moves.
*Cause:* `new CANBus("", …)` — the swerve library enumerated a bus with no devices and raised no error.
*Fix:* `"drivetrain"`, matching the CANivore.
*Lesson:* device-discovery failures are silent; this is why the fault monitor (§12.1) now proves every device's presence every loop.

**2. `setPose()` returned a Command nobody scheduled — vision silently dead.**
*Symptom:* pose frozen regardless of driving or visible tags; 12,481 "rejected" measurements; hours of Limelight-side debugging.
*Cause:* the vision path called a `Command`-returning helper for its side effect; an unscheduled Command is a no-op. The same pattern had already bitten an intake stow.
*Fix:* imperative `resetPose()` at the call site; the wrapper deleted; the rule written into the code comment.
*Lesson:* in command-based code, "returns a Command" and "does the thing" are different contracts — APIs here now offer imperative forms for anything called from periodic logic.

**3. Aim oscillation from a self-exciting compensation term.**
*Symptom:* robot hunts side-to-side around the hub bearing, never settles; looks exactly like bad PID.
*Cause:* gyro-rate × TOF lead *inside the heading target* — the controller chased a target its own motion pushed away (loop gain ≈ 6) — compounded by a deg/s-as-rad/s unit error that pinned the term at its clamp.
*Fix:* term removed; D = 0.2 added for the residual mechanical overshoot; a do-not-re-add comment with the analysis.
*Lesson:* never put a term derived from the controller's own output into the target the controller chases; and distrust "just needs tuning" when the structure can be interrogated.

**4. Wrong hub targeted under a mis-set practice DS.**
*Symptom:* aim "basically forward" everywhere; hood/RPS pinned at max.
*Cause:* alliance-hub selection under a DS set to the other alliance — target 9 m away, off the table's domain.
*Fix:* closest-hub targeting without FMS; alliance hub with FMS; and later, the zone/SHIFTS gates take their alliance from the DS *deliberately*, with the checklist step to set it.
*Lesson:* practice-environment misconfiguration is a first-class failure mode; code should degrade sensibly under it.

**5. Vision jitter shaking the aim.**
*Symptom:* heading twitches while tracking a single tag.
*Cause:* per-frame hard pose resets replayed 1–2 cm botpose noise straight into the aim bearing chased at P = 8.
*Fix:* first a 3 cm deadband; then properly, the snap/fuse split with distance-scaled σ (§5.2).
*Lesson:* trust models beat thresholds — the Kalman filter was already there to do this job.

**6. Hand-adapted auto paths missing the fuel.**
*Symptom:* PathPlanner sim shows collect legs sweeping empty carpet.
*Cause:* paths re-drawn from screenshots of 2910's routes instead of using their published files.
*Fix:* byte-for-byte import, twice-prompted.
*Lesson:* when copying a proven artifact, copy it exactly first and modify second.

**7. Full-speed spin on a Red DS (the interaction bug).**
*Symptom:* the robot spins in place at full speed on driving into the tower zone — but only with the DS set to Red.
*Cause:* `FieldCentricFacingAngle` rotates its target by the operator perspective (180° on Red, verified in the Phoenix 26.1.0 sources), but every heading target here is an absolute blue-origin bearing. On Red the target sat ~180° away; the assist's 60° re-snap hysteresis kept it permanently ahead of the robot → continuous spin. The same flip silently faced hub/pass aim 180° from the target on Red. Blue's perspective rotation is 0°, so the blue-side field session never saw it.
*Fix:* the drivetrain caches the exact perspective rotation it hands Phoenix and exposes `fieldToOperatorBearing()`, which pre-rotates each target by the inverse; applied at all three facing-angle sites.
*Lesson:* two individually-correct behaviors (perspective rotation, snap hysteresis) composed into a violent failure — and an alliance-only bug is invisible until you test the other alliance.

**8. Aiming's silent single point of failure: gyro heading.**
*Symptom:* none yet — found by audit. Aim depends entirely on the gyro being field-aligned, and vision (by deliberate design) only corrected translation, never heading.
*Cause:* a wrong gyro heading (no seed in teleop-only practice, a mid-match bump, a bad auto seed) would offset every shot by that error with no self-correction — and the Red-DS coupling of alliance→zone→perspective raised the stakes.
*Fix:* seed the gyro from **multi-tag** vision heading (unambiguous) once, and re-align only on >10° divergence, so the Pigeon still owns fine rotation but can no longer be silently wrong. `vision/heading seeded` surfaces the state.
*Lesson:* "trust one sensor" is fine only if that sensor is guaranteed correct at least once; a design that never cross-checks a critical value is a latent failure even when it's a deliberate choice.

---

## 15. Known Limitations and Roadmap

1. **Manual ferry (POV-right) fires by timeout.** It gates on `readyToShoot()`'s SCORE targets, which a LOB/SEND spin-up never satisfies, so it always feeds at 1.25 s. The smart pass path fixed this class of problem with `readyToPass()`; fold LOB/SEND onto commanded-value gates if the manual ferry stays in use.
2. **Long-range table rows (4.5–5.0 m) are extrapolation.** One anchored field session calibrated midrange; the far rows carry a modeled drag allowance. Verify at range next session — the shot-event log makes each attempt a data point.
3. **Pass tables are unfired.** Modeled with the same ×1.30 factor but never landed on a real field, and the flatter pass pivot shifts them further. Expect `PassRpsOffset` work.
4. **The 3-second game-data gap.** Between teleop start and game-data arrival the code assumes the hub is active (per WPILib guidance); a pull in that window could offer HUB during what becomes an inactive shift. Accepted: never guess on a real field.
5. **Double-cycle time budget unverified.** The second cycle's fit inside the auto period is a stopwatch question; path max-velocity tuning is the lever if it's close.
6. **Camera mount pose is web-UI-only.** `LocalisationConstants` documents it, but nothing pushes it to the camera; a mismatch between the UI and reality is invisible to code review. Consider `setCameraPose_RobotSpace` at startup so it's version-controlled.
7. **Single camera, heuristic σ.** A second Limelight (or measured standard deviations) would tighten long-range aim and cover the blind arc while intaking.
8. **Flywheel kA is hardcoded (0.2)** in both the config and the live-gain re-apply; fold in the SysId value once the characterization is run on the final drum.
9. **Architecture (offseason):** an IO-layer abstraction for simulation/replay, a superstructure coordinator, and a targeting class that owns all shot math (currently resident in the drivetrain) — the 6328/1678/2910 patterns — deliberately deferred mid-season in favor of field time.

---

## Appendix A: Controller Map

*(Xbox controller, port 0 — duplicate of §10.3 for pit-wall printing.)*

| Input | Action |
|---|---|
| Left stick | Field-centric translate |
| Right stick X | Rotate |
| RT (hold) | Context shot (latched): HUB if in range + in zone + hub active, else PASS |
| A (hold) | Aim at hub only |
| POV-up (hold) | Forced smart pass |
| POV-right (hold) | Manual ferry (LOB pivot, 90 RPS, no aim) |
| LT (hold) | Intake rollers |
| POV-left (hold) | Eject rollers |
| LB / RB | Intake deploy / stow |
| X (hold) | X-lock brake |
| Y (hold) | Shot-block pivot + intake stow |
| B (hold) | Clear jam (reverse) |
| POV-down | Re-seed field-centric heading |
| Rumble | Shot/pass locked and ready |

## Appendix B: Dashboard Key Catalog

| Key | Meaning |
|---|---|
| `Field` | Field2d pose widget |
| `Auto` | Auto chooser |
| `dist to hub` / `hub visible` | Range to nearest hub; any tag in view |
| `vision/pose x·y`, `vision/tag count`, `vision/avg tag dist`, `vision/pose resets` | Raw vision + snap counter |
| `aim/target hub x·y`, `aim/hub heading deg` | Which hub the math targets and the commanded bearing |
| `aim/auto mode`, `aim/in alliance zone` | What an RT pull would latch; zone gate state |
| `our hub active`, `active hub alliance` | SHIFTS state |
| `pass/target x·y`, `pass/distance` | Ferry landing point |
| `ready/shot in range · aimed · shooter · pivot` (+ pass variants) | Individual readiness gates |
| `ready to shoot`, `shot compensated distance`, `shot time of flight` | Composite readiness + solution |
| `interpolated pivot position`, `interpolated shooter rps`, `Actual shooter speed`, `shooter position`, `shooter current draw` | Live shooter targets/actuals |
| `last shot` | Most recent shot-event snapshot string |
| `pivot overcurrent tripped`, `indexer overcurrent tripped` | Protection flags |
| `assist/zone` | NONE / TRENCH_NEAR / TRENCH_IN / TOWER_NEAR / TOWER_IN |
| `PinionState`, `RollerState`, `RollerVoltage`, `*PinionPosition*` | Lintake state |
| `Faults/*` | Continuous per-device connectivity + camera liveness |
| `SystemsCheck/*` | Static check + motor test results and per-motor flags |
| `SysId/Shooter *` | Flywheel characterization buttons |
| `Sim/PoseX·Y·HeadingDeg`, `Sim/ApplyPose` | Simulation pose setter |

## Appendix C: Live-Tunable Parameter Catalog

All under `LoggedTunableNumber` (frozen at defaults when tuning mode is off) except where noted.

| Key | Default | Effect |
|---|---|---|
| `ShotTuning/HeadingOffsetDeg` | 0 | Mechanical aim trim (1° steps; applies to hub and pass bearing) |
| `ShotTuning/MoveCompGain` | 0.7 | Shoot-on-the-move velocity lead scale (0 = aim as if stationary) |
| `ShotTuning/ShootingSpeedScale` | 0.45 | Translation cap while aiming |
| `ShotTuning/PivotOffset` | 0 | Shifts entire hub pivot table |
| `ShotTuning/ShooterRpsOffset` | 0 | Shifts entire hub RPS table |
| `ShotTuning/TimeOfFlightOffset` | 0 | TOF trim (raw SmartDashboard number) |
| `ShotTuning/PassRpsOffset` | 0 | Shifts entire pass RPS table |
| `ShotTuning/PassPivotRot` | 0.086 | Pass launch angle (higher = flatter) |
| `ShotTuning/PassCornerInsetM` | 1.0 | Pass aim point inset from corner walls |
| `ShotTuning/FlywheelKp / Ks / Kv` | 0.3 / 0.15 / 0.125 | Live flywheel gains (re-applied on change) |
| `ShotTuning/ShotBlockPivotPosition` | 0.5 | Blocker-clearance pivot position |
| `ShotTuning/PivotCurrentLimitA / PivotCurrentTimeoutS` | 40 / 0.25 | Pivot overcurrent trip |
| `ShotTuning/IndexerCurrentLimitA / IndexerCurrentTimeoutS` | 25 / 0.2 | Indexer overcurrent trip |
| Shot-detection dip / recover thresholds | 5.0 / 2.0 RPS | Ball-passage detector sensitivity |
| `Sim/PoseX / PoseY / PoseHeadingDeg` | 4.6 / 0.75 / 0 | Pose-setter inputs |

## Appendix D: CAN ID Map

| ID | Device | Bus |
|---|---|---|
| 1 | Pigeon 2 | drivetrain |
| 3 / 4 / 33 | FL drive / steer / encoder | drivetrain |
| 22 / 9 / 47 | FR drive / steer / encoder | drivetrain |
| 55 / 61 / 62 | BL drive / steer / encoder | drivetrain |
| 19 / 54 / 46 | BR drive / steer / encoder | drivetrain |
| 5 / 6 | Pivot leader / follower | rio |
| 7 / 8 / 11 / 49 | Flywheel: left leader / back-left / right / back-right | rio |
| 53 | Indexer (Kraken X44) | rio |
| 35 / 2 | Pinion leader / follower | rio |
| 45 | Intake roller | rio |

## Appendix E: Feature Flags and Power Modes

**Feature flags** (`FeatureFlags/*`, always live, all default ON except PowerManager):

| Flag | Off behavior |
|---|---|
| `VisionFusion` | Pose runs on wheel odometry only |
| `ShootOnTheMove` | Aim as if stationary (no velocity lead) |
| `ShiftsGate` | Right trigger offers HUB regardless of SHIFTS state |
| `AllianceZoneGate` | Right trigger offers HUB regardless of field position |
| `IdleSpin` | Flywheel only spins on a shot command |
| `DriverAssists` | Fully manual driving everywhere |
| `PowerManager` (default **OFF**) | Static compiled current limits (IDLE mode) |

**Power modes** (supply amps per motor, applied by `PowerManager` on the desired-mode change):

| Mode | Drive | Flywheel | Trigger |
|---|---|---|---|
| IDLE | 40 | 35 | Default / feature off (= static compiled limits) |
| SCORING | 25 | 50 | RT / A / POV-up held |
| SPRINT | 65 | 20 | Left-stick magnitude > 0.9, not aiming |
| AUTO | 55 | 45 | Autonomous enabled |
