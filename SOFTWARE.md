# Eclipse Robot Software — Technical Report

**FRC Team 2035 — 2026 Season (*REBUILT*)**
Repository: `CarmelRobotics/Eclipse`, branch `summer` · Java 17, WPILib 2026 command-based, GradleRIO
Major dependencies: CTRE Phoenix 6 (26.1.0), PathPlanner (2026.1.2), DogLog (2026.5.0)
Last updated: 2026-07-06 (post field-calibration session; pre second field session)

---

## 1. Executive Summary

Eclipse is a swerve-drive robot for the 2026 game *REBUILT*, in which robots collect FUEL (5.91" foam balls) and score it through the top opening of a Hub whose front lip is 72" off the carpet, while the two alliances' hubs alternate active/inactive in 25 s SHIFTS. The software is a command-based WPILib project built around a consistent pattern: **commands set enum states, and each subsystem's `periodic()` continuously drives hardware toward its current state.**

Feature set: dual-mode vision-fused odometry (snap when lost, Kalman-fuse when tracking), field-calibrated shot tables, shoot-on-the-move compensation, context-aware shooting (hub shot vs. ferry pass chosen from range + alliance zone + SHIFTS hub-active state), trench/tower driver assists, sensor-free shot detection with per-shot calibration logging, a static systems check plus an active motor self-test, continuous fault monitoring, live dashboard tuning with a competition lockout, and structured WPILOG logging via DogLog. `FIELD_TESTING.md` is the companion field-session script.

---

## 2. Project Layout

```
src/main/java/frc/robot/
├── Main.java / Robot.java / Constants.java     — entry point, logging + brownout init, speed limits
├── RobotContainer.java                          — subsystems, bindings, assists, shot commands, autos
├── util/LoggedTunableNumber.java                — live-tunable dashboard values
└── subsystems/
    ├── drive/        TunerConstants, CommandSwerveDrivetrain (aiming + targeting live here)
    ├── shooter/      Shooter, ShooterConstants
    ├── lintake/      Lintake, LintakeConstants
    ├── diagnostics/  SystemsCheck (static check, fault monitor, motor test)
    └── localisation/ LimelightHelpers (vendor), LimelightInfo, LocalisationConstants
src/main/deploy/pathplanner/                     — 2910-derived trench autos + legacy routines
FIELD_TESTING.md                                 — ordered field-session checklist
```

`Constants.kMaxSpeed` = drivetrain rated speed at 12 V; `kMaxAngularRate` = 1 rot/s.

---

## 3. Drivetrain

### 3.1 Hardware & configuration

CTRE-generated swerve (Tuner X project) — four modules on the `drivetrain` CANivore bus, Pigeon 2 (ID 1), FusedCANcoder steering feedback, 2.05" wheel radius, 5.27:1 drive / 26.09:1 steer ratios, 120 A slip current.

| Module | Drive | Steer | Encoder |
|---|---|---|---|
| Front-left | 3 | 4 | 33 |
| Front-right | 22 | 9 | 47 |
| Back-left | 55 | 61 | 62 |
| Back-right | 19 | 54 | 46 |

Gains: steer kP 33 / kD 0.5 / kS 0.1 / kV 2.49 (voltage output); drive kP 0.1 / kV 0.124.

**Brownout defenses** (added after field testing): drive motors capped at **40 A supply**, steer at **20 A supply**, and a **0.15 s open-loop voltage ramp** flattens stick-slam current spikes in teleop (auto path following is closed-loop and untouched). `Robot` lowers the RIO 2 brownout threshold to **6.0 V** so transient sags during full-power shooting-while-driving don't cut outputs; `Battery/Voltage` and `Battery/BrownedOut` are logged every loop.

### 3.2 Vision-fused localization

One Limelight (`limelight-four`). Every loop the robot pushes its heading + yaw rate to the camera (`SetRobotOrientation`), then reads the **blue-origin** botpose (everything downstream — Field2d, hub positions, assist zones, PathPlanner — is blue-origin). Two regimes, split by how far vision is from current odometry (`kMaxVisionCorrectionMeters` = 1.0 m):

- **Far off (> 1 m)** — the pose is simply wrong (startup, robot carried, odometry ran blind). **Snap:** `resetPose` to vision XY + the gyro's heading (vision heading jitters; the Pigeon owns rotation). A reset counter is published — if it climbs steadily while tags are visible, localization is fighting itself.
- **Tracking (≤ 1 m)** — feed the built-in Kalman filter (`addVisionMeasurement`) with distance-scaled trust: σ = 0.05 + 0.02·d² meters (0.07 m at 1 m, 0.37 m at 4 m), heading σ = ∞. This smooths single-tag jitter and absorbs the garbage frames cameras produce as a tag leaves the FOV.
- **No tags** — odometry carries the pose (cm/s drift), so aiming keeps working through vision dropouts.

> **Hard-learned rule:** pose changes go through `resetPose()`/`addVisionMeasurement()` **imperatively**. A Command-returning helper that isn't scheduled silently does nothing — that bug shipped twice (lintake stow, vision seeding) and cost a field session.

> **Note:** the camera's mount pose (0.35 m forward, 0.15 m up, 20° pitch in `LocalisationConstants`) is *not* pushed by code — it must match the Limelight web UI config.

### 3.3 Driver assists (trench + tower)

Geometry from the official AprilTag layout: barrier lines at x 4.02–5.23 (blue) / 11.31–12.52 (red); trench corridors are the 1.668 m of each line nearest the guardrails; towers reach 0.99 m off each alliance wall, ±0.63 m around centers y = 3.962 (blue) / 4.108 (red). Each structure has two zones:

- **INSIDE** — heading locks to the nearest 90° increment (with 60° re-snap hysteresis so it can't flicker) and translation rails to the axis the robot faces, so it glides through without drifting into walls. The shooter stows to clear the bar and the intake deploys to sweep fuel; the intake restows on exit.
- **NEAR (0.8 m halo)** — the assist is a suggestion: heading snaps only while the rotation stick is centered; any rotation input hands heading straight back to the driver. Translation always manual.

Aiming (RT / A / POV-up) outranks the assist, and the X-lock brake is folded into the assist trigger so releasing the brake re-arms the assist immediately (a plain interrupt would have killed it until zone re-entry — `whileTrue` only reschedules on a rising edge).

### 3.4 Targeting & shot selection

- **Which hub:** with FMS attached, the alliance hub; in the shop, whichever hub is *closer* — practice driver stations routinely sit on the wrong alliance, and aiming at a hub 9 m away made the robot "aim forward" from everywhere while clamping the shot tables at max range.
- **Alliance-zone gate:** hub shots additionally require the robot on the wall side of its hub's X (the hub sits on the zone line: blue x ≤ 4.625, red x ≥ 11.925). The zone side comes from the **DS alliance** (FMS-assigned in matches, manually set in the shop). Without this, a robot past the barrier could still be "in range."
- **SHIFTS gate:** hub shots also require `isOurHubActive()` — during the opponent's shift the same trigger pull ferries instead of donating balls to a dead hub. Timeline (verified against the WPILib 2026 game-data reference, 140 s teleop countdown): 140–130 both active (transition), four 25 s shifts to 30, final 30 s both active (endgame). Game data (`'R'`/`'B'` = alliance inactive first) arrives ~3 s into teleop; empty data assumes active. **DS practice mode:** with no FMS, empty game data synthesizes `'B'` so practice runs actually cycle the shifts (type R/B in the DS Game Data box to pick the order; set practice teleop to 140 s).
- **Right trigger latches** its HUB/PASS choice at the pull — the mode can't flip mid-hold at a boundary. `aim/auto mode` and `aim/in alliance zone` on the dashboard mirror the exact live condition.

### 3.5 Shoot-on-the-move

Solved **once per loop** and cached (it used to be re-derived 5–6× per loop from raw module speeds — the source of on-the-move twitchiness). Field velocity is low-pass filtered (0.15 s IIR) and scaled by `ShotTuning/MoveCompGain` (0.7 default — deliberate under-lead for target stability); TOF and lead are fixed-point iterated 3× so the flight time matches the led distance. Distance to the virtual target (clamped 0–6 m) drives the table lookups; its bearing plus `ShotTuning/HeadingOffsetDeg` (mechanical trim) is the aim heading. `faceHubCommand` heading-locks the swerve (P = 8, **D = 0.2**) while the driver translates freely at the shooting speed scale (45%).

> **Removed on purpose — do not re-add:** an "aim ahead while spinning" term (gyro rate × TOF) inside the aim target. The heading controller's own rotation fed the term, pushing the target further ahead, speeding the rotation — a self-exciting loop (gain ≈ P·TOF ≈ 6) that oscillated around the hub and never settled. It also used deg/s as radians. If spin compensation is ever needed, apply it once at ball release, never inside the target the controller chases.

---

## 4. Shooter

### 4.1 Hardware map

| Component | Motors (TalonFX) | Control |
|---|---|---|
| Flywheel drum (4" wheels, 1:1) | 7 (leader), 8, 11, 49 | `VelocityVoltage`, all four commanded identically (inverts in config) |
| Pivot (42:1) | 5 (leader, CW+), 6 (follower, CCW+) | MotionMagic position (cruise 80, accel 160, jerk 1600; kP 4.8, kD 0.1, kS 0.25, kV 0.12) |
| Indexer / kicker | 53 (Kraken X44) | Direct voltage (−4.5 V feed, +4.5 V reverse) |

Flywheel gains: kP 0.3, kS 0.15, kV 0.125, kA 0.2, with supply/stator current limits — the supply cap exists so the 4-motor shot spike can't sag the battery and starve the voltage-based velocity controller. Status signals actually read by code (including the follower velocities and indexer velocity used by the motor self-test) are explicitly kept alive at 50–100 Hz before `optimizeBusUtilization()` disables the rest.

### 4.2 State machines

- `ShooterState`: `ZERO` (smart idle — see below), `SCORE` (table RPS), `PASS` (distance-interpolated ferry), `LOB` (40), `SEND` (90), `REVERSE` (−10, jam clearance)
- `PivotState`: `STOW` (0), `SCORE` (table angle), `LOB` (25° = 0.0694 rot — nominal kicker feed limit), `PASS` (tunable, default 0.086 — flatter ferry for backspin mitigation), `SHOT_BLOCK` (tunable, default 0.5 rot)
- `IndexerState`: `ZERO` (0 V), `SCORE` (−4.5 V), `REVERSE` (+4.5 V)

**Smart idle:** `ZERO` pre-spins the drum at 6.7 RPS for fast spin-up, but only within 6.5 m of the hub (beyond that it's wasted draw) and only while battery > 7.0 V — under load the nice-to-have pre-spin is shed to protect the drivetrain.

### 4.3 Shot physics and tables (field-calibrated)

Three `InterpolatingDoubleTreeMap`s map **motion-compensated distance → pivot / RPS / TOF**. The shape comes from the minimum-energy lob into a 72" opening from a ~12" release (θ = 45° + ½·atan(Δh/d), v² = g·(Δh + √(d² + Δh²))); the *values* were re-anchored on the 2026-07-04 field session (a ×1.30 field factor over bare physics, absorbing drag and wheel-to-ball transfer):

| Distance | Pivot (output rot) | RPS | TOF (s) |
|---|---|---|---|
| 1.5 m | 0.046 | 39.0 | 0.48 |
| 2.5 m | 0.065 | 41.5 | 0.62 |
| 3.5 m | 0.076 | 45.5 | 0.73 |
| 4.5 m | 0.083 | 49.0 | 0.83 |
| 5.0 m | 0.086 | 51.0 | 0.87 |

The pivot curve is floored by kicker clearance (at dead stow the shooter rests on the kicker wheels) and effectively saturates near the LOB angle at range. Long range (≥ 3.5 m) leans on RPS, which the drag signature demands scale progressively, not by a flat offset. Live trims: `ShotTuning/PivotOffset`, `ShooterRpsOffset`, `TimeOfFlightOffset`.

**Pass tables:** ferry distance (3–13 m) → 28–61 RPS, aimed at the alliance-zone corner on the robot's current side (0.5 m centerline hysteresis so the target can't flicker). Backspin is not code-controllable (single drum surface, no opposing wheels — spin and exit speed scale together), so its landing effect is managed two ways: passes launch at a **deliberately flatter angle** than LOB (`ShotTuning/PassPivotRot`, default 0.086 rot — flatter arrival keeps forward speed above the spin bite so balls roll out instead of checking up and bouncing back), and the aim point sits `ShotTuning/PassCornerInsetM` (1.0 m) in from the walls so any residual bounce-back settles in the zone. The RPS table was modeled at 25°; expect `PassRpsOffset` to trim up at the flatter angle.

### 4.4 Shot lifecycle

```
prepare (set pivot + flywheel states)
  → waitUntil(ready).withTimeout(spin-up cap)     ← 1.25 s shots / 2.0 s passes; never hangs
  → feed indexer (held: until release · auto: 1.5 s "shoot" / 2.5 s "dump")
finallyDo → stop flywheel, indexer, pivot          ← guaranteed cleanup on any interruption
```

`readyToShoot()` ANDs four gates — in range (≤ 5 m), aimed (≤ 5°), at speed (± 3 RPS), pivot in position (± 0.04 rot). `readyToPass()` gates on the PASS targets instead (ferry RPS, LOB pivot, ≤ 10° of the corner bearing) so passes fire on actual spin-up rather than blind timeout. Every gate is published individually, so a blocked shot is diagnosable at a glance. While either readiness is true during an aim hold, the controller **rumbles steadily** — the driver feels the firing window without looking down.

### 4.5 Sensor-free shot detection & per-shot logging

No beam break: a ball through the drum dips flywheel velocity. Once within 2 RPS of target (armed), a dip > 5 RPS followed by recovery counts one shot (thresholds tunable). Each detection also writes a **shot-event snapshot** — `Shooter/ShotEvent/*`: mode, distance (ferry distance for passes), commanded/actual RPS, pivot, heading error, battery, and **the live trim offsets in effect** — plus a glanceable `last shot` dashboard string. This is the calibration record: a shot that landed perfectly with `RpsOffset=+2` means the table is 2 RPS low *at that distance*, and now that's captured per-shot instead of remembered.

### 4.6 Protection

Software overcurrent supervision on pivot (40 A / 0.25 s) and indexer (25 A / 0.2 s): exceeding the limit for the window zeroes the motor and falls back safe (pivot → STOW, indexer → ZERO) with a dashboard flag; thresholds tunable live. A `clearJamCommand` (held button) reverses flywheel and indexer together to back out a stuck ball.

### 4.7 On-robot characterization (SysId)

Four dashboard buttons (`SysId/Shooter …`) run WPILib SysId voltage profiles on all four flywheel motors simultaneously, logging via CTRE SignalLogger to `.hoot` for Tuner X analysis. A `characterizing` flag suspends normal flywheel control while a routine owns the motors.

---

## 5. Lintake (Intake)

Pinion arms (motors 35 leader / 2 follower, brake mode, MotionMagic kP 1.25 / kD 0.15 / kV 0.25, cruise 55 / accel 135 / jerk 1600) position between `STOW` (−4), `AGITATE` (−7.5), and `GROUND` (−9.75) rotations. A roller (motor 45, coast, 30 A supply) runs ±12 V or 0. **Compliance is a current limit, not gains:** the pinion's 15 A stator cap bounds torque so the deployed intake back-drives instead of breaking when it hits a wall or the trench bar; 30 A supply is the brownout budget. During any shot, the pinion automatically bounces AGITATE ↔ GROUND so a full hopper (up to ~30 balls) keeps flowing into the indexer instead of bridging.

---

## 6. Systems Check & Fault Monitoring (`diagnostics/SystemsCheck`)

Three layers, born from two field-session failures that were invisible until the robot was on the carpet (a wrong CAN bus name, a silently-unscheduled Command):

1. **Continuous fault monitor** — every CTRE device's `isConnected()` plus a Limelight heartbeat (1 s staleness window) published under `Faults/*` every loop. A device falling off the bus mid-match is visible immediately.
2. **Static systems check** — read-only sweep of every device, camera, gyro, and battery; auto-runs on Test-mode entry and via the `SystemsCheck/Run` button (works disabled). `SystemsCheck/Result` = PASS or a named list of failures.
3. **Active motor test** — `SystemsCheck/RunMotorTest` (~9.4 s) actually actuates everything one mechanism at a time and verifies each motor's **encoder responds**, catching connected-but-mechanically-dead motors the static check can't see: flywheels → LOB speed (all four velocities), indexer feed, pivot to shot-block and back (both encoders, each against its own start), roller, pinion deploy/retract (both encoders), steer to 90° (error folded to ±90° so steer-optimization's legal 180° flips pass), and a brief 0.4 m/s drive — **the robot moves; run on blocks with no balls loaded**. Starts from forced-stow known state; the button reports SKIPPED when clicked disabled (the real sequence can't be scheduled then); per-motor pass/fail flags roll up to `SystemsCheck/MotorTest/Result`.

---

## 7. Operator Interface (Xbox, port 0)

| Input | Action |
|---|---|
| Left stick / right stick | Field-centric translate / rotate (0.1 deadband) |
| **Right trigger (hold)** | Context shot, latched at pull: HUB (in range + in zone + hub active) else PASS; translation capped at 45% while aiming |
| A (hold) | Aim at hub (heading lock only) |
| POV-up (hold) | Forced smart pass to the corner |
| POV-right (hold) | Manual flat ferry (LOB pivot, fixed 90 RPS) |
| Left trigger / POV-left (hold) | Roller intake / eject |
| LB / RB | Deploy / stow intake |
| **X (hold)** | X-lock defense brake |
| Y (hold) | Pivot to shot-block; stow on release |
| B (hold) | Clear jam (reverse flywheel + indexer) |
| POV-down | Re-seed field-centric heading |
| Rumble | Steady while the latched shot/pass is locked and ready |

Automatic: trench/tower assists (§3.3), shot agitation (§5), smart idle (§4.2), disabled-mode drivetrain idle.

**Simulation pose setter:** edit `Sim/PoseX`, `Sim/PoseY`, `Sim/PoseHeadingDeg` on the dashboard and click `Sim/ApplyPose` to teleport the robot — field-position testing without the separate sim GUI.

---

## 8. Autonomous

PathPlanner `AutoBuilder` (translation PID 10/0/0, rotation 7/0/0, alliance-aware flipping, pathfinding warmup) with a dashboard chooser. Named commands: `intake deploy/retract/run/stop`, `shoot` (1.5 s feed), `dump` (2.5 s feed — empties preload + a collect cycle), `stopshoot`.

Primary routines are built from **team 2910's published paths, copied byte-for-byte** (hand-adapted versions collected empty carpet; the originals are field-proven), with score poses since shortened in-house:

- **`2910_trench_left` / `2910_trench_right`** — preload start → far mid-field collect → near mid-field → trench return → score, intake running throughout, 4 s dump.
- **`2910_trench_left_double` / `2910_trench_right_double`** — the above plus a second collect cycle through the alternate fuel row and a second dump.

Legacy routines (`preload`, `testauto`, `doubleswipe`) are retained for bring-up. All autos reset odometry to the path start; vision corrects from there, so starting placement matters for the first seconds.

---

## 9. Telemetry & Tuning Infrastructure

- **DogLog** (init in `Robot`): WPILOG + live NT mirroring + DS capture. Per-loop shooter suite (targets, actuals, errors, gates, currents, states), battery traces, and discrete `Shooter/ShotEvent/*` records (§4.5).
- **`LoggedTunableNumber`**: every knob under `ShotTuning/` and `Sim/`. Flywheel kP/kS/kV re-apply over CAN only on change. **Tuning mode is currently ON** (`Robot` constructor); set `LoggedTunableNumber.setTuningMode(false)` before competition so a stray dashboard edit can't change gains mid-match.
- **Field workflow:** `FIELD_TESTING.md` is the ordered session script — health checks → motor test → pose/vision → zone/aim sanity → shot sweep (the `last shot` string + ShotEvent records collect calibration data automatically) → pass → assists → autos → failure drills → bake trims into `ShooterConstants` and lock tuning mode.

---

## 10. Known Limitations & Next Steps

1. **LOB/SEND fire via timeout** — the manual ferry (POV-right) still gates on `readyToShoot()`'s SCORE targets it can never satisfy, so it feeds at the 1.25 s timeout. The smart pass path (readyToPass) doesn't have this problem; fold LOB/SEND onto commanded-value gates if the manual ferry stays in use.
2. **Long-range table confidence** — the 3.5–5.0 m rows extrapolate from one anchored field session; verify at range next session (the shot-event log makes this cheap).
3. **Game-data gap at teleop start** — data arrives ~3 s in; until then the code assumes active (per WPILib guidance), so an RT pull in that window could offer HUB during what becomes an inactive shift.
4. **Double-cycle time budget unverified** — confirm both cycles complete in the 140 s teleop practice run / real auto period allotted, and that the shortened score poses still dump reliably.
5. **Limelight mount pose** lives only in the camera web UI; consider pushing it from code at startup so it's version-controlled.
6. **Single camera, heuristic σ** — a second Limelight and measured std-devs would tighten long-range aim.
7. **`kA` hardcoded (0.2)** in the live-gain re-apply path; fold in the SysId value when known.
8. **Architecture (offseason):** IO-layer abstraction, a superstructure coordinator, and a targeting class owning all shot math (currently in the drivetrain) — deliberately deferred mid-season.

---

## 11. Recent Development (branch `summer`, newest first)

- **SHIFTS integration** — hub-active gates the context shot; DS practice mode cycles real shifts (synthesized game data); timeline verified against the WPILib reference (140 s teleop, 10 s transition, 30 s endgame)
- **Per-shot calibration logging** — `Shooter/ShotEvent/*` + `last shot` dashboard string
- **Diagnostics suite** — fault monitor, static systems check, active motor test; hardened by code review (direction-agnostic steer check, disabled-click messaging, per-encoder pivot baselines)
- **Alliance-zone shot gating** — DS-alliance zone with the hub X as the zone line
- **Driver QoL** — X-lock brake (X), assist/brake trigger interplay, dashboard pose setter, `FIELD_TESTING.md`
- **Field session 2026-07-04** — root-caused dead CAN bus name, silently-unscheduled pose reset, aim oscillation (gyro-rate feedback), wrong-hub targeting; shot tables re-anchored on field data (×1.30); vision rewritten to snap/fuse dual-mode
- **2910 autos** — verbatim path import, single + double trench cycles, `dump` named command
