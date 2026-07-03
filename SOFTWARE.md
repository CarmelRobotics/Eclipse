# Eclipse Robot Software — Technical Report

**FRC Team 2035 — 2026 Season (*REBUILT*)**
Repository: `CarmelRobotics/Eclipse`, branch `summer` · Java 17, WPILib 2026 command-based, GradleRIO
Major dependencies: CTRE Phoenix 6 (26.1.0), PathPlanner (2026.1.2), DogLog (2026.5.0)

---

## 1. Executive Summary

Eclipse is a swerve-drive robot for the 2026 game *REBUILT*, in which robots collect FUEL (5.91" foam balls) and score it through the top opening of a Hub whose front lip is 72" off the carpet. The software is a command-based WPILib project with three mechanism subsystems (shooter, intake, drivetrain-integrated aiming) built around a consistent pattern: **commands set enum states, and each subsystem's `periodic()` continuously drives hardware toward its current state.** The codebase includes vision-fused odometry, shoot-on-the-move compensation, physics-derived shot tables, sensor-free shot detection, live dashboard tuning with a competition lockout, on-robot flywheel characterization (SysId), and structured logging to WPILOG via DogLog.

---

## 2. Project Layout

```
src/main/java/frc/robot/
├── Main.java / Robot.java / Constants.java     — entry point, logging init, speed limits
├── RobotContainer.java                          — subsystems, bindings, shot commands, autos
├── util/LoggedTunableNumber.java                — live-tunable dashboard values
└── subsystems/
    ├── drive/       TunerConstants, CommandSwerveDrivetrain
    ├── shooter/     Shooter, ShooterConstants
    ├── lintake/     Lintake, LintakeConstants
    └── localisation/ LimelightHelpers (vendor), LimelightInfo, LocalisationConstants
src/main/deploy/pathplanner/                     — autos (preload, testauto, doubleswipe) + paths
```

`Constants.kMaxSpeed` = 6.0 m/s (the drivetrain's rated speed at 12 V); `kMaxAngularRate` = 1 rot/s.

---

## 3. Drivetrain

### 3.1 Hardware & configuration

CTRE-generated swerve (Tuner X project) — four modules on the RIO CAN bus, Pigeon 2 (ID 1), FusedCANcoder steering feedback, 2.05" wheel radius, 5.27:1 drive / 26.09:1 steer ratios, ±11" module positions, 120 A slip current, 60 A steer stator limit.

| Module | Drive | Steer | Encoder |
|---|---|---|---|
| Front-left | 3 | 4 | 33 |
| Front-right | 22 | 9 | 47 |
| Back-left | 55 | 61 | 62 |
| Back-right | 19 | 54 | 46 |

Gains: steer kP 33 / kD 0.5 / kS 0.1 / kV 2.49 (voltage output); drive kP 0.1 / kV 0.124. Includes CTRE SysId routines for translation, steer, and rotation characterization, and a 4 ms simulation thread.

### 3.2 Vision-fused localization

One Limelight (`limelight-four`) using the MegaTag2 flow:

1. Each loop, the robot's current heading and angular rate are pushed to the camera (`SetRobotOrientation`).
2. The alliance-appropriate botpose estimate (`wpiBlue`/`wpiRed`, function swapped on alliance change) is read back.
3. **Outlier rejection:** estimates farther than 1.0 m from current odometry are discarded (a rejection counter is published).
4. Accepted poses are fused into the drivetrain's Kalman filter with translation standard deviations scaled by tag quality — 0.25 m for far/small tags, `0.6^(avgTagDist+1)` otherwise — and heading deviation `Double.MAX_VALUE` (vision never corrects heading; the Pigeon owns it).

Fusion runs in **both teleop and autonomous**. Timestamps are converted with `Utils.fpgaToCurrentTime()` for correct latency compensation.

> **Note:** the camera's robot-space mount pose (0.35 m forward, 0.15 m up, 20° pitch in `LocalisationConstants`) is *not* pushed to the camera by code — it must be configured in the Limelight web UI. (The class that used to push it was dead code and has been removed.)

### 3.3 Trench automation

A `Trigger` geofence on the robot's field Y coordinate (≥ 7.3 m or ≤ 0.7 m) detects trench entry. While inside: heading locks to the nearer of 0°/180° with 30° hysteresis (so it can't flip-flop mid-trench), driver keeps full translation control, the shooter pivot stows to clear the trench bar, and the intake deploys to ground. On exit the intake restores to stow.

### 3.4 Game-data hub scheduling

*REBUILT* alternates which alliance's Hub is "active" in 25 s shifts, ordered by the game-specific message. The drivetrain decodes match time + game data (shift boundaries at 130/105/80/55/30 s) into `our hub active` and `active hub alliance` dashboard indicators, treating auto and empty game data as active per WPILib guidance.

---

## 4. Shooter

### 4.1 Hardware map

| Component | Motors (TalonFX) | Control |
|---|---|---|
| Flywheel drum (4" wheels, 1:1 direct) | 7 (leader), 8, 11, 49 | `VelocityVoltage`, all four commanded identically (inverts in config: left pair CW+, right pair CCW+) |
| Pivot (42:1) | 5 (leader, CW+), 6 (follower, CCW+) | MotionMagic position (cruise 80, accel 160, jerk 1600; kP 4.8, kD 0.1, kS 0.25, kV 0.12) |
| Indexer / kicker | 53 (Kraken X44) | Direct voltage (−4.5 V to feed) |

Flywheel gains: kP 0.3, kS 0.15, kV 0.125, kA 0.2, with 50 A supply / 100 A stator current limits — the supply cap exists specifically so the 4-motor shot spike can't sag the battery and starve the voltage-based velocity controller. Status signals actually read by code (velocity, position, currents) are explicitly kept alive at 50–100 Hz before `optimizeBusUtilization()` disables the rest.

### 4.2 State machines

- `ShooterState`: `ZERO` (0.5 RPS idle), `SCORE` (table RPS), `LOB` (40), `SEND` (90)
- `PivotState`: `STOW` (0), `SCORE` (table angle), `LOB` (25° — max kicker-feedable travel), `SHOT_BLOCK` (tunable, default 0.5 rot)
- `IndexerState`: `ZERO` (0 V), `SCORE` (−4.5 V)

### 4.3 Shot physics and tables

Three `InterpolatingDoubleTreeMap`s map **motion-compensated distance → pivot / RPS / time-of-flight**, derived from the minimum-energy lob into a 72" top-drop opening from a ~12" release (Δh = 1.524 m):

```
θ = 45° + ½·atan(Δh/d)        v² = g·(Δh + √(d² + Δh²))        RPS = v / (0.75·π·0.1016 m)
```

| Distance | Pivot (output rot) | RPS | TOF (s) |
|---|---|---|---|
| 1.5 m | 0.008 | 25.0 | 0.66 |
| 2.5 m | 0.027 | 27.6 | 0.77 |
| 3.5 m | 0.038 | 30.2 | 0.88 |
| 4.5 m | 0.045 | 32.8 | 0.99 |

The whole pivot curve is offset up ~3° because at dead stow the shooter rests on the kicker wheels, which must be able to spin to feed. The RPS values assume 75% wheel-to-ball transfer and no drag — they're a deliberate *baseline*, expected to be trimmed up ~10–25% on the field (cross-checked against team 1678's published tuned values, which follow the same +30% speed spread across the same distance band). Two dashboard offsets (`ShotTuning/PivotOffset`, `ShooterRpsOffset`) shift each entire table without redeploying.

### 4.4 Shoot-on-the-move

The drivetrain solves the required ball velocity vector: `(hub − robot)/TOF − robotFieldVelocity`, scaled back by TOF into a virtual target. Distance to that virtual target (clamped 0–6 m) drives the table lookups; its angle drives the aim heading. Robot *rotation* during ball flight is additionally compensated (gyro rate × TOF, clamped ±0.35 rad, tunable). `faceHubCommand` heading-locks the swerve to the computed heading (P = 8, continuous input) while the driver translates freely.

### 4.5 Shot lifecycle

```
prepare (set pivot + flywheel states)
  → waitUntil(readyToShoot).withTimeout(1.25 s)     ← fires either way; a shot can never hang
  → feed indexer (unconditionally)
finallyDo → stop flywheel, indexer, pivot           ← guaranteed cleanup on any interruption
```

`readyToShoot()` is an AND of four gates — in range (≤ 5 m), aimed (≤ 5°), flywheel at speed (± 3 RPS), pivot in position (± 0.04 rot) — each published individually (dashboard + log) so a blocked shot is diagnosable at a glance. The same command shape backs both driver (held) and autonomous (timed) shots.

### 4.6 Sensor-free shot detection

No beam break: a ball passing through the drum dips flywheel velocity. Once the wheel has reached within 2 RPS of target ("armed"), a dip > 5 RPS followed by recovery to within 2 RPS counts one shot. Exposed as `getShotCount()` and logged (`Shooter/ShotCount`, `Shooter/BallInFlywheel`). Thresholds are dashboard-tunable.

### 4.7 Protection

Software overcurrent supervision on pivot (default 40 A / 0.25 s) and indexer (25 A / 0.2 s): exceeding the limit for the window zeroes the motor and falls back to a safe state (pivot → STOW, indexer → ZERO), with a dashboard flag. Thresholds/timeouts tunable live.

### 4.8 On-robot characterization (SysId)

Four dashboard buttons (`SysId/Shooter …` quasistatic/dynamic × fwd/rev) run WPILib SysId voltage profiles on all four flywheel motors simultaneously, logging via CTRE SignalLogger to a `.hoot` file for Tuner X analysis (kS/kV/kA, in rotations to match Phoenix units). A `characterizing` flag suspends normal flywheel control while a routine owns the motors; `finallyDo` stops motors and the logger.

---

## 5. Lintake (Intake)

Pinion arms (motors 35 leader / 2 follower, brake mode, MotionMagic: kP 1.25, kD 0.15, kV 0.25, cruise 55 / accel 135 / jerk 1600) position between `STOW` (−4), `AGITATE` (−7.5), and `GROUND` (−9.75) rotations. A roller (motor 45, coast) runs at ±12 V (`INTAKE`/`EJECT`) or 0. The pinion setter exists in both Command form (for bindings/autos) and imperative form (for use inside other commands' lambdas).

---

## 6. Operator Interface (Xbox, port 0)

| Input | Action |
|---|---|
| Left stick / right stick X | Field-centric translate / rotate (6 m/s, 1 rot/s max) |
| **Right trigger (hold)** | Auto-aim at hub + full shot sequence |
| A (hold) | Aim at hub only |
| Left trigger / POV-left (hold) | Roller intake / eject (auto-stops on release) |
| LB / RB | Deploy / stow intake |
| X | Intake agitate |
| Y (hold) | Pivot to shot-block position; stow on release |
| POV-up / POV-right (hold) | Lob shot / long send shot (max pivot, fixed RPS) |
| POV-down | Re-seed field-centric heading |

Automatic behaviors: trench entry/exit (heading lock + stow + deploy), disabled-mode idle request on the drivetrain.

---

## 7. Autonomous

PathPlanner `AutoBuilder` (translation PID 10/0/0, rotation 7/0/0, 20 ms discretization, alliance-aware path flipping) with a dashboard auto chooser and pathfinding warmup. Named commands: `intake deploy/retract/run/stop`, `shoot` (timed 1.5 s feed with the same force-feed timeout), `stopshoot`.

Routines:

- **preload** — shoot the preloaded ball
- **testauto** — deploy, run intake, two sweep paths, shoot, stop intake
- **doubleswipe** — deploy, 0.5 s settle, intake, two sweep paths, then shoot in parallel with intake retract (raced against a 4 s cap), stop intake

---

## 8. Telemetry & Tuning Infrastructure

- **DogLog** (init in `Robot`): WPILOG file + live NetworkTables mirroring + DS/joystick capture. Per-loop shooter suite: distance, target/commanded/actual RPS, RPS error, pivot target/actual, all four readiness gates, currents, indexer volts, all three state enums, shot count.
- **`LoggedTunableNumber`**: every knob under `ShotTuning/` (table offsets, flywheel kP/kS/kV — re-applied over CAN only when changed via `hasChanged()` — shot-detection thresholds, overcurrent limits, shot-block position, TOF offset). A single `setTuningMode(false)` locks all values to compiled defaults for competition.
- **Recommended workflow:** SysId → paste kS/kV/kA → live-sweep kP against `Shooter/RpsError` until recovery rings, back off 20% → range test at 1.5/2.5/3.5/4.5 m trimming `ShooterRpsOffset`/`PivotOffset` → bake winners into `ShooterConstants`, lock tuning mode.

---

## 9. Known Limitations & Recommended Next Steps

1. **LOB/SEND fire via timeout, not readiness** — `readyToShoot()` compares against SCORE targets, which ferry shots never satisfy, so they always feed at the 1.25 s timeout. Acceptable, but comparing against *commanded* values would make ferry shots fire on spin-up.
2. **Unverified-on-hardware values** — pivot table *sign/zero*, the ~3° kicker-clearance floor, RPS baseline (expect +10–25%), and the shot-detection thresholds all need one field session to confirm. All are dashboard-trimmable.
3. **Shot-block position** assumes 0.5 rot ≈ 180° of travel — verify against the real mechanical range before holding Y near hard stops (overcurrent protection is the backstop).
4. **Limelight mount pose** lives only in the camera's web UI; consider pushing `setCameraPose_RobotSpace` from code at startup so it's version-controlled.
5. **Single camera, heuristic std-devs** — a second Limelight and/or measured std-dev tuning would tighten auto-aim at range.
6. **`kA` is hardcoded (0.2)** in the live-gain re-apply path; fold in the SysId value when known.
7. **Architecture (offseason):** IO-layer abstraction, a superstructure/state coordinator, and a targeting class that owns the shot math (currently split between `Shooter` and the drivetrain) — patterns proven in 1678/2910's public code — are the right next structural steps, deliberately deferred mid-season.

---

## 10. Recent Development (branch `summer`)

- `b754b4c` — shot-lifecycle fix (guaranteed stop), vision in auto, auto intake-stop, physics shot tables
- `776b5ed` — DogLog, force-feed timeout, current limits, flywheel SysId
- `b3c3c85` — LoggedTunableNumber, live gains, shot detection
- `70d1d15` — frozen-telemetry fix (status signals vs. bus optimization), lintake stow fix
- `c307f98` — LOB pivot to max travel, dead-code removal, dedup/readability pass
