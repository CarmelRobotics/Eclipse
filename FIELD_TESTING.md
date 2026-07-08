# Field Test Checklist

Ordered so each step de-risks the next — don't skip ahead: a bad pose makes the
shot sweep garbage, a dead motor makes everything garbage. Check boxes as you go;
write numbers down IN THIS DOC (or a photo of it) so tuning survives the drive home.

> **Setup facts baked into the code right now:**
> - Tuning mode is ON (`LoggedTunableNumber.setTuningMode(true)` in Robot.java) — all
>   `ShotTuning/*` dashboard values are live.
> - Without FMS, aiming targets the **closest** hub, but the hub-shot **zone gate**
>   uses the **Driver Station alliance**. Set the DS alliance to the side you're
>   physically on or right-trigger will always give you PASS mode.
> - Hub shots require BOTH: distance ≤ 5 m AND robot on the wall side of its hub's X
>   (blue x ≤ 4.625, red x ≥ 11.925). Watch `aim/in alliance zone` on the dashboard.

---

## 0. Before leaving the shop

- [ ] Latest `summer` deployed to the robot (`6c65d44` or later) — includes zone gate,
      systems check, motor test, and the review fixes
- [ ] Laptop has: Driver Station, Elastic/Glass, AdvantageScope, PathPlanner
- [ ] Batteries charged (bring several — shot sweep + autos eat charge; idle spin
      auto-sheds below 7.0 V which skews spin-up feel on a dying battery)
- [ ] Blocks/cart for the motor test
- [ ] Tape measure (shot sweep distances) + floor tape to mark firing spots
- [ ] **Balls out of the robot** before the motor test

## 1. Power-on & health (robot on cart, DISABLED)

- [ ] Robot boots, DS connects, correct battery voltage shown (>12.5 V fresh)
- [ ] **Set DS alliance + station to match the side you're practicing on**
- [ ] Check `Faults/*` on the dashboard — everything true/green (fault monitor runs
      continuously; a red one here = CAN/power problem, fix before anything else)
- [ ] Enable **Test** mode briefly — static systems check auto-runs.
      `SystemsCheck/Result` = PASS. If FAIL, `SystemsCheck/Report` lists exactly
      which device/camera/gyro is missing. (Also re-runnable: `SystemsCheck/Run`.)

## 2. Active motor test (ON BLOCKS, wheels free, no balls)

- [ ] Robot on blocks, area clear
- [ ] Enable Teleop → click `SystemsCheck/RunMotorTest`
- [ ] ~9 s: flywheels → indexer → pivot → roller → pinion → steer → **drive (wheels spin)**
- [ ] `SystemsCheck/MotorTest/Result` = PASS. If FAIL, per-motor flags under
      `SystemsCheck/Shooter|Lintake|Drive/*` name the culprit.
- [ ] Clicked while disabled it says SKIPPED (that's it working, not broken)

## 3. Vision & pose sanity (on the floor)

- [ ] Place robot at a spot you can measure (e.g., against a wall at a known tag).
      Field2d pose matches reality (x, y, heading)
- [ ] Drive a slow lap: pose tracks smoothly, no teleporting.
      `vision/pose resets` should NOT climb steadily while tags are in view
      (occasional +1 after being carried/lost is fine — steady climbing means it's
      snapping every frame: check camera mount/config)
- [ ] `vision/tag count` ≥ 1 when facing tags; `hub visible` true
- [ ] **`vision/heading seeded` goes true** after you face a **two-tag** view — this is the
      gyro getting field-aligned from vision. Aim trusts the gyro heading, so if this is
      still false, aiming is running on an unseeded/possibly-wrong heading. It re-aligns
      automatically if the gyro ever diverges >10° from a multi-tag read (bump recovery)
- [ ] **Cover the Limelight** and drive ~5 s: pose keeps moving on odometry (slow
      drift OK, freeze/jump = problem). Uncover: pose corrects smoothly, no violent snap
- [ ] Spin in place 2–3 turns, drive back to the measured spot: pose still ≈ truth

## 4. Aim & zone gate sanity (no balls yet)

- [ ] Stand inside your zone, ~2.5 m from hub. Hold **A**: robot heading-locks onto
      the hub, no oscillation, settles < 1 s. Strafe left/right while holding A —
      heading keeps tracking the hub
- [ ] `aim/auto mode` reads HUB inside the zone within 5 m; drive past your hub's
      X-line → flips to PASS; `aim/in alliance zone` flips false
      (mode also requires `our hub active` — always true in ordinary teleop driving;
      in a real match RT auto-ferries during the opponent's SHIFT)
- [ ] Optional SHIFTS rehearsal: run **DS Practice mode** (set teleop length to 140 s
      in the DS setup tab — real REBUILT teleop is 2:20) — the shifts cycle
      automatically: 10 s all-active, four 25 s shifts (`our hub active` swaps, RT
      flips HUB↔PASS with it), 30 s all-active endgame. Synthesized order is
      blue-inactive-first; type `R` or `B` in the DS Game Data box to choose
- [ ] `dist to hub` vs tape measure at your marked spot: within ~0.2 m
      (if this is wrong, DO NOT tune shots — fix pose first, everything depends on it)

## 5. Shot table sweep (THE BIG ONE — budget the most time)

Setup: tape marks at 1.5 / 2.5 / 3.5 / 4.5 / 5.0 m from hub center. ~5 balls per
mark. Hold RT until the shot fires (rumble = locked). Watch where balls land.

Every detected shot auto-logs its full solution: glance at the **`last shot`**
dashboard string right after each ball (distance, RPS, pivot, heading error, and
the trims in effect) — note where it landed next to that. The same snapshots are
in the WPILOG under `Shooter/ShotEvent/*` (step through `Number` in
AdvantageScope), so the calibration data survives even if the table below doesn't
get filled in.

**Current table (what the code will do):**

| Dist (m) | Pivot (rot) | RPS  | Landed? (short/in/long) | Offset used |
|----------|-------------|------|--------------------------|-------------|
| 1.5      | 0.046       | 39.0 |                          |             |
| 2.5      | 0.065       | 41.5 |                          |             |
| 3.5      | 0.076       | 45.5 |                          |             |
| 4.5      | 0.083       | 49.0 |                          |             |
| 5.0      | 0.086       | 51.0 |                          |             |

**Live trims (dashboard, take effect immediately):**
- Falling short / long everywhere → `ShotTuning/ShooterRpsOffset` (+2 RPS ≈ noticeably deeper)
- Short only at range → the drag scaling needs help; note it, we re-slope the table at home
- Left/right bias → `ShotTuning/HeadingOffsetDeg` (1° steps; if it moves the wrong way, flip sign)
- Arc too flat/steep → `ShotTuning/PivotOffset` (±0.005 rot steps)
- [ ] **Write final offset values in the table above** — they do NOT survive a
      reboot; we bake them into ShooterConstants afterward

**Shoot-on-the-move** (after stationary is dialed):
- [ ] Strafe at ~30–50% while holding RT from 2.5 m: shots still land.
      Too much lead → lower `ShotTuning/MoveCompGain` (0.7 default, 0 = no lead)
- [ ] Translation feels controllable during aim? If not: `ShotTuning/ShootingSpeedScale`
      (0.45 default)

## 6. Pass / ferry

- [ ] From past the zone line, hold RT (auto-PASS) or POV-up (forced): robot faces
      the correct corner (`pass/target x/y` sane, side flips with field half)
- [ ] Passes launch flatter now (`ShotTuning/PassPivotRot` = 0.086, feeds fine) so
      balls should ROLL OUT instead of checking up and bouncing back. Still bouncing
      back → flatter: raise `PassPivotRot` in +0.005 steps. Landing short →
      `ShotTuning/PassRpsOffset` up (flatter carry needs more speed than the
      25°-modeled table; expect to add a few RPS). Rolling THROUGH the zone → too
      flat: back off `PassPivotRot`
- [ ] Bounce-out of the corner → also aim deeper: lower `ShotTuning/PassCornerInsetM`
      (1.0 now)
- [ ] Write down final `PassPivotRot` + `PassRpsOffset` to bake in

## 7. Driver assists & utilities

- [ ] Drive into a trench corridor: `assist/zone` = TRENCH_IN, heading snaps to
      nearest 90°, translation rails to the corridor axis, shooter stows + intake
      deploys; exit → intake restows
- [ ] Near-zone (halo): heading snaps only while rotation stick is centered;
      any rotation input returns full control
- [ ] Tower zone same behavior
- [ ] **X-lock (X):** wheels form X, robot resists a shove; release → normal driving;
      if used inside the trench, assist resumes on release
- [ ] **Clear jam (B):** flywheel + indexer reverse while held, stop on release
- [ ] Rumble: steady buzz exactly while `ready to shoot` is true, clears on release
- [ ] If a shot ever hangs, read the `ready/*` booleans — the false one is the reason

## 7.5 Lintake pinion tuning (optional, on blocks or intake clear)

The pinion arms are MotionMagic position-controlled; gains are live under `LintakeTuning/*`
(defaults = current compiled values, so nothing changes until you touch a knob). Tune against
`LintakeTuning/PinionErrorRot` (target − actual, motor rot) — want it to reach ~0 fast, no
overshoot, no ringing. **Key constraint:** the 15 A stator limit caps pinion torque (that's
the compliance), so a too-aggressive profile just saturates the limit and the arm lags — fix
that with the profile, not by cranking kP.

- [ ] Feedforward first — set `PinionKp` and `PinionKd` to 0 temporarily
- [ ] `PinionKs`: raise from 0 until the arm just barely creeps on the lightest command
      (kS=0 today is physically wrong — every mechanism needs a static term; expect ~0.1–0.2)
- [ ] `PinionKv`: adjust so error stays small *during* a move (arm tracks the profile).
      Current 0.25 is ~2× a Kraken's textbook rotor kV (~0.12) — plausible under load but
      unverified; SysId would ground it
- [ ] Restore `PinionKp` (start 1.25), raise until it reaches target crisply; if it
      overshoots/rings, add `PinionKd`
- [ ] Profile: raise `PinionCruiseRps` / `PinionAccelRps2` for a faster deploy until the arm
      can't keep up (error grows mid-move = hitting the current limit) or slams the stop; back off
- [ ] Verify STOW ↔ GROUND ↔ AGITATE all land clean and hold
- [ ] Bake winners into `LintakeConstants.PinionConfigs`, zero the dashboard knobs, done

## 8. Autonomous (clear the field of people)

Run in this order; be ready to disable. Robot placed at the path's start pose
(the autos reset odometry to path start, vision then corrects — big placement
error = wasted first seconds):

- [ ] `2910_trench_left` — collects from trench run, returns, dumps. Watch: does the
      collect leg actually drive through fuel? Does the dump land in the hub?
- [ ] `2910_trench_right` — same, mirrored
- [ ] `2910_trench_left_double` — both cycles complete inside auto period?
      Note where the timer runs out; we retune path speeds if it's close
- [ ] `2910_trench_right_double`
- [ ] Note actual score poses vs. the shortened paths (today's path edits moved the
      score waypoint to (2.72, 5.50)) — confirm dump from there actually scores

## 8.5 Feature flags & power manager (dashboard)

Under `FeatureFlags/*` — flip any OFF to disable a behavior live, no redeploy. Your field
escape hatches:
- [ ] Confirm all default ON except `PowerManager` (default OFF). Toggling should visibly
      change behavior (e.g. `VisionFusion` off → `vision/pose resets` stops climbing and
      pose rides odometry; `ShootOnTheMove` off → aim stops leading while strafing)
- [ ] `VisionFusion` — off if the camera feeds garbage; robot runs on odometry
- [ ] `ShiftsGate` / `AllianceZoneGate` — off to force HUB mode anywhere (bad game data, or
      bench-shooting from outside the zone)

**PowerManager** (dynamic drive-vs-flywheel current budgeting) starts OFF because it
re-allocates current limits at runtime and hasn't met the field. To validate:
- [ ] Baseline first with it OFF (static limits — the known-good config)
- [ ] Flip `FeatureFlags/PowerManager` ON. Watch `PowerManager/Applied` change with activity:
      IDLE parked → SCORING while aiming → SPRINT on full-stick drive → AUTO in auto
- [ ] Full-power shot-while-driving: does the battery hold better than baseline? Watch
      `Battery/Voltage` / `Battery/BrownedOut`. If a pushing match feels weaker, that's
      SCORING/IDLE capping drive to 25/40 A — expected; the budgets live in `PowerMode`
- [ ] If anything feels off, flip it back OFF → instantly restores static limits (IDLE)

## 9. Failure drills (5 minutes, worth it)

- [ ] Cover Limelight mid-driving → drive + aim still usable on odometry for ~10 s
- [ ] Unplug controller while enabled, replug → bindings all work again
- [ ] Run a nearly-dead battery: robot stays drivable during full-power moves
      (idle spin shedding + brownout floor at 6.0 V doing their jobs); check
      `Battery/BrownedOut` in the log afterwards
- [ ] Intentionally jam a ball (if you dare): overcurrent trips (`pivot/indexer
      overcurrent tripped`), B clears it, robot recovers without reboot

## 10. Before you leave

- [ ] Final offset values written down (table in §5 + pass trims)
- [ ] Pull the WPILOG files (USB/AdvantageScope) — they contain every
      `Shooter/*` + `Battery/*` trace for post-mortem
- [ ] Note every "that felt weird" moment, however small
- [ ] Back home: bake offsets into `ShooterConstants`, zero the dashboard offsets,
      re-deploy, and set `LoggedTunableNumber.setTuningMode(false)` before comp

---

## Appendix A — Controls

| Input | Action |
|---|---|
| Left stick | Field-centric translate |
| Right stick | Rotate |
| RT (hold) | Context shot: HUB if in-range + in-zone, else PASS (latches at pull) |
| A (hold) | Aim at hub (heading lock only, no shot) |
| POV-up (hold) | Forced pass to corner |
| POV-right (hold) | Manual flat ferry (fixed 90 RPS) |
| LT (hold) | Run intake rollers |
| POV-left (hold) | Eject rollers |
| LB / RB | Intake deploy / stow |
| X (hold) | X-lock brake |
| Y (hold) | Pivot to shot-block + stow intake |
| B (hold) | Clear jam (reverse flywheel + indexer) |
| POV-down | Re-seed field-centric heading |
| Rumble | Shot/pass is locked and ready |

Agitate is automatic during shots (pinion bounces AGITATE↔GROUND while feeding).

## Appendix B — Dashboard quick reference

| Key | Healthy looks like |
|---|---|
| `SystemsCheck/Result`, `MotorTest/Result` | PASS |
| `Faults/*` | all true |
| `vision/pose resets` | flat while tags visible |
| `dist to hub` | matches tape measure |
| `aim/auto mode`, `aim/in alliance zone` | HUB + true inside zone |
| `our hub active` | true in practice; alternates during match SHIFTS |
| `last shot` | full solution of the most recent shot (see §5) |
| `ready/shot in range·aimed·spun up·pivot` | the shot-hang diagnosis set |
| `assist/zone` | NONE / *_NEAR / *_IN as you move |
| `ShotTuning/*` | live trims (see §5/§6) |
| `Battery/Voltage`, `Battery/BrownedOut` | in AdvantageScope logs |
