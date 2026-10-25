# Teleop localization / wrong-hub hotfix

All poses remain in **blue-origin field coordinates on both alliances**. Do not change
the Limelight API to `wpired` or reflect odometry just because the DS reports Red.

## Current competition control: teleop auto-alignment disabled

`Constants.kTeleopAutoAlignEnabled` is compiled **false**. A, right trigger, and POV-up
do not rotate the chassis toward a hub or pass target; yaw remains on the driver's
right stick. Dashboard edits cannot re-enable it. Trench/tower assists remain disabled.

Right trigger uses **VISION RANGE** for hood/RPS only; the driver retains chassis yaw.
Hold **A before pulling right trigger** to explicitly select **TOWER PRESET** instead.
This selection latches until trigger release. Vision never silently changes into a tower
shot or a pass. POV-up smart pass and POV-right manual ferry remain separate controls.

### Vision-only teleop range

`VisionHubRange` reads `botpose_targetspace` plus the primary hub tag ID, raw ambiguity,
target validity, and capture/pipeline latency. Only actual hub tags are eligible. It
converts official tag-to-hub-center offsets into Limelight target space, including
the lateral offset of non-centered hub tags, then measures horizontal center range.
It uses neither robot field X/Y nor field heading nor FMS alliance selection. A mirrored
global pose therefore cannot change this range. Of valid camera observations, the
closest observed hub range is selected. The configured camera name is **limelight**.

Range must be 1.5-5 m, ambiguity at most 0.2, and frame age at most 0.25 s; mismatched
pose/quality packets, bad numbers and large isolated jumps are rejected. New frames are
low-pass filtered. Existing pivot/RPS trims latch on the trigger press; no odometry,
shoot-on-the-move, trajectory-map, gyro or tilt compensation enters the vision targets.
The camera's robot-space mounting calibration and the event's tag IDs must be correct.

**No valid hub tag = no vision-shot feeding**, even after the existing spin-up timeout.
Feeding also pauses if the flywheel/pivot are not ready as range changes. After losing
vision, the last hood/wheel targets are held while the trigger is held, but feeding stops
when range becomes invalid. New valid vision resumes range updates and feeding only
after mechanical readiness. Dashboard `Shooter/VisionRange/Valid`, `DistanceM`, and
`Shooter/ControlMode` indicate the source/status. There is no automatic tower fallback:
release, hold A, then re-pull the trigger **from the tower shooting position**.
Without visible hub tags the camera-relative mode cannot range using tower/trench tags.

### Fixed tower-shot starting point

The [official FIRST field drawings](https://firstfrc.blob.core.windows.net/frc2026/FieldAssets/2026-field-dimension-dwgs.pdf),
FE-2026 sheet 3 (welded perimeter), show **115.05 in from the field-facing tower uprights
to the near hub face**. Add half the 47 in hub depth to target its center:
`(115.05 + 47 / 2) * 0.0254 = 3.51917 m`, rounded to **3.52 m**.
This assumes the shooter is beside the upright line, not against the alliance wall or
at the forward edge of the tower base. The robot's actual release point changes range.

The existing shot table at 3.52 m gives **0.07614 pivot output rotations (27.4104 degrees
from the mechanism's zero, not a measured ball launch angle)** and **46.55 RPS (2793 RPM)**.
`ShooterConstants.kTowerShotDistanceMeters` fixes the base range in code. Old dashboard
`ShotTuning/ManualShotDistanceM` values are ignored. Existing `ShotTuning/PivotOffset`
and `ShotTuning/ShooterRpsOffset` trims snapshot on each trigger press; neither dashboard
edits nor odometry/motion/tilt compensation alter the targets during that hold. Release
and re-press to pick up new trims. `Shooter/TowerShot/TargetPivotRot` and `TargetRps`
show the effective latched targets. This is an **unverified preset based on the existing
table**, not a guarantee that shots from every position near the tower will score.

**Autonomous path-following rotation, startup alignment, and autonomous shooting heading
control are unchanged. This does not resolve the reported backwards autonomous launch.**

## Optional pose-based range setup when no auto runs

1. Once placed, point the robot's **+X/front away from your alliance wall**, toward
   midfield. The rear intake is **not** the front for this procedure.
2. Press **D-pad down**, while disabled or in teleop. This now seeds the absolute
   field heading (Blue 0 degrees / Red 180 degrees), independent of our deliberately
   reversed driver-stick perspective. Never press it in an arbitrary orientation.
3. Valid stationary camera frames establish X/Y without an auto. Verify
   `Localization/Ready to aim` is true for trusted field calculations. Teleop auto-align
   remains disabled regardless of this value. Right trigger's vision range does not
   depend on this initialization; these steps do not re-enable chassis auto-align.

A PathPlanner full-pose reset initializes X/Y and heading automatically; do not apply
this downfield reset during/after an auto unless physically pointing downfield.

## Software behavior

- A missing alliance packet retains the last actual DS assignment; Unknown is not
  treated as a trustworthy Blue assignment. Hub selection no longer changes to the
  nearest hub when FMS is absent or disconnects.
- Startup/staging rejects a vision seed on the opposite alliance half when FMS has
  identified the starting alliance. This placement assumption applies to a new-match
  seed only, **not** to crossing midfield during play. A mid-match reboot that did not
  observe staging does not infer the robot's current half from its alliance.
- Vision rejects stale/future frames, non-finite/out-of-field poses, poor single-tag
  solutions, and duplicate frames. Acquiring/relocalizing position needs four distinct
  stationary frames spanning at least 0.10 seconds and agreeing within 0.15 m.
- Once position is initialized, an enabled vision correction exceeding 1 m is rejected
  rather than teleporting the pose. Large corrections can be acquired while stationary
  and disabled. Normal nearby measurements still fuse; losing tags retains odometry.
- Vision initialization changes X/Y only. It does not silently trust camera yaw.
- Until position, heading, and alliance are initialized, position-dependent automatic
  shooting remains blocked, including the autonomous spin-up timeout path. **Right
  trigger is independent of this gate:** it uses vision-only relative hub range;
  A+trigger explicitly selects the 3.52 m tower preset. Both preserve manual yaw.
- `Shooter/ControlMode` explicitly shows VISION RANGE, VISION LOST, or TOWER PRESET;
  `aim/auto mode` describes the vision/tower controls, not pose-dependent hub/pass
  selection. The tower fallback retains the existing readiness-or-1.25-second-timeout
  feed/recovery sequence. Vision shots cannot bypass invalid range or mechanical readiness.
  Release stops feed and clears manual targets; the normal low-speed idle pre-spin may
  remain. Ordinary driving/intake and the separate pass/ferry controls are unchanged.
- `Sim/ApplyPose` cannot move a real robot's estimate. `active hub alliance` now displays
  Both/Neither where applicable, rather than mislabeling both-active periods as Blue.

## Diagnostics and limits

`Localization/*` reports alliance, FMS connection, position/heading initialization,
aim readiness, and the vision rejection reason. Raw vision X/Y, estimated X/Y/heading,
frame age, alliance, FMS, and readiness are logged at the existing bounded 10 Hz rate.

Headless tests inject FMS Red with no auto, mirrored and valid camera positions,
heading resets, missing alliance packets, and tag loss into the production drivetrain
with Phoenix simulation. They also verify stationary aim-request bearings on both
alliances. Existing auto/path tests remain enabled. These are synthetic camera inputs,
not validation of the physical Limelight or event field.

The right-trigger regression test constructs the actual RobotContainer, simulates an
Xbox trigger on both alliances with no localization, and checks native flywheel velocity
and pivot position requests, indexer feed, release cleanup, fixed-preset/trim latching,
and the same explicit tower preset on a later localized, out-of-range press. Vision tests
verify relative ranging without initialized field pose, mirrored pose independence,
live filtered updates, positive feeding with injected at-setpoint motor feedback, and
no feeding with missing/stale vision even past the spin-up timeout. Pure geometry tests
cover all 16 hub tags, including lateral offsets, and reject non-hub/poor-quality data.
Tests also verify old dashboard range values are ignored and autonomous cancels the teleop shot,
and A, right trigger, and POV-up preserve joystick yaw without updating a heading-lock target. The localization
integration test verifies autonomous heading requests still work on both alliances.
No desktop, robot, or camera connection is started.

**This code cannot repair a wrong field map, physically swapped tag IDs, camera
mounting calibration, or a different/stale deployed binary.** If the camera keeps
reporting the opposite starting half, **automatic** aiming stays blocked with an explicit map/tag
warning; it does not guess a reflection. The uploaded Limelight field map must match
the actual event, hostname must be `limelight`, and networking must match team
9995. No camera settings or robot deployment were performed by this change.
