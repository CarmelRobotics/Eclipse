# Teleop localization / wrong-hub hotfix

All poses remain in **blue-origin field coordinates on both alliances**. Do not change
the Limelight API to `wpired` or reflect odometry just because the DS reports Red.

## Current competition control: teleop auto-alignment disabled

`Constants.kTeleopAutoAlignEnabled` is compiled **false**. A, right trigger, and POV-up
do not rotate the chassis toward a hub or pass target; yaw remains on the driver's
right stick. Dashboard edits cannot re-enable it. Trench/tower assists remain disabled.

Right trigger now always uses **TOWER PRESET**, independent of pose, alliance, tags,
hub-active shifts, or range. The driver must aim and choose the shooting location/time;
right trigger no longer automatically changes to a pass. POV-up smart pass and POV-right
manual ferry remain separate controls. Mechanical readiness, the existing 1.25-second
spin-up timeout, feed recovery, and release cleanup remain.

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
   remains disabled regardless of this value. Right trigger stays on the fixed tower
   preset even after initialization; these steps do not re-enable automatic ranging.

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
  trigger is independent of this gate:** it always selects the fixed 3.52 m tower
  preset, preserving manual joystick rotation. The driver must aim and shoot from
  the corresponding approximate range.
- `Shooter/ControlMode` explicitly shows TOWER PRESET; `aim/auto mode` also reports
  TOWER PRESET regardless of localization. A tower shot starts spinning immediately,
  then follows the existing readiness-or-1.25-second-timeout feed/recovery sequence.
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
and the same tower preset on a later localized, out-of-range press. It also verifies
old dashboard range values are ignored, switching to autonomous cancels the teleop shot,
and A, right trigger, and POV-up preserve joystick yaw without updating a heading-lock target. The localization
integration test verifies autonomous heading requests still work on both alliances.
No desktop, robot, or camera connection is started.

**This code cannot repair a wrong field map, physically swapped tag IDs, camera
mounting calibration, or a different/stale deployed binary.** If the camera keeps
reporting the opposite starting half, **automatic** aiming stays blocked with an explicit map/tag
warning; it does not guess a reflection. The uploaded Limelight field map must match
the actual event, hostname must be `limelight-four`, and networking must match team
9995. No camera settings or robot deployment were performed by this change.
