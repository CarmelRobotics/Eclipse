# Teleop localization / wrong-hub hotfix

All poses remain in **blue-origin field coordinates on both alliances**. Do not change
the Limelight API to `wpired` or reflect odometry just because the DS reports Red.

## Driver procedure when no auto runs

1. Once placed, point the robot's **+X/front away from your alliance wall**, toward
   midfield. The rear intake is **not** the front for this procedure.
2. Press **D-pad down**, while disabled or in teleop. This now seeds the absolute
   field heading (Blue 0 degrees / Red 180 degrees), independent of our deliberately
   reversed driver-stick perspective. Never press it in an arbitrary orientation.
3. Valid stationary camera frames establish X/Y without an auto. Verify
   `Localization/Ready to aim` is true for automatic aiming. AUTO/MANUAL selection
   latches per right-trigger press; release and press again after initialization to
   switch out of the manual fallback.

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
- Until position, heading, and alliance are initialized, automatic aiming and
  position-dependent shooting remain blocked, including the auto spin-up timeout
  path. **Right trigger does not become a dead button:** it selects a manually aimed
  fixed-range hub shot, preserving manual joystick rotation. Default range is **2.75 m**
  (existing shot table, not a new ballistic model). `ShotTuning/ManualShotDistanceM`
  can be set from 1.5 to 5 m **before** pressing the trigger. Range and existing pivot/RPS
  trims snapshot at the press and stay unchanged for that hold. This is not automatic
  range estimation: the driver must aim and shoot from the selected approximate range.
- `Shooter/ControlMode` explicitly shows AUTO HUB/PASS versus MANUAL PRESET; `aim/auto mode`
  also reports MANUAL PRESET when unlocalized. A manual shot starts spinning immediately,
  then follows the existing readiness-or-1.25-second-timeout feed/recovery sequence.
  Release stops feed and clears manual targets; the normal low-speed idle pre-spin may
  remain. Ordinary driving/intake and the separate manual ferry are unchanged.
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
Xbox trigger in teleop with no localization, and checks the native flywheel velocity
request, indexer feed, release cleanup, fixed-preset latching, and automatic selection
on a subsequent localized press. No desktop, robot, or camera connection is started.

**This code cannot repair a wrong field map, physically swapped tag IDs, camera
mounting calibration, or a different/stale deployed binary.** If the camera keeps
reporting the opposite starting half, **automatic** aiming stays blocked with an explicit map/tag
warning; it does not guess a reflection. The uploaded Limelight field map must match
the actual event, hostname must be `limelight-four`, and networking must match team
9995. No camera settings or robot deployment were performed by this change.
