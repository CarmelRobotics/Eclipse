# Competition vision and auto-aim (2026-10-28 rebuild)

## Status and limits

This replaces the previous `VisionHubRange` reader. Chassis alignment is **enabled by
explicit driver request**. The implementation is tested headlessly, not certified by
a physical field test. No simulation can verify camera mounting, tag-map correctness,
motor wiring, actual shot trajectories, lens occlusion, carpet grip, or battery condition.
Do the short acceptance procedure below before using automatic shots in a match.

## Driver controls

- **A alone:** aim the chassis; translation remains under driver control. Rumble means
  the relative target is aligned and the chassis is slow/level, not that wheels are spun up.
- **Right trigger:** acquire a target, align the chassis, command the calibrated hood/RPS,
  and feed only when target, alignment, motion, and mechanism checks pass.
- **Right stick while aiming:** immediately overrides automatic yaw. Vision-shot feed
  pauses while that override is active. Auto yaw is capped at 2 rad/s with a 6 rad/s² ramp.
- **Hold A before pulling right trigger:** explicitly select the fixed 3.52 m tower shot
  with manual yaw. Selection and trims latch until trigger release; this retains the
  previous manual readiness/1.25-second timeout behavior. It is not a safe preset for
  arbitrary locations. Release/re-pull to change mode.
- **POV up:** the existing alliance-referenced smart pass, requiring referenced field pose.
  **POV right:** the existing fixed 90 RPS manual ferry. **B:** clear jam. **Y:** shot block.
  An interrupted held command requires releasing and pulling its trigger again.

There is no silent switch to a pass or tower shot when vision fails. The driver is
responsible for choosing the correct shooting position and active hub. Teleop uses the
nearest hub in the camera's blue-origin frame, as previously requested; this is **not**
an alliance/legality check. Autonomous and passing targets still use DS alliance.

## Camera setup that must be verified on hardware

1. Device/NetworkTables name: **`limelight`**. Robot network/team: **9995**.
2. Use the correct **2026 REBUILT AprilTag map**, not a prior season or a red-origin map.
   The code's official tag geometry and hub centers use the **2026 welded** layout.
   If the event uses different field geometry, verify and update that configuration first.
3. Configure the camera's actual robot-space mounting translation and rotation in the
   Limelight web interface. Camera pose is not overwritten from the placeholder
   mounting numbers in `LimelightInfo`.
4. Enable AprilTag 3D localization so **`botpose_wpiblue`** is published. This rebuild
   deliberately uses MegaTag1, not `botpose_orb_wpiblue`/MegaTag2: it must not depend on
   an already-correct externally supplied gyro heading. Aim for at least 20 fresh pose
   frames/second and low latency. `tv` alone is not a usable 3D pose.
5. Verify the camera can see useful tags with a full hopper/net and while fuel is feeding.

## One frame pipeline

`VisionSystem` subscribes once per configured camera to an atomic blue-origin MegaTag1
packet. Header, pose, latency, tag count, and embedded tag quality are decoded together
by `VisionObservation`. No separate `tid`, `tv`, `rawfiducials`, or target-space topic is
required. **Any valid field tag can supply a pose**, not only a hub tag.

Duplicate values with genuinely new packet timestamps are accepted. A repeated read
of one image does not count as a new image. A camera heartbeat cannot refresh an old
pose. There is no JSON parsing, listener-per-frame allocation, or unbounded frame history;
subscriber storage and latest observations are bounded by the number of cameras.

Validation includes finite numbers, packet shape/counts, capture time/future limits,
latency, physical field/ground-plane bounds, tag area/span, known IDs, duplicate IDs,
ambiguity, and agreement between reported robot pose and embedded robot-to-tag ranges.
Detected map/origin/range inconsistencies produce a visible rejection, not guessed reflection.
These checks cannot prove the camera's map or mounting calibration is correct.
Single-tag solves require embedded quality. Older standard 11-value multi-tag packets
are accepted only with sufficient tag span and valid summary quality.

The same decoded observation feeds the odometry guard and the teleop aim tracker.
Field odometry remains blue-origin and retains its existing stationary acquisition,
enabled-jump rejection, and gyro-owned heading. Vision does not reset gyro heading.
Field-centric driving/auto/pass still need a valid heading reference (PathPlanner's full
pose reset or the deliberately placed D-pad-down seed); **teleop aiming does not**.
Do not press D-pad down unless robot +X/front physically points downfield.

## Target tracking and feed checks

`VisionAimTracker` turns the selected hub's displacement into robot-relative coordinates
using the camera pose. It does not use the fused/global robot pose to aim. An incorrect
Elastic/fused pose cannot by itself change teleop shot range or turn direction. A fully
consistent 180-degree frame transformation preserves nearest-hub relative geometry;
contradictory target jumps are rejected even if their ranges look plausible. A stable
but incorrectly calibrated camera yaw still requires the physical acceptance test.

Three distinct, consistent images acquire a target. Capture latency is compensated
using bounded relative robot motion. Short camera interruptions use only relative
measured robot-relative chassis motion for **at most 0.30 s from the last accepted image capture**. Then auto
yaw and vision feeding stop. The last hood/RPS targets remain held while the trigger
is held; acquisition uses low-speed idle pre-spin, never an invented scoring distance.

Three contradictory target jumps latch a fault to prevent an unexplained half-turn.
Release the aim/shoot control and re-press to re-arm and acquire a new consistent target.
Bad motion data or a long scheduler gap invalidates tracking. Trims are finite/bounded
and snapshot when the corresponding aiming/shooting command starts.

Vision feed requires:

- A live/acquired target in the calibrated **1.5-5 m** range.
- Heading error within **5 degrees**, no right-stick yaw override.
- Translation at most **0.75 m/s**, rotation at most **0.5 rad/s**, pitch/roll within **5 degrees**.
  This version intentionally uses stationary/slow shots, not unvalidated moving-shot ballistics.
- All four flywheels within **3 RPS** of target, with valid/recent encoder data.
- Both pivot sides within **0.006 output rotations (2.16 degrees)** of target, with valid/recent data.

The spin-up timeout never overrides these vision-shot checks. Transient pauses retain
the jam retry budget and pause its no-shot timer; they do not silently reset recovery
attempts. Release/cancellation stops feeding and clears the shot mode. Autonomous path
following, auto timing/strategy, drive gearing/current limits, and the calibrated shot
table are not redesigned by this change.

## Dashboard / bounded logs

- `Vision/Camera`, `Vision/InputStatus`, `Vision/TagCount`: camera configuration and raw-frame rejection.
- `Vision/AimStatus`, `Vision/TargetAgeS`, `Vision/RobotRelativeBearingDeg`: acquisition/holdover/aim state.
- `Vision/DriverOverride`: whether the right stick is taking control.
- `Shooter/ControlMode`, **`Shooter/ShotBlockReason`**: current mode and exact feed blocker.
- `Shooter/VisionRange/Valid`, `Status`, `DistanceM`, `FrameAgeS`: compatible shot diagnostics.
- `Shooter/ShotWheelErrorRps`, `ShotPivotErrorRot`, `FeedPaused`, `ready/all vision mechanisms`.

Dashboard and selected WPILib file entries remain decimated to 10 Hz. DogLog, blanket
NetworkTables logging, and JSON image dumps remain absent. RIO 1 native/OS memory and
physical CAN/loop timing must still be checked on the actual robot.

## Required hardware acceptance (do not skip)

1. **Disabled, full hopper:** show a known tag; verify camera name/map, sensible raw X/Y/yaw,
   tag count, low capture age, and range within roughly 0.1-0.2 m of a tape measurement.
   Repeat from a second location and a different tag group. A visible tag with missing 3D
   output must say why; do not force an unverified pose into the estimator.
2. **No fuel, safe clear area:** begin 10-15 degrees off target. Hold A. The chassis must
   turn toward the hub, settle, and yield immediately to right stick. Repeat on both
   alliances. If it turns away or shows a roughly 180-degree bearing error, stop and
   fix camera/front-axis calibration; do not guess-invert gyro or alliance coordinates.
3. **Stationary with fuel:** right trigger must spin all wheels, settle both pivot sides,
   then feed. Verify actual wheel/pivot feedback and shot landing at two or three ranges.
   Existing shot-table entries and mechanical zero are not substitutes for real shots.
4. **While aiming:** cover/unplug the camera. Auto yaw/feed must stop after the bounded
   holdover. Uncover and verify re-acquisition. Confirm A+trigger at the tower still works
   without tags, release stops feed, and B/Y interrupts behave as the driver expects.
5. Run a full practice match on the robot and inspect `Shooter/ShotBlockReason`, CAN
   faults, battery voltage, loop overruns, target age and memory. Do not select a full-speed
   auto until its physical start direction is separately verified; this is not a fix for
   the previously reported backwards autonomous launch.

## Automated verification versus field validation

Tests include actual controller commands/Phoenix requests with synthetic encoder feedback,
both alliances, any field-tag IDs, blue-origin-only IO, real NT4 loopback transport,
duplicate timestamps/unchanged values, stale heartbeat/pose, non-finite/malformed data,
field-map inconsistency, mirrored fused odometry, conflicting yaw, latency, bounded
holdover, right-stick override, failed follower wheels, interruption, and explicit manual
fallback. Long streams exercise 20,000 camera packets and 15,000 target-tracker updates.
These tests do **not** simulate ball flight, camera optics, motor physics under fuel load,
radio traffic, or the roboRIO's whole native/OS memory footprint.

Primary API references: [Limelight MegaTag1](https://docs.limelightvision.io/docs/docs-limelight/pipeline-apriltag/apriltag-robot-localization),
[Limelight packet format](https://docs.limelightvision.io/docs/docs-limelight/apis/complete-networktables-api),
[WPILib NT timestamps](https://docs.wpilib.org/en/stable/docs/software/networktables/networktables-intro.html),
[WPILib publish/subscribe options](https://docs.wpilib.org/en/stable/docs/software/networktables/publish-and-subscribe.html).
