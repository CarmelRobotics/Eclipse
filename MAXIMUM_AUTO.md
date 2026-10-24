# Maximum-aggression pickup option — 2026-10-23

New PathPlanner chooser names: `eclipse_max_left_double` and
`eclipse_max_right_double`. The existing four competition autos and their deployed
paths are unchanged. No deployment, commit, push, or desktop control was performed.

This is an opt-in, pickup-heavy variant, not proof of the physically fastest or
highest-scoring autonomous routine. It trades second-volley shooting time for
more fresh collection area. Actual fuel density, hopper capacity, and collection
throughput determine whether that trade scores more; they are not simulated.

## Placement and route

PathPlanner blue-origin coordinates and global headings, meters/degrees:

| Start | X | Y | Heading |
| --- | ---: | ---: | ---: |
| Blue left | 4.4 | 7.5453 | +90 |
| Blue right | 4.4 | 0.5247 | -90 |
| Red left | 12.14 | 0.5247 | -90 |
| Red right | 12.14 | 7.5453 | +90 |

Same physical starting placement as the existing sideways-start doubles. Intake
is on the rear/-X side, 9 inches beyond the assumed 0.90 m square bumpers.
Right reflects Y about 8.07 m and negates headings; red uses one PathPlanner
rotational flip. Neither side gets a second manual alliance flip.

Blue LEFT route; RIGHT is the exact reflection:

1. Verify shooter stowed and starting heading settled, with roller off. Launch
   through the trench at Y=7.5453, via (6.3,7.5453), toward (8.45,6.95).
2. Once clear of the trench and rear intake aligned, enable the roller. Sweep
   down to (8.45,6.15), then take a rear-leading clockwise arc to (7.85,5.55).
3. Return straight over the bump to the existing score point (3.0,5.55). Shoot
   for the requested five-second feed period.
4. Stop the roller and verify shooter stowed. Begin the second departure near
   the shooting bearing: -45 degrees left / +45 right, instead of first waiting
   for the existing auto's -90/+90 reposition heading. Turn to the trench
   heading within the clear alliance-side corner, BEFORE any barrier overlap.
5. Go outbound through the trench at Y=7.39. Restart intake only after full
   trench clearance, approaching (5.9,5.95).
6. Take a larger rear-leading C sweep centered at (7.2,4.25), radius 1.3 m:
   left (5.9,4.25), bottom (7.2,2.95), right (8.5,4.25), top (7.2,5.55).
   This reaches deeper carpet and more neutral-side behind-hub territory.
7. Exit directly west, return over the bump, and shoot until the 19.8 s output
   guard stops the routine. No S-turn or in-place reversal is needed to return.

The opening keeps 4.6 m/s and 10.2 m/s² global ceilings, with earlier local braking
for the turn-in, 3.3 m/s straight pickup and 2.3 m/s tight first-harvest arc.
The broad second pickup uses 2.8 m/s. Loaded bump crossings retain the existing
4.2 m/s bound. Unlimited constraints remain off. Power settings are unchanged:
91 A supply per drive motor only in enabled auto, 40 A otherwise, with existing
voltage fallback and the separate 120 A stator cap. No odometry changes.

## Measured headless comparison

These are generated trajectories and a delayed planar model, not hardware times.
Pickup uses the same assumed 0.80 m usable roller width and geometric fuel-region
grid as the existing tests. Swept area is NOT a fuel count or score estimate.

| Metric | Existing double | New maximum pickup |
| --- | ---: | ---: |
| First carpet coverage | 1.219 m² | 1.382 m² (+13%) |
| Total carpet, counted once | 3.365 m² | 3.966 m² (+18%) |
| Fresh trip-two carpet | 2.146 m² | 2.584 m² (+20%) |
| Combined behind-hub pocket coverage | 1.267 m² | 1.717 m² (+35%) |
| First collection + return drive | 3.610 s | 3.788 s |
| Second collection + return drive | 6.591 s | 7.446 s |
| Total generated drive | 10.201 s | 11.234 s |
| Robot-center arrival at centerline, from drive start | 1.200 s | 1.240 s |
| First active pickup, from drive start | 1.400 s | 1.390 s |
| Nominal final feed, including 0.35 s startup stow | 2.32–2.38 s | 1.42–1.46 s |

The new route is more aggressive in collection coverage, NOT faster overall.
Do not select it expecting a second five-second dump. With the assumed 35–45
capacity and four-to-five-second full-hopper discharge, substantial fuel can
remain at autonomous end. The existing double may score more if its smaller
pickup already fills the hopper, because it leaves longer for shooting.

With the startup mechanism already stowed, modeled final feed is 1.76–1.78 s.
With 1.25 s final readiness and the assumed 0.35 s startup stow, 0.74–0.76 s remains.
Modeled robot-center arrival is about 1.62 s from auto enable with that startup
stow; it is not a sub-second launch claim.

## Timing/fault behavior

The new profile alone reserves 9.45 s AFTER second-departure preparation:
7.446 drive + 1.25 final readiness + 0.50 minimum feed + 0.25 scheduling margin.
Its departure cutoff is 10.35 s. Both the pre-preparation and post-preparation
checks must pass. Existing profiles retain their original 9.0 s reserve.

Slow first readiness or successful-but-late stow stays at the first score and
uses remaining time shooting. Failed startup or second-travel clearance/heading
checks abort; they must not launch an unsafe trench transit. Every exit stops
the roller/shooter and retains the existing output cleanup and auto brake.

Five-second unloading is still the team's accepted TIMED ASSUMPTION, not an
empty-hopper sensor. A jam or residual fuel above trench height invalidates that
assumption. The new route does not solve it. Both outbound trips still require
the shooter stowed and the physical hopper/net/fuel load to fit the trench.

## Verification

`gradlew.bat clean check build`: 26 JUnit methods passed, zero failures/errors.
All 20 existing regression methods remain intact; six methods cover the new autos.

- Planned oriented bumper/rear-intake clearance, trench and bump routing, finite
  trajectories, exact right reflection, single red flip, and moving joins.
- Planned pickup direction <=10 degrees (observed maximum about 8.12 degrees).
- 32 new standalone tracking cases with 60/120 ms response on both alliances.
- 32 new launch cases across all +/-5 cm X/Y and +/-2 degree heading signs
  (64 new standalone tracking cases total, including the 32 leg-response cases).
- 28 whole-auto command-graph cases with mocked mechanism readiness: both sides
  and alliances, two real collection trips, full first dump, late first/final
  readiness, late-but-successful second stow, failed stow and startup abort.
- Modeled whole-auto pickup misalignment peaks around 14.8 degrees, below the
  unchanged 25-degree acceptance bound. Old acceptance tolerances were not changed.
- Separate admission-budget checks and the same low-voltage/current-policy tests.

Earlier S-turn candidates were rejected for excess time, tracking errors, or
overlap. New coverage targets were scoped to the final deadline-compatible
continuous-arc strategy: >=10% first-carpet and >=15% total-carpet improvement,
not the rejected draft's unattainable-in-budget 40%/25% targets. No existing
competition test threshold was loosened.

Reports: `build/reports/auto-paths/max-timing.csv`, `max-coverage.csv`,
`max-samples.csv`, and `max-left-double-command-trace.csv`.

The planar tracking model knows its injected pose error; it does not prove
recovery from an unknown physical placement error. It is not calibrated bump
pitch/roll, wheel slip, fuel physics, battery sag, CAN timing, roboRIO performance,
or other-robot avoidance. Coordinate lanes with alliance partners. Nominal
sideways trench margin remains only 7.47 cm per side; verify hardware clearance.
Start with controlled physical testing of this NEW route on both alliances,
especially second-departure rotation, loaded bump behavior, and shooting time.
