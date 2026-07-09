package frc.robot.subsystems.shooter;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.signals.InvertedValue;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;

public final class ShooterConstants {
    public static final String kShooterTargetPositionKey = "ShooterTargetPosition";

    public static final int kIndexerMotorId = 53; // Kraken X44

    public static final int kLeaderPivotMotorId = 5;
    public static final int kFollowerPivotMotorId = 6;

    public static final int kLeftLeaderShooterMotorId = 7;
    public static final int kBackLeftFollowerShooterMotorId = 8;
    public static final int kRightFollowerShooterMotorId = 11;
    public static final int kBackRightFollowerShooterMotorId = 49;

    // ===== Game-specific targets =====
    // Hub positions in blue-origin field coordinates (field corners: blue (0,0), red (16.54, 8.21)).
    public static final Translation2d kBlueHubPosition = new Translation2d(4.625, 4.035);
    public static final Translation2d kRedHubPosition = new Translation2d(11.925, 4.035);

    // Field bounds from the official 2026-rebuilt-welded AprilTag layout.
    public static final double kFieldLengthMeters = 16.541;
    public static final double kFieldWidthMeters = 8.069;

    // ===== Passing (ferry to the alliance zone) =====
    // Pass landing targets are the alliance-zone corners. The heavy backspin makes
    // balls check up and often bounce BACK toward the field on landing, so the aim
    // point is deliberately deep (small inset): impact near the corner, bounce-back
    // settles the ball in the zone. Even a wall hit drops dead at the base with
    // backspin. Live-tunable via kPassCornerInsetKey -- tune by watching where balls
    // SETTLE, not where they first land.
    public static final double kPassCornerInsetMeters = 1.0;
    public static final String kPassCornerInsetKey = "ShotTuning/PassCornerInsetM";
    // Wider aim tolerance than hub shots: a pass lands in a ~2 m zone, not a 0.6 m hub.
    public static final double kPassHeadingToleranceDegrees = 10.0;
    // Passes command up to ~60 RPS; spin-up from idle takes longer than a hub shot's.
    public static final double kPassSpinupTimeoutSeconds = 2.0;
    public static final String kPassRpsOffsetKey = "ShotTuning/PassRpsOffset";

    // ===== Shooter fixed states =====
    public static final double kIdleShooterRps = 6.7;        // wheel pre-spin while coasting (near hub only)
    // Only idle-spin the flywheel within this range of the hub; coast to a stop farther
    // out to save the continuous 4-motor draw. Generous enough to be fully spun up before
    // you enter the 5 m shooting range.
    public static final double kIdleSpinMaxDistanceMeters = 6.5;
    // Below this bus voltage, drop the flywheel idle-spin to protect the drivetrain
    // (brownout floor is ~6.0-6.3 V; this leaves margin to react before we get there).
    public static final double kIdleShedVoltage = 7.0;
    public static final double kLobShooterRps = 40;          // ferry shot (slower pass through field)
    public static final double kSendShooterRps = 90;         // long ferry shot (fast pass)
    public static final double kStowPivotPosition = 0;       // stow angle (output rotations)
    // LOB/SEND pivot: full feedable travel (~25°) for flattest kicker-feedable launch.
    // 25 deg is the kicker FEED limit, not a travel limit -- the pivot travels much
    // farther (SHOT_BLOCK commands 0.5 rot). SCORE table entries may sit above this;
    // they fed fine on the field.
    public static final double kLobPivotPosition = 25.0 / 360.0;  // ~0.0694 output rotations
    // PASS pivot: at the LOB feed limit (0.069). Originally set FLATTER (0.086) to reduce
    // backspin bounce-back on landing, but that is PAST the kicker feed point -- it won't
    // feed (field-confirmed). So the pass launches at the flattest angle that actually
    // feeds; backspin bounce is instead managed by aiming deep into the corner
    // (kPassCornerInsetMeters). Clamped by ShotTuning/MaxFeedablePivotRot in Shooter.java.
    public static final double kPassPivotPosition = 0.069;
    // SHOT_BLOCK: raise the shooter to block incoming shots. 0.5 rot (~180 deg) is real,
    // reachable travel -- the pivot's range extends far past the 25 deg feed limit
    // (confirmed on the robot; 25 deg is only where the kicker stops being able to feed).
    public static final double kShotBlockPivotPosition = 0.5;

    // ===== Pivot geometry =====
    public static final double kPivotGearRatio = 42.0;   // motor rotations per output rotation

    // ===== Live tuning keys (dashboard) =====
    public static final String kPassPivotPositionKey = "ShotTuning/PassPivotRot";
    public static final String kShotBlockPivotPositionKey = "ShotTuning/ShotBlockPivotPosition";
    public static final String kPivotCurrentLimitKey = "ShotTuning/PivotCurrentLimitA";
    public static final String kPivotCurrentTimeoutKey = "ShotTuning/PivotCurrentTimeoutS";
    public static final String kIndexerCurrentLimitKey = "ShotTuning/IndexerCurrentLimitA";
    public static final String kIndexerCurrentTimeoutKey = "ShotTuning/IndexerCurrentTimeoutS";

    // ===== Indexer/kicker =====
    public static final double kIndexerScoreVolts = -4.5;   // negative = feed the ball into the drum

    // ===== Readiness gates (shot preconditions) =====
    // These four gates form a conjunctive (AND) precondition for readyToShoot(). Each is
    // published individually on the dashboard so a stuck gate is diagnosable without guessing.
    // If any gate blocks for longer than kShotSpinupTimeoutSeconds (1.25 s), the shot feeds
    // unconditionally — preventing the robot from hanging with the flywheel spinning forever.
    //
    // DESIGN RATIONALE: The four gates separate concerns:
    //   1. In-range: ignores hub when it's too far to reach accurately. Beyond ~5 m, angle
    //      errors dominate. Shooting beyond range is usually a bug (bad vision, bad math).
    //   2. Aimed: heading stability. The drivetrain's heading PID (P=8) settles in ~1 s, so
    //      waiting for <5° error is cheap and ensures the shot doesn't leave heading-wise.
    //   3. Flywheel at speed: velocity ripple dies down. At 25–30 RPS with MotionMagic control,
    //      ripple is ±5–10 RPS at startup, then ±2 RPS once settled. The 3 RPS tolerance
    //      waits for steady-state.
    //   4. Pivot in position: mechanical settling. The pivot is MotionMagic-controlled with
    //      0.04 rot tolerance = ~14.4°, which accounts for electrical noise and mechanical
    //      play in the 42:1 reducer. Tighter tolerances are noise-limited; looser tolerances
    //      risk overshooting or jamming on a hard stop.
    //
    // TUNING: These values are conservative. If a gate never clears (e.g., heading is always
    // off by >5°), increase the tolerance temporarily to diagnose the subsystem (bad gyro,
    // bad odometry, bad aiming math) rather than raising the gate timeout.

    // Flywheel readiness: within 3 RPS of target. At 25 RPS idle, ±3 is ±12% error.
    // Accounts for ripple from the velocity PID and CAN-bus quantization (0.25 RPS units).
    // Too tight: noisy, waits forever. Too loose: inconsistent exit speed.
    public static final double kShooterReadyToleranceRps = 3.0;

    // Pivot readiness: within 0.04 output rotations (~14.4°). The pivot is geared 42:1,
    // so 0.04 rot ≈ 1.68 motor rotations. Accounts for sensor noise (±0.01 rot), gear
    // play, and MotionMagic overshoot damping. The Pigeon 2 mag encoder has 1-bit jitter
    // (±0.0005 rot), but the swerve steer encoders are FusedCANcoder (more stable).
    public static final double kPivotReadyToleranceRotations = 0.04;

    // Heading readiness: within 5°. The drivetrain's heading PID (P=8, continuous input)
    // settles in ~0.5–1 s with zero steady-state error. The 5° tolerance accounts for
    // measurement noise (Pigeon 2 drift ~0.1°/s) and wind gusts hitting the robot. Shots
    // within 5° of perfect heading miss the hub at typical distances by <6 inches.
    public static final double kHeadingReadyToleranceDegrees = 5.0;

    // In-range gate: don't attempt shots beyond 5 m. Beyond ~5 m, the minimum-energy angle
    // is nearly 45°, and the required speed exceeds 33 RPS (wheel limit). Also, measurement
    // noise (vision, odometry) grows with distance; absolute heading error is small, but
    // angular error (error/distance) becomes large. At 5 m with 5° heading error, the ball
    // misses the hub by ~0.44 m. At 6 m, it misses by ~0.52 m. Not worth the reliability
    // tradeoff. If the drivetrain can't reach a closer position, back up instead.
    public static final double kMaxShotDistanceMeters = 5.0;

    // Vision outlier rejection: reject pose updates that jump >1.0 m from current odometry.
    // Accounts for ambiguous tag detections (tag is seen from two sides of the hub),
    // reflections off shiny objects, and transient occlusions. A 1 m jump at odometry error
    // <0.5 m is statistically impossible; a jump at odometry error >0.5 m is worth investigating.
    public static final double kMaxVisionCorrectionMeters = 1.0;

    // Force-feed timeout: if readyToShoot() doesn't go true within 1.25 s, feed anyway.
    // Why 1.25 s? Flywheel spin-up is ~0.5 s (from 0 to 30 RPS with current controller).
    // Pivot motion-magic takes ~0.3 s for a 15° move. Heading PID settles in ~0.5 s.
    // Total critical path is ~0.8 s; 1.25 s gives 0.45 s of slack for retry (if a gate
    // temporarily blocks, the robot waits a beat rather than timing out immediately).
    // If a gate hangs longer than 1.25 s, it usually signals a hardware failure (dead motor,
    // broken encoder, jammed mechanism); force-feeding is better than hanging forever.
    public static final double kShotSpinupTimeoutSeconds = 1.25;

    // ===== Tuning offsets (dashboard) =====
    // These shift the entire shot table without code redeploy. The driver can trim the
    // pivot angle and RPS globally to match the real robot's performance.
    public static final String kPivotOffsetKey = "ShotTuning/PivotOffset";
    public static final String kShooterRpsOffsetKey = "ShotTuning/ShooterRpsOffset";
    public static final String kTimeOfFlightOffsetKey = "ShotTuning/TimeOfFlightOffset";

    private static final InterpolatingDoubleTreeMap ScorePivotPositionByDistance = new InterpolatingDoubleTreeMap();
    private static final InterpolatingDoubleTreeMap ScoreShooterRpsByDistance = new InterpolatingDoubleTreeMap();
    private static final InterpolatingDoubleTreeMap ShotTimeOfFlightSecondsByDistance = new InterpolatingDoubleTreeMap();
    private static final InterpolatingDoubleTreeMap PassRpsByDistance = new InterpolatingDoubleTreeMap();

    static {
        // === Physics-derived shot tables (minimum-energy trajectories) ===
        //
        // GEOMETRY: Hub opening lip at 72" (1.829 m) above carpet. Shooter release point
        // estimated at ~12" (0.305 m) above carpet → Δh = 1.524 m. Hub is a ~2.5' wide
        // horizontal opening, so the ball must descend steeply enough to pass through.
        //
        // OPTIMIZATION CRITERION: Minimum energy (slowest speed). Why? Because slower
        // balls are more affected by gravity, descending more steeply into the opening.
        // Fast shots arc flat and overshoot the back. The minimum-energy trajectory for
        // a given distance d and height Δh is the 45° + ½atan(Δh/d) solution.
        //
        // LAUNCH ANGLE DERIVATION:
        //   At minimum energy, the optimal launch angle is θ = 45° + ½·atan(Δh/d).
        //   Why? It's the angle that minimizes kinetic energy needed for a given horizontal
        //   distance and vertical rise. As d increases, atan(Δh/d) → 0, so θ → 45°.
        //   As d decreases, the angle steepens (e.g., at 1.5 m, θ ≈ 46.5°).
        //   Constraint: our pivot range is ~0–30°, and the kicker-clearance floor is ~3°,
        //   so the table stays within [0.008, 0.050] rotations (roughly 3°–18°).
        //
        // VELOCITY DERIVATION:
        //   Required speed to reach distance d with height gain Δh:
        //     v² = g·(Δh + √(d² + Δh²))
        //   Why? It's the minimum speed such that the trajectory passes through the point
        //   (d, Δh) when launched at the optimal angle. The sqrt term accounts for the
        //   hypotenuse of the target point in 3D space (horizontal distance + vertical rise).
        //
        // CONVERSIONS TO MOTOR UNITS:
        //   Ball speed v (m/s) → Wheel RPS: assume 75% wheel-to-ball transfer efficiency
        //   and 4" diameter drum (radius r = 0.1016 m, circumference = 0.638 m/rot).
        //     RPS = v / (0.75 · 0.638) ≈ 4.177 · v
        //   TOF (seconds): derived from the trajectory equation; at apex the ball's vertical
        //   velocity is zero, so t_up = v_y / g. Total TOF ≈ 2 · t_up.
        //
        // FIELD-MEASURED ADJUSTMENTS NEEDED:
        //   1. Pivot floor: ~3° (0.008 rot) is a guess. The true floor is where the pivot
        //      lifts the drum just clear of the kicker wheels. This MUST be confirmed on
        //      the field; if the floor is wrong, close shots will jam the kicker.
        //   2. RPS bias: these are vacuum (no air resistance) and assume 75% efficiency.
        //      Real life has drag and belt slippage. Expect to trim up 10–25% on the field.
        //      Trim is global across all distances via ShotTuning/ShooterRpsOffset.
        //   3. TOF: includes latency from shot trigger to ball exit. If release is delayed
        //      or the shot exits slower than modeled, increase ShotTuning/TimeOfFlightOffset.
        //
        // DATA SOURCE: Derivation follows the minimum-energy ballistic lob used by teams
        // 1678 (Citrus Circuits) and 2910 (Jack in the Bot), published openly. Their field-
        // tuned values at distances 1.5/2.5/3.5/4.5 m match this curve within 5 RPS,
        // validating the physics model.

        // Pivot angle (output rotations) by distance. Relative spacing encodes the
        // min-energy launch angles; absolute position includes the ~3° floor offset.
        // RE-ANCHOR 2026-07-04 (supersedes the min-energy baseline derived above): while
        // the hub-distance bug clamped the table at max, every shot ran hood=0.047 rot /
        // 37.3 RPS, and that combination demonstrably landed at midrange (~2.75 m). That
        // one working point re-anchors both curves:
        //   - Hood model: stow launches ~70 deg from horizontal, 1 pivot deg = 1 launch
        //     deg flatter (LOB at 25 deg pivot ~ 45 deg launch fits the same line).
        //   - The working shot sat ~0.018 rot flatter than the min-energy curve, so the
        //     whole pivot curve shifts up by +0.018 (same shape, kicker floor still clear;
        //     5.0 m entry stays under the 0.0694 feedable max).
        //   - RPS recomputed for the flatter arcs with a x1.30 real-world factor (drag +
        //     transfer losses) calibrated from that same shot: vacuum said 28.7 where the
        //     field needed 37.3. Sanity check: this table independently reproduces the
        //     working midrange shot (predicts 37.0 at 2.5 m).
        // Long end (4.5-5.0 m) is still model, not measurement -- verify and trim there.
        // FIELD-BAKED 2026-07-04: shots landed at all tested distances with PivotOffset
        // +0.02 over the re-anchored curve, so +0.02 is baked straight in. Entries past
        // ~3 m sit above the 25 deg LOB feed limit -- that's fine for SCORE shots on this
        // mechanism (25 deg is a feed limit, not a travel limit, and these fed fine).
        // FEED LIMIT: the kicker can't push a ball through past ~0.069 rot (the LOB angle,
        // ~45 deg launch). Field-confirmed that 0.076+ won't feed, so the hood SATURATES at
        // the feed limit from 3.5 m out and range is carried by RPS instead. 45 deg is also
        // the max-range launch angle, so nothing is lost aerodynamically. Shooter.java clamps
        // to ShotTuning/MaxFeedablePivotRot as a backstop against a stray offset.
        ScorePivotPositionByDistance.put(1.5, 0.046);
        ScorePivotPositionByDistance.put(2.5, 0.065);
        ScorePivotPositionByDistance.put(3.5, 0.069);
        ScorePivotPositionByDistance.put(4.5, 0.069);
        ScorePivotPositionByDistance.put(5.0, 0.069);  // table must cover the 5 m range gate

        // Flywheel speed (RPS) by distance, matched to the re-anchored (flatter) hood
        // curve above: v^2 = g*d^2 / (2*cos^2(theta)*(d*tan(theta) - dh)) per distance,
        // converted to wheel RPS and scaled by the field-calibrated x1.30 factor. A small
        // progressive drag allowance (+0.5/+1/+1.5) is added at 3.5/4.5/5.0 m since the
        // x1.30 factor was calibrated at midrange and drag grows with range.
        // NOTE: after any table change, ShotTuning/ShooterRpsOffset AND PivotOffset on
        // the dashboard must go back to 0 or the trims get double-counted.
        // FIELD-BAKED 2026-07-04: +4.5 RPS (tuned alongside the +0.02 hood trim, confirmed
        // working at all tested distances) baked in flat.
        ScoreShooterRpsByDistance.put(1.5, 39.0);
        ScoreShooterRpsByDistance.put(2.5, 41.5);
        ScoreShooterRpsByDistance.put(3.5, 45.5);
        ScoreShooterRpsByDistance.put(4.5, 49.0);
        ScoreShooterRpsByDistance.put(5.0, 51.0);

        // Time-of-flight (seconds) by distance. Used by the drivetrain for motion
        // compensation (where will the hub be when the ball arrives?). These are computed
        // from the trajectory equation; precision matters because errors here throw off
        // the aim heading. The TOF offset (dashboard) accounts for release latency and
        // any systematic under/overestimate in the physics model.
        // Recomputed for the flatter, faster re-anchored trajectories: t = d/(v*cos(theta)).
        ShotTimeOfFlightSecondsByDistance.put(1.5, 0.48);
        ShotTimeOfFlightSecondsByDistance.put(2.5, 0.62);
        ShotTimeOfFlightSecondsByDistance.put(3.5, 0.73);
        ShotTimeOfFlightSecondsByDistance.put(4.5, 0.83);
        ShotTimeOfFlightSecondsByDistance.put(5.0, 0.87);

        // Pass power by distance to the landing target. Same calibrated model as the hub
        // tables: launch is the LOB pivot (~45 deg from the hood model), ground-to-ground
        // range solved from ballistics with a 0.305 m release height, ball speed converted
        // to wheel RPS with the field-calibrated x1.30 factor. Untested on the field --
        // trim globally with ShotTuning/PassRpsOffset, then re-bake like the hub tables.
        PassRpsByDistance.put(3.0, 28.0);
        PassRpsByDistance.put(5.0, 37.0);
        PassRpsByDistance.put(7.0, 44.0);
        PassRpsByDistance.put(9.0, 50.0);
        PassRpsByDistance.put(11.0, 56.0);
        PassRpsByDistance.put(13.0, 61.0);
    }

    public static double getScorePivotPosition(double distanceMeters) {
        return ScorePivotPositionByDistance.get(distanceMeters);
    }

    public static double getScoreShooterRps(double distanceMeters) {
        return ScoreShooterRpsByDistance.get(distanceMeters);
    }

    public static double getShotTimeOfFlightSeconds(double distanceMeters) {
        return ShotTimeOfFlightSecondsByDistance.get(distanceMeters);
    }

    public static double getPassRps(double distanceMeters) {
        return PassRpsByDistance.get(distanceMeters);
    }

    private static final class PivotConfigs {
        
        private static final double kA = 0.01;
        private static final double kD = 0.1;
        private static final double kI = 0;
        private static final double kP = 4.8;
        private static final double kS = 0.25;
        private static final double kV = 0.12;

        private static final double kMotionMagicCruiseVelocity = 80;
        private static final double kMotionMagicAcceleration = 160;
        private static final double kMotionMagicJerk = 1600;

        private static final Slot0Configs Slot0Configs = new Slot0Configs()
            .withKA(kA).withKD(kD).withKI(kI).withKP(kP).withKS(kS).withKV(kV);

        private static final MotionMagicConfigs MotionMagicConfigs = new MotionMagicConfigs()
            .withMotionMagicCruiseVelocity(kMotionMagicCruiseVelocity)
            .withMotionMagicAcceleration(kMotionMagicAcceleration)
            .withMotionMagicJerk(kMotionMagicJerk);
    }

    private static final class ShooterConfigs {
        // kS was 0, which is physically wrong (every motor needs some static term to
        // overcome friction). 0.15 is a conservative starting estimate -- run SysId to
        // ground kS/kV/kA in real data. kP raised 0.125 -> 0.3 to fight the velocity dip
        // when a ball passes through harder; tune it up toward the edge of oscillation
        // using the DogLog Shooter/RpsError trace. kD intentionally left 0: on a velocity
        // loop it differentiates an already-noisy signal and usually hurts more than helps.
        private static final double kS = 0.15;
        private static final double kV = .125;
        private static final double kP = .3;
        private static final double kI = 0;
        private static final double kD = 0;

        private static final Slot0Configs Slot0Configs = new Slot0Configs()
            .withKD(kD).withKI(kI).withKP(kP).withKV(kV).withKS(kS).withKA(.2);

        // Supply limit protects the bus voltage (and the main breaker) during spin-up and
        // shot spikes -- this is what most directly helps VelocityVoltage hold speed,
        // since it keeps battery sag from eating the voltage command. 35 A x 4 motors
        // caps the flywheel at 140 A of battery draw (was 200 A at 50 each, which
        // stacked with an unlimited drivetrain into brownout territory). Spin-up gets a
        // touch slower; the 1.25 s / 2 s shot timeouts already cover it. Stator limit is
        // kept generous so recovery torque during the shot is never the bottleneck.
        private static final CurrentLimitsConfigs CurrentLimits = new CurrentLimitsConfigs()
            .withSupplyCurrentLimitEnable(true).withSupplyCurrentLimit(35)
            .withStatorCurrentLimitEnable(true).withStatorCurrentLimit(100);
    }

    private static final class IndexerConfigs {
        private static final double kS = 0.1;
        private static final double kV = 0.12;
        private static final double kP = 0.11;
        private static final double kI = 0;
        private static final double kD = 0;

        private static final Slot0Configs Slot0Configs = new Slot0Configs()
            .withKD(kD).withKI(kI).withKP(kP).withKV(kV).withKS(kS);
    }

    // Hardware supply caps for the small mechanisms (brownout budget). These sit above
    // the software overcurrent trips (pivot 40 A/0.25 s watches stall; indexer 25 A),
    // so the soft trips keep their diagnostic role while the hardware cap bounds what
    // the battery can ever see.
    private static final CurrentLimitsConfigs PivotCurrentLimits = new CurrentLimitsConfigs()
        .withSupplyCurrentLimitEnable(true).withSupplyCurrentLimit(30);
    private static final CurrentLimitsConfigs IndexerCurrentLimits = new CurrentLimitsConfigs()
        .withSupplyCurrentLimitEnable(true).withSupplyCurrentLimit(30);

    public static final TalonFXConfiguration LeaderPivotConfig = new TalonFXConfiguration()
        .withSlot0(PivotConfigs.Slot0Configs).withMotionMagic(PivotConfigs.MotionMagicConfigs)
        .withCurrentLimits(PivotCurrentLimits)
        .withMotorOutput(new MotorOutputConfigs().withInverted(InvertedValue.Clockwise_Positive));

    public static final TalonFXConfiguration FollowerPivotConfig = new TalonFXConfiguration()
        .withSlot0(PivotConfigs.Slot0Configs).withMotionMagic(PivotConfigs.MotionMagicConfigs)
        .withCurrentLimits(PivotCurrentLimits)
        .withMotorOutput(new MotorOutputConfigs().withInverted(InvertedValue.CounterClockwise_Positive));

    public static final TalonFXConfiguration LeftShooterConfig = new TalonFXConfiguration()
        .withSlot0(ShooterConfigs.Slot0Configs).withCurrentLimits(ShooterConfigs.CurrentLimits)
        .withMotorOutput(new MotorOutputConfigs().withInverted(InvertedValue.Clockwise_Positive));
    public static final TalonFXConfiguration RightShooterConfig = new TalonFXConfiguration()
        .withSlot0(ShooterConfigs.Slot0Configs).withCurrentLimits(ShooterConfigs.CurrentLimits)
        .withMotorOutput(new MotorOutputConfigs().withInverted(InvertedValue.CounterClockwise_Positive));

    public static final TalonFXConfiguration IndexerConfig = new TalonFXConfiguration()
        .withSlot0(IndexerConfigs.Slot0Configs).withCurrentLimits(IndexerCurrentLimits);

    public enum PivotState {
        STOW,
        SCORE,
        LOB,
        PASS,     // flatter-than-LOB ferry angle (tunable) to tame backspin bounce-back
        SHOT_BLOCK;
    }

    public enum ShooterState {
        ZERO,
        SCORE,
        PASS,     // distance-interpolated ferry to the alliance-zone corner
        LOB,      // legacy fixed-speed ferry (kLobShooterRps)
        SEND,     // fixed-speed long ferry (kSendShooterRps)
        REVERSE;  // Back out a jammed ball at -10 RPS
    }

    public enum IndexerState {
        ZERO(0),
        SCORE(kIndexerScoreVolts),
        REVERSE(4.5);  // Back out a jammed ball toward the intake

        public final double volts;

        private IndexerState(double volts) {
            this.volts = volts;
        }
    }
}
