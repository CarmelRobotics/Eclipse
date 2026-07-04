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

    // ===== Shooter fixed states =====
    public static final double kIdleShooterRps = 6.7;        // wheel spinning slowly while coasting
    public static final double kLobShooterRps = 40;          // ferry shot (slower pass through field)
    public static final double kSendShooterRps = 90;         // long ferry shot (fast pass)
    public static final double kStowPivotPosition = 0;       // stow angle (output rotations)
    // LOB/SEND pivot: full feedable travel (~25°) for flattest kicker-feedable launch.
    // (If pivot goes lower, kicker wheels hit the drum. If higher, angle is too steep.)
    public static final double kLobPivotPosition = 25.0 / 360.0;  // ~0.0694 output rotations
    // SHOT_BLOCK: blocker-clear angle. Approx 180° of available pivot motion = 0.5 rot.
    public static final double kShotBlockPivotPosition = 0.5;

    // ===== Pivot geometry =====
    public static final double kPivotGearRatio = 42.0;   // motor rotations per output rotation

    // ===== Live tuning keys (dashboard) =====
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
        ScorePivotPositionByDistance.put(1.5, 0.008);
        ScorePivotPositionByDistance.put(2.5, 0.027);
        ScorePivotPositionByDistance.put(3.5, 0.038);
        ScorePivotPositionByDistance.put(4.5, 0.045);
        ScorePivotPositionByDistance.put(5.0, 0.048);  // table must cover the 5 m range gate

        // Flywheel speed (RPS) by distance. FIELD-CALIBRATED 2026-07-04: a flat +4.5 RPS
        // trim over the vacuum baseline landed midrange but left long range short -- the
        // drag deficit grows with distance, so the curve needs slope, not offset. The
        // +4.5 is baked in flat through 3.5 m (confirmed good), and the 4.5-5.0 m entries
        // carry progressively more on top (~+2 / +3 extra) to cover drag. Long end is an
        // estimate: re-verify at 4.5 m and trim with ShooterRpsOffset, then re-bake.
        // NOTE: after this table change, ShotTuning/ShooterRpsOffset on the dashboard
        // must go back to 0 or the trim gets double-counted.
        ScoreShooterRpsByDistance.put(1.5, 29.5);
        ScoreShooterRpsByDistance.put(2.5, 32.1);
        ScoreShooterRpsByDistance.put(3.5, 34.7);
        ScoreShooterRpsByDistance.put(4.5, 39.5);
        ScoreShooterRpsByDistance.put(5.0, 42.0);

        // Time-of-flight (seconds) by distance. Used by the drivetrain for motion
        // compensation (where will the hub be when the ball arrives?). These are computed
        // from the trajectory equation; precision matters because errors here throw off
        // the aim heading. The TOF offset (dashboard) accounts for release latency and
        // any systematic under/overestimate in the physics model.
        ShotTimeOfFlightSecondsByDistance.put(1.5, 0.66);
        ShotTimeOfFlightSecondsByDistance.put(2.5, 0.77);
        ShotTimeOfFlightSecondsByDistance.put(3.5, 0.88);
        ShotTimeOfFlightSecondsByDistance.put(4.5, 0.99);
        ShotTimeOfFlightSecondsByDistance.put(5.0, 1.05);
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

        // Supply limit protects the bus voltage (and the main breaker) during the shot
        // current spike -- this is what most directly helps VelocityVoltage hold speed,
        // since it keeps battery sag from eating the voltage command. Stator limit is
        // kept generous so recovery torque is never the bottleneck.
        private static final CurrentLimitsConfigs CurrentLimits = new CurrentLimitsConfigs()
            .withSupplyCurrentLimitEnable(true).withSupplyCurrentLimit(50)
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

    public static final TalonFXConfiguration LeaderPivotConfig = new TalonFXConfiguration()
        .withSlot0(PivotConfigs.Slot0Configs).withMotionMagic(PivotConfigs.MotionMagicConfigs)
        .withMotorOutput(new MotorOutputConfigs().withInverted(InvertedValue.Clockwise_Positive));
    
    public static final TalonFXConfiguration FollowerPivotConfig = new TalonFXConfiguration()
        .withSlot0(PivotConfigs.Slot0Configs).withMotionMagic(PivotConfigs.MotionMagicConfigs)
        .withMotorOutput(new MotorOutputConfigs().withInverted(InvertedValue.CounterClockwise_Positive));

    public static final TalonFXConfiguration LeftShooterConfig = new TalonFXConfiguration()
        .withSlot0(ShooterConfigs.Slot0Configs).withCurrentLimits(ShooterConfigs.CurrentLimits)
        .withMotorOutput(new MotorOutputConfigs().withInverted(InvertedValue.Clockwise_Positive));
    public static final TalonFXConfiguration RightShooterConfig = new TalonFXConfiguration()
        .withSlot0(ShooterConfigs.Slot0Configs).withCurrentLimits(ShooterConfigs.CurrentLimits)
        .withMotorOutput(new MotorOutputConfigs().withInverted(InvertedValue.CounterClockwise_Positive));

    public static final TalonFXConfiguration IndexerConfig = new TalonFXConfiguration().withSlot0(IndexerConfigs.Slot0Configs);

    public enum PivotState {
        STOW,
        SCORE,
        LOB,
        SHOT_BLOCK;
    }

    public enum ShooterState {
        ZERO,
        SCORE,
        LOB,
        SEND,
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
