package frc.robot.subsystems.shooter;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.signals.InvertedValue;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.math.util.Units;

public final class ShooterConstants {
    public static final String kShooterTargetPositionKey = "ShooterTargetPosition";

    public static final int kIndexerMotorId = 53; // Kraken X44

    public static final int kLeaderPivotMotorId = 5;
    public static final int kFollowerPivotMotorId = 6;

    public static final int kLeftLeaderShooterMotorId = 7;
    public static final int kBackLeftFollowerShooterMotorId = 8;
    public static final int kRightFollowerShooterMotorId = 11;
    public static final int kBackRightFollowerShooterMotorId = 49;

    public static final Translation2d kBlueHubPosition = new Translation2d(4.625, 4.035);
    public static final Translation2d kRedHubPosition = new Translation2d(11.925, 4.035);

    public static final double kIdleShooterRps = 0.5;
    public static final double kLobShooterRps = 40;
    public static final double kSendShooterRps = 90;
    public static final double kStowPivotPosition = 0;
    // LOB/SEND pivot target: the full feedable travel (~25 deg from stow), giving the
    // flattest launch the kicker can still feed at -- used for ferry shots.
    public static final double kLobPivotPosition = 25.0 / 360.0;
    // Approx 180 degrees of pivot motion (in rotations). Assumption: 0.5 rotations ~= 180°
    public static final double kShotBlockPivotPosition = 0.5;
    // Gear ratio: motor rotations per output (pivot) rotation
    public static final double kPivotGearRatio = 42.0;
    // SmartDashboard keys for runtime tuning
    public static final String kShotBlockPivotPositionKey = "ShotTuning/ShotBlockPivotPosition";
    public static final String kPivotCurrentLimitKey = "ShotTuning/PivotCurrentLimitA";
    public static final String kPivotCurrentTimeoutKey = "ShotTuning/PivotCurrentTimeoutS";
    public static final String kIndexerCurrentLimitKey = "ShotTuning/IndexerCurrentLimitA";
    public static final String kIndexerCurrentTimeoutKey = "ShotTuning/IndexerCurrentTimeoutS";
    public static final double kIndexerScoreVolts = -4.5;
    public static final double kShooterHeadingOffsetRadians = Units.degreesToRadians(0);
    public static final double kShooterReadyToleranceRps = 3.0;
    public static final double kPivotReadyToleranceRotations = 0.04;
    public static final double kHeadingReadyToleranceDegrees = 5.0;
    public static final double kMaxShotDistanceMeters = 5.0;
    public static final double kMaxVisionCorrectionMeters = 1.0;
    public static final double kShotSpinupTimeoutSeconds = 1.25;

    public static final String kPivotOffsetKey = "ShotTuning/PivotOffset";
    public static final String kShooterRpsOffsetKey = "ShotTuning/ShooterRpsOffset";
    public static final String kTimeOfFlightOffsetKey = "ShotTuning/TimeOfFlightOffset";

    private static final InterpolatingDoubleTreeMap ScorePivotPositionByDistance = new InterpolatingDoubleTreeMap();
    private static final InterpolatingDoubleTreeMap ScoreShooterRpsByDistance = new InterpolatingDoubleTreeMap();
    private static final InterpolatingDoubleTreeMap ShotTimeOfFlightSecondsByDistance = new InterpolatingDoubleTreeMap();

    static {
        // Minimum-energy lob solution for a top-drop hub whose opening lip is 72" off
        // the carpet, with the shooter releasing at ~12" (Delta-h = 1.524 m). For each
        // distance the least-speed trajectory is used: it gives the steepest descent
        // into the horizontal opening, and its launch angle stays within the pivot's
        // ~25-30 deg range. Farther shots flatten the arc.
        // theta = 45 + 0.5*atan(dh/d); v^2 = g*(dh + hypot(d, dh)); rps = 4.177*v_ball.
        //
        // Pivot positions are output rotations. The relative spacing between distances
        // comes from the min-energy launch angles; the whole curve is then offset up by
        // a ~3 deg (0.008 rot) floor because at dead stow the shooter rests ON the kicker
        // wheels and they cannot spin to feed until the pivot lifts clear. So the closest
        // shot sits at the floor, not at 0. The floor is approximate -- confirm the true
        // kicker-clearance angle on the field.
        // These RPS are vacuum values (no drag) and bias ~5-10% low; trim globally on the
        // field with the ShotTuning/ShooterRpsOffset dashboard key.
        ScorePivotPositionByDistance.put(1.5, 0.008);
        ScorePivotPositionByDistance.put(2.5, 0.027);
        ScorePivotPositionByDistance.put(3.5, 0.038);
        ScorePivotPositionByDistance.put(4.5, 0.045);

        ScoreShooterRpsByDistance.put(1.5, 25.0);
        ScoreShooterRpsByDistance.put(2.5, 27.6);
        ScoreShooterRpsByDistance.put(3.5, 30.2);
        ScoreShooterRpsByDistance.put(4.5, 32.8);

        ShotTimeOfFlightSecondsByDistance.put(1.5, 0.66);
        ShotTimeOfFlightSecondsByDistance.put(2.5, 0.77);
        ShotTimeOfFlightSecondsByDistance.put(3.5, 0.88);
        ShotTimeOfFlightSecondsByDistance.put(4.5, 0.99);
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
        SEND;
    }

    public enum IndexerState {
        ZERO(0),
        SCORE(kIndexerScoreVolts);

        public final double volts;

        private IndexerState(double volts) {
            this.volts = volts;
        }
    }
}
