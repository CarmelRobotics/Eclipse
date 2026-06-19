package frc.robot.subsystems.shooter;

import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.signals.InvertedValue;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.math.util.Units;

public final class ShooterConstants {
    public static final String kShooterPivotStateKey = "ShooterPivotState";
    public static final String kShooterIndexerStateKey = "ShooterIndexerState";
    public static final String kShooterPositionKey = "ShooterPosition";
    public static final String kShooterVelocityKey = "ShooterVelocity";
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
        ScorePivotPositionByDistance.put(1.5, 0.0);
        ScorePivotPositionByDistance.put(2.5, 0.0);
        ScorePivotPositionByDistance.put(3.5, 0.0);
        ScorePivotPositionByDistance.put(4.5, 0.0);

        ScoreShooterRpsByDistance.put(1.5, 47.0);
        ScoreShooterRpsByDistance.put(2.5, 47.0);
        ScoreShooterRpsByDistance.put(3.5, 47.0);
        ScoreShooterRpsByDistance.put(4.5, 47.0);

        ShotTimeOfFlightSecondsByDistance.put(1.5, 0.18);
        ShotTimeOfFlightSecondsByDistance.put(2.5, 0.24);
        ShotTimeOfFlightSecondsByDistance.put(3.5, 0.30);
        ShotTimeOfFlightSecondsByDistance.put(4.5, 0.36);
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
        private static final double kS = 0;
        private static final double kV = .125;
        private static final double kP = .125;
        private static final double kI = 0;
        private static final double kD = 0;

        private static final Slot0Configs Slot0Configs = new Slot0Configs()
            .withKD(kD).withKI(kI).withKP(kP).withKV(kV).withKS(kS).withKA(.2);
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
        .withSlot0(ShooterConfigs.Slot0Configs).withMotorOutput(new MotorOutputConfigs().withInverted(InvertedValue.Clockwise_Positive));
    public static final TalonFXConfiguration RightShooterConfig = new TalonFXConfiguration()
        .withSlot0(ShooterConfigs.Slot0Configs).withMotorOutput(new MotorOutputConfigs().withInverted(InvertedValue.CounterClockwise_Positive));

    public static final TalonFXConfiguration IndexerConfig = new TalonFXConfiguration().withSlot0(IndexerConfigs.Slot0Configs);

    public enum PivotState {
        STOW,
        SCORE,
        LOB;
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
