package frc.robot.subsystems.shooter;

import static edu.wpi.first.units.Units.RotationsPerSecond;

import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.signals.InvertedValue;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.units.measure.AngularVelocity;

public final class ShooterConstants {
    // TODO: Configure constants

    public static final String kShooterPivotStateKey = "ShooterPivotState";
    public static final String kShooterIndexerStateKey = "ShooterIndexerState";
    public static final String kShooterPositionKey = "ShooterPosition";
    public static final String kShooterVelocityKey = "ShooterVelocity";

    public static final int kIndexerMotorId = 53; // Kraken X44

    public static final int kLeaderPivotMotorId = 5;
    public static final int kFollowerPivotMotorId = 6;

    public static final int kLeftLeaderShooterMotorId = 7;
    public static final int kBackLeftFollowerShooterMotorId = 8;
    public static final int kRightFollowerShooterMotorId = 11;
    public static final int kBackRightFollowerShooterMotorId = 49;

    public static final Translation2d kBlueHubPosition = new Translation2d(4.625, 4.035);
    public static final Translation2d kRedHubPosition = new Translation2d(11.925, 4.035);

    public static final InterpolatingDoubleTreeMap PivotPositionMap = InterpolatingDoubleTreeMap
        .ofEntries(
            // TODO: put data here
        );
    public static final InterpolatingDoubleTreeMap ShooterVelocityRPSMap = InterpolatingDoubleTreeMap
        .ofEntries(
            // TODO: put data here
        );

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
        private static final double kS = 0.1;
        private static final double kV = 0.22;
        private static final double kP = 0.5;
        private static final double kI = 0;
        private static final double kD = 0;

        private static final Slot0Configs Slot0Configs = new Slot0Configs()
            .withKD(kD).withKI(kI).withKP(kP).withKV(kV).withKS(kS);
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
        SCORE;
    }

    public enum ShooterState {
        ZERO,
        SCORE;
    }

    public enum IndexerState {
        ZERO(RotationsPerSecond.of(0)),
        SCORE(RotationsPerSecond.of(-10));

        public final AngularVelocity velocity;

        private IndexerState(AngularVelocity velocity) {
            this.velocity = velocity;
        }
    }
}
