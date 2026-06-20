package frc.robot.subsystems.shooter;

import static edu.wpi.first.units.Units.Rotations;
import static edu.wpi.first.units.Units.RotationsPerSecond;
import static edu.wpi.first.units.Units.Seconds;

import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Time;

public final class ShooterConstants {
    public static final String kShooterPivotAtTargetKey = "Shooter/PivotAtTarget";
    public static final String kShooterPivotPositionKey = "Shooter/PivotAngle";
    public static final String kShooterRollerAtTargetKey = "Shooter/RollerAtTarget";
    public static final String kShooterRollerVelocityKey = "Shooter/RollerVelocity";
    public static final String kShooterKickerVelocityKey = "Shooter/KickerVelocity";

    public static final int kKickerMotorId = 53;

    public static final int kLeaderPivotMotorId = 5;
    public static final int kFollowerPivotMotorId = 6;

    public static final int kLeftLeaderShooterMotorId = 7;
    public static final int kBackLeftFollowerShooterMotorId = 8;
    public static final int kRightFollowerShooterMotorId = 11;
    public static final int kBackRightFollowerShooterMotorId = 49;

    private static final class PivotConfigs {
        private static final Slot0Configs Slot0Configs = new Slot0Configs()
            .withKA(0.01)
            .withKD(0.1)
            .withKI(0)
            .withKP(4.8)
            .withKS(0.25)
            .withKV(0.12);

        private static final MotionMagicConfigs MotionMagicConfigs = new MotionMagicConfigs()
            .withMotionMagicCruiseVelocity(80)
            .withMotionMagicAcceleration(160)
            .withMotionMagicJerk(1600);
    }

    private static final class ShooterConfigs {
        private static final Slot0Configs Slot0Configs = new Slot0Configs()
            .withKD(0)
            .withKI(0)
            .withKP(0.125)
            .withKV(0.125)
            .withKS(0)
            .withKA(.2);
    }

    private static final class KickerConfigs {
        private static final Slot0Configs Slot0Configs = new Slot0Configs()
            .withKD(0)
            .withKI(0)
            .withKP(0.11)
            .withKV(0.12)
            .withKS(0.1);
    }

    public static final TalonFXConfiguration PivotTalonFXConfigs = new TalonFXConfiguration()
        .withSlot0(PivotConfigs.Slot0Configs).withMotionMagic(PivotConfigs.MotionMagicConfigs)
        .withMotorOutput(new MotorOutputConfigs().withNeutralMode(NeutralModeValue.Brake));

    public static final TalonFXConfiguration RollerTalonFXConfigs = new TalonFXConfiguration()
        .withSlot0(ShooterConfigs.Slot0Configs);

    public static final TalonFXConfiguration KickerTalonFXConfigs = new TalonFXConfiguration()
        .withSlot0(KickerConfigs.Slot0Configs);

    public static final double kRollerVelocityErrorTolerance = 2.0;
    public static final Time kFeedTimeout = Seconds.of(1);

    public static final Angle kPivotStowAngle = Rotations.zero();
    public static final Angle kPivotLobAngle = Rotations.zero();
    public static final Angle kPivotSendAngle = Rotations.zero();
    public static final Angle kPivotScoreAngle = Rotations.zero();

    public static final AngularVelocity kRollerIdleVelocity = RotationsPerSecond.of(0.5);
    public static final AngularVelocity kRollerLobVelocity = RotationsPerSecond.of(40);
    public static final AngularVelocity kRollerSendVelocity = RotationsPerSecond.of(90);
    public static final AngularVelocity kRollerScoreVelocity = RotationsPerSecond.of(47);

    public static final AngularVelocity kKickerIdleVelocity = RotationsPerSecond.of(0);
    public static final AngularVelocity kkickerFeedVelocity = RotationsPerSecond.of(-4.5);

    public enum PivotState {
        STOW,
        LOB,
        SEND,
        SCORE;
    }

    public enum ShooterState {
        IDLE,
        LOB,
        SEND,
        SCORE;
    }

    public enum KickerState {
        IDLE,
        FEED;
    }
}
