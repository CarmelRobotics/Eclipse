package frc.robot.subsystems.lintake;

import static edu.wpi.first.units.Units.RotationsPerSecond;

import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.signals.InvertedValue;

import edu.wpi.first.units.measure.AngularVelocity;

public final class LintakeConstants {
    // TODO: Configure constants

    public static final String kPinionStateKey = "PinionState";
    public static final String kRollerStateKey = "RollerState";
    public static final String kLeaderPinionPositionKey = "LeaderPinionPosition";
    public static final String kFollowerPinionPositionKey = "FollowerPinionPosition";
    public static final String kPinionPositionTargetKey = "PinionPositionTarget";


    public static final int kLeaderPinionMotorId = 48;
    public static final int kFollowerPinionMotorId = 2;
    public static final int kRollerMotorId = 45;

    private static final class PinionConfigs {
        private static final double kA = 0;
        private static final double kD = 0;
        private static final double kI = 0;
        private static final double kP = 0;
        private static final double kS = 0;
        private static final double kV = 0.25;

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

    private static final class RollerConfigs {
        private static final double kS = 0.1;
        private static final double kV = 0.12;
        private static final double kP = 0.11;
        private static final double kI = 0;
        private static final double kD = 0.125;

        private static final Slot0Configs Slot0Configs = new Slot0Configs()
            .withKD(kD).withKI(kI).withKP(kP).withKV(kV).withKS(kS);
    }
    
    public static final TalonFXConfiguration LeaderPinionConfig = new TalonFXConfiguration()
        .withSlot0(PinionConfigs.Slot0Configs).withMotionMagic(PinionConfigs.MotionMagicConfigs)
        .withMotorOutput(new MotorOutputConfigs().withInverted(InvertedValue.Clockwise_Positive));
    public static final TalonFXConfiguration FollowerPinionConfig = new TalonFXConfiguration()
        .withSlot0(PinionConfigs.Slot0Configs).withMotionMagic(PinionConfigs.MotionMagicConfigs)
        .withMotorOutput(new MotorOutputConfigs().withInverted(InvertedValue.CounterClockwise_Positive));

    public static final TalonFXConfiguration RollerConfig = new TalonFXConfiguration().withSlot0(RollerConfigs.Slot0Configs);

    public enum PinionState {
        STOW(-2),
        GROUND(0.1);

        public final double position;

        private PinionState(double position) {
            this.position = position;
        }
    }

    public enum RollerState {
        ZERO(0),
        EJECT(12),
        INTAKE(-5.15),
        SCORE(12);

        public final double velocity;

        private RollerState(double velocity) {
            this.velocity = velocity;
        }
    }
}
