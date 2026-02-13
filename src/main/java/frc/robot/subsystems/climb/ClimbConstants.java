package frc.robot.subsystems.climb;

import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.signals.InvertedValue;

public final class ClimbConstants {
    // TODO: configure constants

    public static final String kClimbStateKey = "ClimbState";
    public static final String kClimbPositionKey = "ClimbPosition";

    public static final int kClimbMotorId = 4;

    private static final class ClimbConfigs {
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

    public static final TalonFXConfiguration ClimbMotorConfig = new TalonFXConfiguration()
        .withSlot0(ClimbConfigs.Slot0Configs).withMotionMagic(ClimbConfigs.MotionMagicConfigs)
        .withMotorOutput(new MotorOutputConfigs().withInverted(InvertedValue.CounterClockwise_Positive));
    
    public enum ClimbState {
        STOW(0),
        CLIMB(30);

        public final double position;

        private ClimbState(final double position) {
            this.position = position;
        }
    }
}
