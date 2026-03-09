package frc.robot.subsystems.feeder;

import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.signals.InvertedValue;

public final class FeederConstants {
    public static final String kFeederStateKey = "FeederState";
    public static final String kFeederVoltageKey = "FeederVoltage";
    public static final String kFeederTargetVoltageKey = "FeederTargetVoltage";

    public static final int kFeederMotorId = 44;

    private static final double kS = 0.1;
    private static final double kV = 0.12;
    private static final double kP = 0.11;
    private static final double kI = 0;
    private static final double kD = 0;

    private static final Slot0Configs Slot0Configs = new Slot0Configs()
        .withKD(kD).withKI(kI).withKP(kP).withKV(kV).withKS(kS);

    public static final TalonFXConfiguration FeederConfig = 
        new TalonFXConfiguration().withSlot0(Slot0Configs)
            .withMotorOutput(new MotorOutputConfigs().withInverted(InvertedValue.Clockwise_Positive));

    public enum FeederState {
        ZERO(0),
        SCORE(-6);

        public final double volts;

        private FeederState(double volts) {
            this.volts = volts;
        }
    }
}
