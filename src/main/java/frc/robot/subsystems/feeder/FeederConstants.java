package frc.robot.subsystems.feeder;

import static edu.wpi.first.units.Units.RotationsPerSecond;

import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;

import edu.wpi.first.units.measure.AngularVelocity;

public final class FeederConstants {
    public static final String kFeederStateKey = "FeederState";

    public static final int kFeederMotorId = 44;

    private static final double kS = 0.1;
    private static final double kV = 0.12;
    private static final double kP = 0.11;
    private static final double kI = 0;
    private static final double kD = 0;

    private static final Slot0Configs Slot0Configs = new Slot0Configs()
        .withKD(kD).withKI(kI).withKP(kP).withKV(kV).withKS(kS);

    public static final TalonFXConfiguration FeederConfig = new TalonFXConfiguration().withSlot0(Slot0Configs);

    public enum FeederState {
        ZERO(RotationsPerSecond.zero()),
        SCORE(RotationsPerSecond.of(100));

        public final AngularVelocity velocity;

        private FeederState(AngularVelocity velocity) {
            this.velocity = velocity;
        }
    }
}
