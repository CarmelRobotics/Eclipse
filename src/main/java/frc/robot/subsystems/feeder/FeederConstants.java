package frc.robot.subsystems.feeder;

import static edu.wpi.first.units.Units.RotationsPerSecond;

import com.ctre.phoenix6.configs.TalonFXConfiguration;

import edu.wpi.first.units.measure.AngularVelocity;

public final class FeederConstants {
    public static final String kFeederStateKey = "FeederState";

    public static final int kFeederMotorId = 3;

    public static final TalonFXConfiguration FeederConfig = new TalonFXConfiguration();

    public enum FeederState {
        ZERO(RotationsPerSecond.zero()),
        SCORE(RotationsPerSecond.of(100));

        public final AngularVelocity velocity;

        private FeederState(AngularVelocity velocity) {
            this.velocity = velocity;
        }
    }
}
