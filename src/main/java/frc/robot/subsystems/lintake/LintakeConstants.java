package frc.robot.subsystems.lintake;

import static edu.wpi.first.units.Units.Rotations;
import static edu.wpi.first.units.Units.Seconds;
import static edu.wpi.first.units.Units.Volts;

import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Time;
import edu.wpi.first.units.measure.Voltage;

public final class LintakeConstants {
    public static final String kRollerVoltageKey = "Lintake/RollerVoltage";
    public static final String kPinionPositionKey = "Lintake/PinionPosition";
    public static final String kPinionAtTargetKey = "Lintake/PinionAtTarget";

    public static final int kLeaderPinionMotorId = 35;
    public static final int kFollowerPinionMotorId = 2;
    public static final int kRollerMotorId = 45;

    private static final class PinionConfigs {
        private static final Slot0Configs Slot0Configs = new Slot0Configs()
            .withKA(0.01)
            .withKD(0.15)
            .withKI(0)
            .withKP(1.25)
            .withKS(0)
            .withKV(0.25);

        private static final MotionMagicConfigs MotionMagicConfigs = new MotionMagicConfigs()
            .withMotionMagicCruiseVelocity(55)
            .withMotionMagicAcceleration(135)
            .withMotionMagicJerk(1600);
    }

    private static final class RollerConfigs {
        private static final Slot0Configs Slot0Configs = new Slot0Configs()
            .withKS(0.1)
            .withKV(0.12)
            .withKP(0.11)
            .withKI(0)
            .withKD(0.125);
    }
    
    public static final TalonFXConfiguration PinionTalonFXConfigs = new TalonFXConfiguration()
        .withSlot0(PinionConfigs.Slot0Configs).withMotionMagic(PinionConfigs.MotionMagicConfigs)
        .withMotorOutput(new MotorOutputConfigs().withNeutralMode(NeutralModeValue.Brake));

    public static final TalonFXConfiguration RollerTalonFXConfigs = new TalonFXConfiguration().withSlot0(RollerConfigs.Slot0Configs);

    public static final Angle kStowAngle = Rotations.of(-4);
    public static final Angle kAgitateStowAngle = Rotations.of(-7.5);
    public static final Angle kDeployAngle = Rotations.of(-12); 

    public static final Time kAgitateTimeout = Seconds.of(0.5);

    public static final Voltage kIdleVoltage = Volts.zero();
    public static final Voltage kEjectVoltage = Volts.of(-12);
    public static final Voltage kIntakeVoltage = Volts.of(12);

    public enum PinionState {
        STOW,
        AGITATE,
        DEPLOY;
    }

    public enum RollerState {
        IDLE,
        EJECT,
        INTAKE;
    }
}
