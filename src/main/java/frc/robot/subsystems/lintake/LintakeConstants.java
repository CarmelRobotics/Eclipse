package frc.robot.subsystems.lintake;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.signals.InvertedValue;

public final class LintakeConstants {
    public static final String kPinionStateKey = "PinionState";
    public static final String kRollerStateKey = "RollerState";
    public static final String kRollerVoltageKey = "RollerVoltage";
    public static final String kLeaderPinionPositionKey = "LeaderPinionPosition";
    public static final String kFollowerPinionPositionKey = "FollowerPinionPosition";
    public static final String kPinionPositionTargetKey = "PinionPositionTarget";


    public static final int kLeaderPinionMotorId = 35;
    public static final int kFollowerPinionMotorId = 2;
    public static final int kRollerMotorId = 45;

    private static final class PinionConfigs {
        private static final double kA = 0.01;
        private static final double kD = 0.15;
        private static final double kI = 0;
        private static final double kP = 1.25;
        private static final double kS = 0;
        private static final double kV = 0.25;

        private static final double kMotionMagicCruiseVelocity = 55;
        private static final double kMotionMagicAcceleration = 135;
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
    
    // Pinion current limits.
    //   Stator 15 A: the COMPLIANCE knob. Stator current is proportional to torque, so
    //     capping it caps how hard the pinions can ever push. Gearing is 3:1 belt -> 10T
    //     10DP pinion (1" pitch dia) on the rack, and a Kraken makes ~0.0194 N*m/A, so
    //     15 A x 2 motors x 3 : (0.5" radius) ~= 31 lbf of hold/resist force. Drive the
    //     intake into a wall harder than that and it back-drives inward instead of
    //     stalling the motors; it re-extends when the wall is gone. Deploy/retract only
    //     needs ~5-15 lbf, so 15 A still moves it crisply. Raise for a firmer hold that
    //     yields to less; lower to slide in on lighter contact.
    //   Supply 30 A: brownout budget, unchanged.
    private static final CurrentLimitsConfigs kPinionCurrentLimits = new CurrentLimitsConfigs()
        .withStatorCurrentLimitEnable(true).withStatorCurrentLimit(15)
        .withSupplyCurrentLimitEnable(true).withSupplyCurrentLimit(30);

    // Roller keeps just the brownout supply cap (no compliance role).
    private static final CurrentLimitsConfigs kRollerCurrentLimits = new CurrentLimitsConfigs()
        .withSupplyCurrentLimitEnable(true).withSupplyCurrentLimit(30);

    public static final TalonFXConfiguration LeaderPinionConfig = new TalonFXConfiguration()
        .withSlot0(PinionConfigs.Slot0Configs)
        .withMotionMagic(PinionConfigs.MotionMagicConfigs)
        .withCurrentLimits(kPinionCurrentLimits)
        .withMotorOutput(new MotorOutputConfigs().withInverted(InvertedValue.Clockwise_Positive));
    public static final TalonFXConfiguration FollowerPinionConfig = new TalonFXConfiguration()
        .withSlot0(PinionConfigs.Slot0Configs)
        .withMotionMagic(PinionConfigs.MotionMagicConfigs)
        .withCurrentLimits(kPinionCurrentLimits)
        .withMotorOutput(new MotorOutputConfigs().withInverted(InvertedValue.CounterClockwise_Positive));

    public static final TalonFXConfiguration RollerConfig = new TalonFXConfiguration()
        .withSlot0(RollerConfigs.Slot0Configs).withCurrentLimits(kRollerCurrentLimits);

    public enum PinionState {
        STOW(-4),
        AGITATE(-7.5),
        GROUND(-9.75);

        public final double position;

        private PinionState(double position) {
            this.position = position;
        }
    }

    public enum RollerState {
        ZERO(0),
        EJECT(-12),
        INTAKE(12);

        public final double volts;

        private RollerState(double volts) {
            this.volts = volts;
        }
    }
}
