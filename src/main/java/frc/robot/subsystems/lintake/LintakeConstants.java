package frc.robot.subsystems.lintake;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.Slot1Configs;
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

        // Slot 0: stiff gains for actual moves (stow, agitate, deploy) where tracking
        // authority matters -- e.g. clearing the trench bar on a stow.
        private static final Slot0Configs Slot0Configs = new Slot0Configs()
            .withKA(kA).withKD(kD).withKI(kI).withKP(kP).withKS(kS).withKV(kV);

        // Slot 1: COMPLIANT gains, used only while holding GROUND. A front impact from
        // another robot back-drives the intake inward against a weak spring instead of
        // a rigid position hold (which put the whole hit through the pinion gearing);
        // when the hit ends, the same weak P walks it back out to GROUND on its own.
        // If the intake won't stay planted on the carpet while driving, raise this kP
        // a little; if hits still feel harsh, lower it.
        private static final double kCompliantP = 0.15;
        private static final Slot1Configs Slot1Configs = new Slot1Configs()
            .withKP(kCompliantP).withKI(0).withKD(0).withKS(0).withKV(0).withKA(0);

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
    
    // Brownout budget: neither the pinions nor the roller had ANY current limit, and the
    // shooting-time agitation pump accelerates both pinions every 0.3 s. 30 A supply each
    // bounds the whole intake at ~90 A worst case without slowing normal moves.
    private static final CurrentLimitsConfigs kIntakeCurrentLimits = new CurrentLimitsConfigs()
        .withSupplyCurrentLimitEnable(true).withSupplyCurrentLimit(30);

    public static final TalonFXConfiguration LeaderPinionConfig = new TalonFXConfiguration()
        .withSlot0(PinionConfigs.Slot0Configs).withSlot1(PinionConfigs.Slot1Configs)
        .withMotionMagic(PinionConfigs.MotionMagicConfigs)
        .withCurrentLimits(kIntakeCurrentLimits)
        .withMotorOutput(new MotorOutputConfigs().withInverted(InvertedValue.Clockwise_Positive));
    public static final TalonFXConfiguration FollowerPinionConfig = new TalonFXConfiguration()
        .withSlot0(PinionConfigs.Slot0Configs).withSlot1(PinionConfigs.Slot1Configs)
        .withMotionMagic(PinionConfigs.MotionMagicConfigs)
        .withCurrentLimits(kIntakeCurrentLimits)
        .withMotorOutput(new MotorOutputConfigs().withInverted(InvertedValue.CounterClockwise_Positive));

    public static final TalonFXConfiguration RollerConfig = new TalonFXConfiguration()
        .withSlot0(RollerConfigs.Slot0Configs).withCurrentLimits(kIntakeCurrentLimits);

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
