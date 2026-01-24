package frc.robot.subsystems.intake;

import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.signals.InvertedValue;

public final class IntakeConstants {
    public static final String kPivotStateKey = "PivotState";
    public static final String kRollerStateKey = "RollerState";
    public static final String kLeftPivotPositionKey = "LeftPivotPosition";
    public static final String kRightPivotPositionKey = "RightPivotPosition";

    public static final int kLeftPivotMotorID = 1;
    public static final int kRightPivotMotorID = 2;
    public static final int kRollerMotorID = 3;

    private static final double kA = 0.01;
    private static final double kD = 0.1;
    private static final double kI = 0;
    private static final double kP = 4.8;
    private static final double kS = 0.25;
    private static final double kV = 0.12;

    private static final double kMotionMagicCruiseVelocity = 80;
    private static final double kMotionMagicAcceleration = 160;
    private static final double kMotionMagicJerk = 1600;

    private static final Slot0Configs kPivotSlot0Configs = new Slot0Configs()
        .withKA(kA).withKD(kD).withKI(kI).withKP(kP).withKS(kS).withKV(kV);

    private static final MotionMagicConfigs kPivotMotionMagicConfigs = new MotionMagicConfigs()
        .withMotionMagicCruiseVelocity(kMotionMagicCruiseVelocity)
        .withMotionMagicAcceleration(kMotionMagicAcceleration)
        .withMotionMagicJerk(kMotionMagicJerk);

    public static final TalonFXConfiguration kRightPivotConfig = new TalonFXConfiguration()
        .withSlot0(kPivotSlot0Configs).withMotionMagic(kPivotMotionMagicConfigs)
        .withMotorOutput(new MotorOutputConfigs().withInverted(InvertedValue.CounterClockwise_Positive));

    public static final TalonFXConfiguration kLeftPivotConfig = new TalonFXConfiguration()
        .withSlot0(kPivotSlot0Configs).withMotionMagic(kPivotMotionMagicConfigs)
        .withMotorOutput(new MotorOutputConfigs().withInverted(InvertedValue.Clockwise_Positive));
    
    public static final TalonFXConfiguration kRollerConfig = new TalonFXConfiguration();

    public enum PivotState {
        STOW(0),
        GROUND(30);

        public final double position;

        private PivotState(double position) {
            this.position = position;
        }
    }

    public enum RollerState {
        ZERO(0),
        EJECT(-12),
        INTAKE(12),
        SPIN(10),
        SCORE(10);

        public final double volts;

        private RollerState(double volts) {
            this.volts = volts;
        }
    }
}
