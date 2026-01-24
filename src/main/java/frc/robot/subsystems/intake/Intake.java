package frc.robot.subsystems.intake;

import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.intake.IntakeConstants.PivotState;
import frc.robot.subsystems.intake.IntakeConstants.RollerState;

public class Intake extends SubsystemBase {
    private final TalonFX m_leftPivotMotor = new TalonFX(IntakeConstants.kLeftPivotMotorID);
    private final TalonFX m_rightPivotMotor = new TalonFX(IntakeConstants.kRightPivotMotorID);
    private final TalonFX m_rollerMotor = new TalonFX(IntakeConstants.kRollerMotorID);

    private final MotionMagicVoltage m_request = new MotionMagicVoltage(0);

    private PivotState m_pivotState = PivotState.STOW;
    private RollerState m_rollerState = RollerState.ZERO;

    public Intake() {
        m_leftPivotMotor.getConfigurator().apply(IntakeConstants.kLeftPivotConfig);
        m_rightPivotMotor.getConfigurator().apply(IntakeConstants.kRightPivotConfig);
        m_rollerMotor.getConfigurator().apply(IntakeConstants.kRollerConfig);

        m_leftPivotMotor.setNeutralMode(NeutralModeValue.Brake);
        m_leftPivotMotor.setNeutralMode(NeutralModeValue.Brake);
    }

    public void setState(PivotState pivotState) {
        m_pivotState = pivotState;
    }

    public void setState(RollerState rollerState) {
        m_rollerState = rollerState;
    }

    private void setZero() {
        setState(RollerState.ZERO);
    }

    public Command stow() {
        return Commands.run(() -> setState(PivotState.STOW));
    }

    public Command ground() {
        return Commands.run(() -> setState(PivotState.GROUND));
    }

    public Command eject() {
        return Commands.runEnd(() -> setState(RollerState.EJECT), this::setZero, this);
    }

    public Command intake() {
        return Commands.runEnd(() -> setState(RollerState.INTAKE), this::setZero, this);
    }

    public Command spin() {
        return Commands.runEnd(() -> setState(RollerState.SPIN), this::setZero, this);
    }

    public Command score() {
        return Commands.runEnd(() -> setState(RollerState.SCORE), this::setZero, this);
    }

    public Command spinAndScore(double spinSeconds) {
        return spin().withTimeout(spinSeconds).andThen(score());
    }

    @Override
    public void periodic() {
        m_leftPivotMotor.setControl(m_request.withPosition(m_pivotState.position));
        m_rightPivotMotor.setControl(m_request.withPosition(m_pivotState.position));
        m_rollerMotor.setVoltage(m_rollerState.volts);

        SmartDashboard.putString(IntakeConstants.kPivotStateKey, m_pivotState.toString());
        SmartDashboard.putString(IntakeConstants.kRollerStateKey, m_rollerState.toString());
        SmartDashboard.putNumber(IntakeConstants.kLeftPivotPositionKey, m_leftPivotMotor.getPosition().getValueAsDouble());
        SmartDashboard.putNumber(IntakeConstants.kRightPivotPositionKey, m_rightPivotMotor.getPosition().getValueAsDouble());
    }
}
