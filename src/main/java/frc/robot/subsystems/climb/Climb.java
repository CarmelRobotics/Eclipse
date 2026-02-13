package frc.robot.subsystems.climb;

import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.climb.ClimbConstants.ClimbState;

public class Climb extends SubsystemBase {
    private final TalonFX m_climbMotor = new TalonFX(ClimbConstants.kClimbMotorId);
    private final MotionMagicVoltage m_positionRequest = new MotionMagicVoltage(0);
    private ClimbState m_climbState = ClimbState.STOW;

    public Climb() {
        m_climbMotor.getConfigurator().apply(ClimbConstants.ClimbMotorConfig);
        m_climbMotor.setNeutralMode(NeutralModeValue.Brake);
    }

    public void setState(ClimbState climbState) {
        m_climbState = climbState;
    }

    @Override
    public void periodic() {
        m_climbMotor.setControl(m_positionRequest.withPosition(m_climbState.position));

        SmartDashboard.putString(ClimbConstants.kClimbStateKey, m_climbState.toString());
        SmartDashboard.putNumber(ClimbConstants.kClimbPositionKey, m_climbMotor.getPosition().getValueAsDouble());
    }
}
