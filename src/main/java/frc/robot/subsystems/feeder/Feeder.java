package frc.robot.subsystems.feeder;

import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.feeder.FeederConstants.FeederState;

public class Feeder extends SubsystemBase {
    private final TalonFX m_feederMotor = new TalonFX(FeederConstants.kFeederMotorId);
    private final VelocityVoltage m_velocityRequest = new VelocityVoltage(0);
    private FeederState m_feederState = FeederState.ZERO;

    public Feeder() {
        m_feederMotor.getConfigurator().apply(FeederConstants.FeederConfig);
    }

    public void setState(FeederState indexState) {
        m_feederState = indexState;
    }

    @Override
    public void periodic() {
        m_feederMotor.setControl(m_velocityRequest.withVelocity(m_feederState.velocity));

        SmartDashboard.putString(FeederConstants.kFeederStateKey, m_feederState.toString());
    }
}
