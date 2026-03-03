package frc.robot.subsystems.feeder;

import org.ejml.ops.MatrixFeatures_F;

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
        if(m_feederState == FeederState.SCORE){
            this.m_feederMotor.set(100);
        } else {
            this.m_feederMotor.set(0);
        }
        SmartDashboard.putNumber("Feeder voltage", this.m_feederMotor.getMotorVoltage().getValueAsDouble());
        SmartDashboard.putNumber("Feeder target voltage", m_feederState.velocity*20);
        SmartDashboard.putString(FeederConstants.kFeederStateKey, m_feederState.toString());
    }
}
