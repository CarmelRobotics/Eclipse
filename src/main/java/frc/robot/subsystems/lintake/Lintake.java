package frc.robot.subsystems.lintake;

import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.lintake.LintakeConstants.PinionState;
import frc.robot.subsystems.lintake.LintakeConstants.RollerState;

public class Lintake extends SubsystemBase {
    private final TalonFX m_leaderPinionMotor = new TalonFX(LintakeConstants.kLeaderPinionMotorId);
    private final TalonFX m_followerPinionMotor = new TalonFX(LintakeConstants.kFollowerPinionMotorId);
    private final TalonFX m_rollerMotor = new TalonFX(LintakeConstants.kRollerMotorId);

    private final MotionMagicVoltage m_positionRequest = new MotionMagicVoltage(0);

    private PinionState m_pinionState = PinionState.STOW;
    private RollerState m_rollerState = RollerState.ZERO;

    public Lintake() {
        m_followerPinionMotor.setPosition(0);
        m_leaderPinionMotor.setPosition(0);

        m_leaderPinionMotor.getConfigurator().apply(LintakeConstants.LeaderPinionConfig);
        m_followerPinionMotor.getConfigurator().apply(LintakeConstants.FollowerPinionConfig);
        m_rollerMotor.getConfigurator().apply(LintakeConstants.RollerConfig);

        m_leaderPinionMotor.setNeutralMode(NeutralModeValue.Brake);
        m_followerPinionMotor.setNeutralMode(NeutralModeValue.Brake);

        m_rollerMotor.setNeutralMode(NeutralModeValue.Coast);
        //m_followerPinionMotor.setControl(new Follower(m_leaderPinionMotor.getDeviceID(), MotorAlignmentValue.Opposed));
    }

    public Command setState(PinionState pinionState) {
        return runOnce(() -> setPinionState(pinionState));
    }

    /** Imperative pinion setter for use inside command lambdas (the Command-returning
     *  overload above is a no-op if its result isn't scheduled). */
    public void setPinionState(PinionState pinionState) {
        m_pinionState = pinionState;
    }

    public void setState(RollerState rollerState) {
        m_rollerState = rollerState;
    }

    @Override
    public void periodic() {
        m_rollerMotor.setVoltage(m_rollerState.velocity);

        m_leaderPinionMotor.setControl(m_positionRequest.withPosition(m_pinionState.position));
         m_followerPinionMotor.setControl(m_positionRequest.withPosition(m_pinionState.position));
       // m_followerPinionMotor.setVoltage(m_leaderPinionMotor.getMotorVoltage().getValueAsDouble());

        SmartDashboard.putNumber(LintakeConstants.kRollerVoltageKey, m_rollerMotor.getMotorVoltage().getValueAsDouble());
        SmartDashboard.putString(LintakeConstants.kPinionStateKey, m_pinionState.toString());
        SmartDashboard.putString(LintakeConstants.kRollerStateKey, m_rollerState.toString());
        SmartDashboard.putNumber(LintakeConstants.kPinionPositionTargetKey, m_pinionState.position);
        SmartDashboard.putNumber(LintakeConstants.kLeaderPinionPositionKey, m_leaderPinionMotor.getPosition().getValueAsDouble());
        SmartDashboard.putNumber(LintakeConstants.kFollowerPinionPositionKey, m_followerPinionMotor.getPosition().getValueAsDouble());

    }
    
}
