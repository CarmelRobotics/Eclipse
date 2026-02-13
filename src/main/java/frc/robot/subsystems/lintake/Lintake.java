package frc.robot.subsystems.lintake;

import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.MotorAlignmentValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.lintake.LintakeConstants.PinionState;
import frc.robot.subsystems.lintake.LintakeConstants.RollerState;

public class Lintake extends SubsystemBase {
    private final TalonFX m_leaderPinionMotor = new TalonFX(LintakeConstants.kLeaderPinionMotorId);
    private final TalonFX m_followerPinionMotor = new TalonFX(LintakeConstants.kFollowerPinionMotorId);
    private final TalonFX m_rollerMotor = new TalonFX(LintakeConstants.kRollerMotorId);

    private final VelocityVoltage m_velocityRequest = new VelocityVoltage(0);
    private final MotionMagicVoltage m_positionRequest = new MotionMagicVoltage(0);

    private PinionState m_pinionState = PinionState.STOW;
    private RollerState m_rollerState = RollerState.ZERO;

    public Lintake() {
        m_leaderPinionMotor.getConfigurator().apply(LintakeConstants.PinionConfig);
        m_followerPinionMotor.getConfigurator().apply(LintakeConstants.PinionConfig);
        m_rollerMotor.getConfigurator().apply(LintakeConstants.RollerConfig);

        m_leaderPinionMotor.setNeutralMode(NeutralModeValue.Brake);
        m_followerPinionMotor.setNeutralMode(NeutralModeValue.Brake);

        m_followerPinionMotor.setControl(new Follower(m_leaderPinionMotor.getDeviceID(), MotorAlignmentValue.Opposed));
    }

    public void setState(PinionState pinionState) {
        m_pinionState = pinionState;
    }

    public void setState(RollerState rollerState) {
        m_rollerState = rollerState;
    }

    @Override
    public void periodic() {
        m_leaderPinionMotor.setControl(m_positionRequest.withPosition(m_pinionState.position));
        m_rollerMotor.setControl(m_velocityRequest.withVelocity(m_rollerState.velocity));

        SmartDashboard.putString(LintakeConstants.kPinionStateKey, m_pinionState.toString());
        SmartDashboard.putString(LintakeConstants.kRollerStateKey, m_rollerState.toString());
        SmartDashboard.putNumber(LintakeConstants.kLeaderPinionPositionKey, m_leaderPinionMotor.getPosition().getValueAsDouble());
        SmartDashboard.putNumber(LintakeConstants.kFollowerPinionPositionKey, m_followerPinionMotor.getPosition().getValueAsDouble());
    }
}
