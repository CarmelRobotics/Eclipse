package frc.robot.subsystems.lintake;

import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.hardware.ParentDevice;
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
    }

    /** Every CTRE device this subsystem owns, for the SystemsCheck fault monitor. */
    public java.util.Map<String, ParentDevice> getDevices() {
        java.util.Map<String, ParentDevice> devices = new java.util.LinkedHashMap<>();
        devices.put("Lintake/PinionLeader", m_leaderPinionMotor);
        devices.put("Lintake/PinionFollower", m_followerPinionMotor);
        devices.put("Lintake/Roller", m_rollerMotor);
        return devices;
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
        m_rollerMotor.setVoltage(m_rollerState.volts);

        // Compliance is handled by the pinion stator current limit (see
        // LintakeConstants.kPinionCurrentLimits), not by gains -- so a single stiff
        // slot 0 drives every state and the intake still back-drives when it hits a wall.
        m_leaderPinionMotor.setControl(m_positionRequest.withPosition(m_pinionState.position));
        m_followerPinionMotor.setControl(m_positionRequest.withPosition(m_pinionState.position));

        SmartDashboard.putNumber(LintakeConstants.kRollerVoltageKey, m_rollerMotor.getMotorVoltage().getValueAsDouble());
        SmartDashboard.putString(LintakeConstants.kPinionStateKey, m_pinionState.toString());
        SmartDashboard.putString(LintakeConstants.kRollerStateKey, m_rollerState.toString());
        SmartDashboard.putNumber(LintakeConstants.kPinionPositionTargetKey, m_pinionState.position);
        SmartDashboard.putNumber(LintakeConstants.kLeaderPinionPositionKey, m_leaderPinionMotor.getPosition().getValueAsDouble());
        SmartDashboard.putNumber(LintakeConstants.kFollowerPinionPositionKey, m_followerPinionMotor.getPosition().getValueAsDouble());

    }
    
}
