package frc.robot.subsystems.lintake;

import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.MotorAlignmentValue;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.lintake.LintakeConstants.PinionState;
import frc.robot.subsystems.lintake.LintakeConstants.RollerState;

public class Lintake extends SubsystemBase {
    private final TalonFX m_leaderPinionMotor = new TalonFX(LintakeConstants.kLeaderPinionMotorId);
    private final TalonFX m_followerPinionMotor = new TalonFX(LintakeConstants.kFollowerPinionMotorId);
    private final TalonFX m_rollerMotor = new TalonFX(LintakeConstants.kRollerMotorId);

    private final StatusSignal<Boolean> m_pinionAtTarget = m_leaderPinionMotor.getMotionMagicAtTarget();
    private final StatusSignal<Angle> m_pinionAngle = m_leaderPinionMotor.getPosition();
    private final StatusSignal<Voltage> m_rollerVoltage = m_rollerMotor.getMotorVoltage();

    private final VoltageOut m_voltageRequest = new VoltageOut(0);
    private final MotionMagicVoltage m_positionRequest = new MotionMagicVoltage(0);

    public Lintake() {
        m_followerPinionMotor.setPosition(0);
        m_leaderPinionMotor.setPosition(0);

        m_leaderPinionMotor.getConfigurator().apply(LintakeConstants.PinionTalonFXConfigs);
        m_followerPinionMotor.getConfigurator().apply(LintakeConstants.PinionTalonFXConfigs);
        m_rollerMotor.getConfigurator().apply(LintakeConstants.RollerTalonFXConfigs);

        m_followerPinionMotor.setControl(new Follower(m_leaderPinionMotor.getDeviceID(), MotorAlignmentValue.Opposed));
    }

    public boolean pinionHasReachedTarget() {
        return m_pinionAtTarget.getValue().booleanValue();
    }

    public Command setState(PinionState pinionState) {
        return runFromPinionState(pinionState);
    }
    
    public Command setState(RollerState rollerState) {
        return runFromRollerState(rollerState);
    }

    private Command runPinionToAngle(Angle angle) {
        return runOnce(() -> {
            m_leaderPinionMotor.setControl(m_positionRequest.withPosition(angle));
        });
    }

    private Command runRollerToVoltage(Voltage voltage) {
        return runOnce(() -> {
            m_rollerMotor.setControl(m_voltageRequest.withOutput(voltage));
        });
    }

    private Command runFromPinionState(PinionState pinionState) {
        return switch (pinionState) {
            case STOW -> runPinionToAngle(LintakeConstants.kStowAngle);
            case AGITATE -> defer(() -> {
                final var lastAngle = m_pinionAngle.getValue();

                return Commands.repeatingSequence(
                        runPinionToAngle(LintakeConstants.kAgitateStowAngle),
                        Commands.waitUntil(this::pinionHasReachedTarget)
                                .withTimeout(LintakeConstants.kAgitateTimeout),
                        runPinionToAngle(LintakeConstants.kDeployAngle),
                        Commands.waitUntil(this::pinionHasReachedTarget)
                                .withTimeout(LintakeConstants.kAgitateTimeout)
                ).finallyDo(() -> {
                    m_leaderPinionMotor.setControl(m_positionRequest.withPosition(lastAngle));   
                });
            });
            case DEPLOY -> runPinionToAngle(LintakeConstants.kDeployAngle);
        };
    }

    private Command runFromRollerState(RollerState rollerState) {
        return switch (rollerState) {
            case IDLE -> runRollerToVoltage(LintakeConstants.kIdleVoltage);
            case EJECT -> runRollerToVoltage(LintakeConstants.kEjectVoltage);
            case INTAKE -> runRollerToVoltage(LintakeConstants.kIntakeVoltage);
        };
    }

    @Override
    public void periodic() {
        SmartDashboard.putNumber(LintakeConstants.kRollerVoltageKey, m_rollerVoltage.getValueAsDouble());
        SmartDashboard.putNumber(LintakeConstants.kPinionPositionKey, m_pinionAngle.getValueAsDouble());
        SmartDashboard.putBoolean(LintakeConstants.kPinionAtTargetKey, m_pinionAtTarget.getValue().booleanValue());
    }
}
