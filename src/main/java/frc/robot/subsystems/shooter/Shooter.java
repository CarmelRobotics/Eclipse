package frc.robot.subsystems.shooter;

import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.MotorAlignmentValue;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.shooter.ShooterConstants.KickerState;
import frc.robot.subsystems.shooter.ShooterConstants.PivotState;
import frc.robot.subsystems.shooter.ShooterConstants.ShooterState;

public class Shooter extends SubsystemBase {
    private final TalonFX m_leaderPivotMotor = new TalonFX(ShooterConstants.kLeaderPivotMotorId);
    private final TalonFX m_followerPivotMotor = new TalonFX(ShooterConstants.kFollowerPivotMotorId);

    private final TalonFX m_frontLeftLeaderRollerMotor = new TalonFX(ShooterConstants.kLeftLeaderShooterMotorId);
    private final TalonFX m_backLeftFollowerRollerMotor = new TalonFX(ShooterConstants.kBackLeftFollowerShooterMotorId);
    private final TalonFX m_frontRightFollowerRollerMotor = new TalonFX(ShooterConstants.kRightFollowerShooterMotorId);
    private final TalonFX m_backRightFollowerRollerMotor = new TalonFX(ShooterConstants.kBackRightFollowerShooterMotorId);

    private final TalonFX m_kickerMotor = new TalonFX(ShooterConstants.kKickerMotorId);

    private final StatusSignal<Boolean> m_pivotAtTarget = m_leaderPivotMotor.getMotionMagicAtTarget();
    private final StatusSignal<Angle> m_pivotAngle = m_leaderPivotMotor.getPosition();
    private final StatusSignal<Double> m_rollerError = m_frontLeftLeaderRollerMotor.getClosedLoopError();
    private final StatusSignal<AngularVelocity> m_rollerVelocity = m_frontLeftLeaderRollerMotor.getVelocity();
    private final StatusSignal<AngularVelocity> m_kickerVelocity = m_kickerMotor.getVelocity();

    private final VelocityVoltage m_velocityRequest = new VelocityVoltage(0);
    private final MotionMagicVoltage m_positionRequest = new MotionMagicVoltage(0);

    public Shooter() {
        m_leaderPivotMotor.setPosition(0);
        m_followerPivotMotor.setPosition(0);

        m_leaderPivotMotor.getConfigurator().apply(ShooterConstants.PivotTalonFXConfigs);
        m_followerPivotMotor.getConfigurator().apply(ShooterConstants.PivotTalonFXConfigs);

        m_frontLeftLeaderRollerMotor.getConfigurator().apply(ShooterConstants.RollerTalonFXConfigs);
        m_backLeftFollowerRollerMotor.getConfigurator().apply(ShooterConstants.RollerTalonFXConfigs);
        m_frontRightFollowerRollerMotor.getConfigurator().apply(ShooterConstants.RollerTalonFXConfigs);
        m_backRightFollowerRollerMotor.getConfigurator().apply(ShooterConstants.RollerTalonFXConfigs);

        m_kickerMotor.getConfigurator().apply(ShooterConstants.KickerTalonFXConfigs);

        m_followerPivotMotor.setControl(new Follower(m_leaderPivotMotor.getDeviceID(), MotorAlignmentValue.Opposed));

        m_backLeftFollowerRollerMotor.setControl(new Follower(m_frontLeftLeaderRollerMotor.getDeviceID(), MotorAlignmentValue.Aligned));
        m_frontRightFollowerRollerMotor.setControl(new Follower(m_frontLeftLeaderRollerMotor.getDeviceID(), MotorAlignmentValue.Opposed));
        m_backRightFollowerRollerMotor.setControl(new Follower(m_frontLeftLeaderRollerMotor.getDeviceID(), MotorAlignmentValue.Opposed));
    }

    public boolean pivotHasReachedTarget() {
        return m_pivotAtTarget.getValue().booleanValue();
    }

    public boolean rollerHasReachedTarget() {
        return Math.abs(m_rollerError.getValueAsDouble()) <= ShooterConstants.kRollerVelocityErrorTolerance;
    }

    public Command setState(PivotState pivotState) {
        return runFromPivotState(pivotState);
    }

    public Command setState(ShooterState rollerState) {
        return runFromRollerState(rollerState);
    }

    public Command setState(KickerState kickerState) {
        return runFromKickerState(kickerState);
    }

    public Command setState(PivotState pivotState, ShooterState rollerState, KickerState kickerState) {
        return runFromPivotState(pivotState)
                .andThen(runFromRollerState(rollerState))
                .andThen(runFromKickerState(kickerState));
    }

    private Command runPivotToAngle(Angle angle) {
        return runOnce(() -> {
            m_leaderPivotMotor.setControl(m_positionRequest.withPosition(angle));
        });
    }

    private Command runRollerToVelocity(AngularVelocity velocity) {
        return runOnce(() -> {
            m_frontLeftLeaderRollerMotor.setControl(m_velocityRequest.withVelocity(velocity));
        });
    }

    private Command runKickerToVelocity(AngularVelocity velocity) {
        return runOnce(() -> {
            m_kickerMotor.setControl(m_velocityRequest.withVelocity(velocity));
        });
    }

    private Command runFromPivotState(PivotState pivotState) {
        return switch (pivotState) {
            case STOW -> runPivotToAngle(ShooterConstants.kPivotStowAngle);
            case LOB -> runPivotToAngle(ShooterConstants.kPivotLobAngle);
            case SEND -> runPivotToAngle(ShooterConstants.kPivotSendAngle);
            case SCORE -> runPivotToAngle(ShooterConstants.kPivotScoreAngle);
        };
    }

    private Command runFromRollerState(ShooterState rollerState) {
        return switch (rollerState) {
            case IDLE -> runRollerToVelocity(ShooterConstants.kRollerIdleVelocity);
            case LOB -> runRollerToVelocity(ShooterConstants.kRollerLobVelocity);
            case SEND -> runRollerToVelocity(ShooterConstants.kRollerSendVelocity);
            case SCORE -> runRollerToVelocity(ShooterConstants.kRollerScoreVelocity);
        };
    }

    private Command runFromKickerState(KickerState kickerState) {
        return switch (kickerState) {
            case IDLE -> runKickerToVelocity(ShooterConstants.kKickerIdleVelocity);
            case FEED -> Commands.waitUntil(() -> { return pivotHasReachedTarget() && rollerHasReachedTarget(); })
                    .withTimeout(ShooterConstants.kFeedTimeout)
                    .andThen(runKickerToVelocity(ShooterConstants.kkickerFeedVelocity));
        };
    }

    @Override
    public void periodic() {
        SmartDashboard.putBoolean(ShooterConstants.kShooterPivotAtTargetKey, pivotHasReachedTarget());
        SmartDashboard.putNumber(ShooterConstants.kShooterPivotPositionKey, m_pivotAngle.getValueAsDouble());
        SmartDashboard.putBoolean(ShooterConstants.kShooterRollerAtTargetKey, rollerHasReachedTarget());
        SmartDashboard.putNumber(ShooterConstants.kShooterRollerVelocityKey, m_rollerVelocity.getValueAsDouble());
        SmartDashboard.putNumber(ShooterConstants.kShooterKickerVelocityKey, m_kickerVelocity.getValueAsDouble());
    }
}