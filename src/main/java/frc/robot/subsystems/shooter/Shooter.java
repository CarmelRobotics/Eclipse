package frc.robot.subsystems.shooter;

import static edu.wpi.first.units.Units.RotationsPerSecond;

import java.util.function.Supplier;

import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.MotorAlignmentValue;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.shooter.ShooterConstants.IndexerState;
import frc.robot.subsystems.shooter.ShooterConstants.PivotState;
import frc.robot.subsystems.shooter.ShooterConstants.ShooterState;

public class Shooter extends SubsystemBase {
    private final TalonFX m_indexerMotor = new TalonFX(ShooterConstants.kIndexerMotorId);

    private final TalonFX m_leaderPivotMotor = new TalonFX(ShooterConstants.kLeaderPivotMotorId);
    private final TalonFX m_followerPivotMotor = new TalonFX(ShooterConstants.kFollowerPivotMotorId);

    private final TalonFX m_leftLeaderShooterMotor = new TalonFX(ShooterConstants.kLeftLeaderShooterMotorId);
    private final TalonFX m_backLeftFollowerShooterMotor = new TalonFX(ShooterConstants.kBackLeftFollowerShooterMotorId);
    private final TalonFX m_rightFollowerShooterMotor = new TalonFX(ShooterConstants.kRightFollowerShooterMotorId);
    private final TalonFX m_backRightFollowerShooterMotor = new TalonFX(ShooterConstants.kBackRightFollowerShooterMotorId);

    private final VelocityVoltage m_velocityRequest = new VelocityVoltage(0);
    private final MotionMagicVoltage m_positionRequest = new MotionMagicVoltage(0);

    private final Supplier<Pose2d> m_poseSupplier;
    private Translation2d m_hubPosition;

    private PivotState m_pivotState = PivotState.STOW;
    private ShooterState m_shooterState = ShooterState.ZERO;
    private IndexerState m_indexerState = IndexerState.ZERO;

    public Shooter(Supplier<Pose2d> poseSupplier) {
        m_poseSupplier = poseSupplier;
        setHubPosition(DriverStation.getAlliance().orElse(Alliance.Blue));

        m_indexerMotor.getConfigurator().apply(ShooterConstants.IndexerConfig);

        m_leaderPivotMotor.getConfigurator().apply(ShooterConstants.PivotConfig);
        m_followerPivotMotor.getConfigurator().apply(ShooterConstants.PivotConfig);

        m_leftLeaderShooterMotor.getConfigurator().apply(ShooterConstants.ShooterConfig);
        m_backLeftFollowerShooterMotor.getConfigurator().apply(ShooterConstants.ShooterConfig);
        m_rightFollowerShooterMotor.getConfigurator().apply(ShooterConstants.ShooterConfig);
        m_backRightFollowerShooterMotor.getConfigurator().apply(ShooterConstants.ShooterConfig);

        m_followerPivotMotor.setControl(new Follower(m_leaderPivotMotor.getDeviceID(), MotorAlignmentValue.Opposed));

        m_backLeftFollowerShooterMotor.setControl(new Follower(m_leftLeaderShooterMotor.getDeviceID(), MotorAlignmentValue.Aligned));
        m_rightFollowerShooterMotor.setControl(new Follower(m_leftLeaderShooterMotor.getDeviceID(), MotorAlignmentValue.Opposed));
        m_backRightFollowerShooterMotor.setControl(new Follower(m_leftLeaderShooterMotor.getDeviceID(), MotorAlignmentValue.Opposed));
    }

    public void setState(IndexerState indexerState, PivotState pivotState, ShooterState shooterState) {
        m_indexerState = indexerState;
        m_pivotState = pivotState;
        m_shooterState = shooterState;
    }

    public void setState(IndexerState indexerState) {
        m_indexerState = indexerState;
    }

    public void setState(PivotState pivotState) {
        m_pivotState = pivotState;
    }

    public void setState(ShooterState shooterState) {
        m_shooterState = shooterState;
    }

    public void setHubPosition(Alliance alliance) {
        m_hubPosition = alliance == Alliance.Red
            ? ShooterConstants.kRedHubPosition
            : ShooterConstants.kBlueHubPosition;
    }

    private void shoot() {
        final double distance = m_poseSupplier.get().getTranslation().getDistance(m_hubPosition);
        final AngularVelocity velocity = RotationsPerSecond.of(ShooterConstants.ShooterVelocityRPSMap.get(distance));
        m_leftLeaderShooterMotor.setControl(m_velocityRequest.withVelocity(velocity));
    }

    private void pivot() {
        final double distance = m_poseSupplier.get().getTranslation().getDistance(m_hubPosition);
        final double position = ShooterConstants.PivotPositionMap.get(distance);
        m_leaderPivotMotor.setControl(m_positionRequest.withPosition(position));
    }

    @Override
    public void periodic() {
        switch (m_shooterState) {
            case ZERO -> m_leftLeaderShooterMotor.stopMotor();
            case SCORE -> shoot();
        }
        switch (m_pivotState) {
            case STOW -> m_leaderPivotMotor.setControl(m_positionRequest.withPosition(0));
            case SCORE -> pivot();
        }
        m_indexerMotor.setControl(m_velocityRequest.withVelocity(m_indexerState.velocity));

        SmartDashboard.putString(ShooterConstants.kShooterIndexerStateKey, m_indexerState.toString());
        SmartDashboard.putString(ShooterConstants.kShooterPivotStateKey, m_pivotState.toString());
        SmartDashboard.putNumber(ShooterConstants.kShooterPositionKey, m_leaderPivotMotor.getPosition().getValueAsDouble());
        SmartDashboard.putNumber(ShooterConstants.kShooterVelocityKey, m_leftLeaderShooterMotor.getVelocity().getValueAsDouble());
    }
}