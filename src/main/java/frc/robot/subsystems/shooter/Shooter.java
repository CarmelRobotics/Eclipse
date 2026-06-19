package frc.robot.subsystems.shooter;

import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.MotorAlignmentValue;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.drive.CommandSwerveDrivetrain;
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

    private PivotState m_pivotState = PivotState.STOW;
    private ShooterState m_shooterState = ShooterState.ZERO;
    private IndexerState m_indexerState = IndexerState.ZERO;
    private double m_targetPivotPosition = ShooterConstants.kStowPivotPosition;
    private double m_targetShooterRps = ShooterConstants.kIdleShooterRps;

    private final CommandSwerveDrivetrain m_drive;

    public Shooter(CommandSwerveDrivetrain drive) {
        m_drive = drive;
        SmartDashboard.putNumber(ShooterConstants.kPivotOffsetKey, 0);
        SmartDashboard.putNumber(ShooterConstants.kShooterRpsOffsetKey, 0);
        SmartDashboard.putNumber(ShooterConstants.kTimeOfFlightOffsetKey, 0);

        m_indexerMotor.getConfigurator().apply(ShooterConstants.IndexerConfig);

        m_leaderPivotMotor.getConfigurator().apply(ShooterConstants.LeaderPivotConfig);
        m_followerPivotMotor.getConfigurator().apply(ShooterConstants.FollowerPivotConfig);

        m_leaderPivotMotor.setPosition(0);
        m_followerPivotMotor.setPosition(0);

        m_leftLeaderShooterMotor.getConfigurator().apply(ShooterConstants.LeftShooterConfig);
        m_backLeftFollowerShooterMotor.getConfigurator().apply(ShooterConstants.LeftShooterConfig);
        m_rightFollowerShooterMotor.getConfigurator().apply(ShooterConstants.RightShooterConfig);
        m_backRightFollowerShooterMotor.getConfigurator().apply(ShooterConstants.RightShooterConfig);

        

        //m_followerPivotMotor.setControl(new Follower(m_leaderPivotMotor.getDeviceID(), MotorAlignmentValue.Opposed));

        m_backLeftFollowerShooterMotor.setControl(new Follower(m_leftLeaderShooterMotor.getDeviceID(), MotorAlignmentValue.Aligned));
        m_rightFollowerShooterMotor.setControl(new Follower(m_leftLeaderShooterMotor.getDeviceID(), MotorAlignmentValue.Opposed));
        m_backRightFollowerShooterMotor.setControl(new Follower(m_leftLeaderShooterMotor.getDeviceID(), MotorAlignmentValue.Opposed));

        m_indexerMotor.optimizeBusUtilization();
        m_leaderPivotMotor.optimizeBusUtilization();
        m_followerPivotMotor.optimizeBusUtilization();
        m_leftLeaderShooterMotor.optimizeBusUtilization();
        m_backLeftFollowerShooterMotor.optimizeBusUtilization();
        m_rightFollowerShooterMotor.optimizeBusUtilization();
        m_backRightFollowerShooterMotor.optimizeBusUtilization();

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

    public Command zero(){
        return Commands.runOnce(()->{
            this.m_followerPivotMotor.setPosition(0);
            this.m_leaderPivotMotor.setPosition(0);
        });
    }

    public double getAvgShooterCurrentDraw(){
        var sum = m_leftLeaderShooterMotor.getSupplyCurrent().getValueAsDouble() + m_backLeftFollowerShooterMotor.getSupplyCurrent().getValueAsDouble() + m_rightFollowerShooterMotor.getSupplyCurrent().getValueAsDouble() + m_backRightFollowerShooterMotor.getSupplyCurrent().getValueAsDouble();
        return sum / 4;

    }

    public boolean readyToShoot() {
        boolean shotInRange = m_drive.getShotDistance() <= ShooterConstants.kMaxShotDistanceMeters;
        boolean aimed = Math.abs(m_drive.getHubHeadingError().getDegrees()) <= ShooterConstants.kHeadingReadyToleranceDegrees;
        boolean shooterReady = Math.abs(m_leftLeaderShooterMotor.getVelocity().getValueAsDouble() - m_targetShooterRps)
            <= ShooterConstants.kShooterReadyToleranceRps;
        boolean pivotReady = Math.abs(m_leaderPivotMotor.getPosition().getValueAsDouble() - m_targetPivotPosition)
            <= ShooterConstants.kPivotReadyToleranceRotations;

        return shotInRange && aimed && shooterReady && pivotReady;
    }

    @Override
    public void periodic() {
        /*
        switch (m_shooterState) {
            case ZERO -> m_leftLeaderShooterMotor.stopMotor();
            case SCORE -> shoot();
        }
        switch (m_pivotState) {
            case STOW -> m_leaderPivotMotor.setControl(m_positionRequest.withPosition(0));
            case SCORE -> pivot();
        }

        m_indexerMotor.setControl(m_velocityRequest.withVelocity(m_indexerState.velocity));
        */

        double shotDistance = m_drive.getShotDistance();
        double pivotOffset = SmartDashboard.getNumber(ShooterConstants.kPivotOffsetKey, 0);
        double shooterRpsOffset = SmartDashboard.getNumber(ShooterConstants.kShooterRpsOffsetKey, 0);
        m_targetPivotPosition = ShooterConstants.getScorePivotPosition(shotDistance) + pivotOffset;
        m_targetShooterRps = ShooterConstants.getScoreShooterRps(shotDistance) + shooterRpsOffset;

        SmartDashboard.putNumber("shot compensated distance", shotDistance);
        SmartDashboard.putNumber("shot time of flight", m_drive.getShotTimeOfFlightSeconds());
        SmartDashboard.putNumber("interpolated pivot position", m_targetPivotPosition);
        SmartDashboard.putNumber("interpolated shooter rps", m_targetShooterRps);
        SmartDashboard.putBoolean("ready to shoot", readyToShoot());


        switch (m_pivotState) {
            
            case STOW -> {
                 m_leaderPivotMotor.setControl(m_positionRequest.withPosition(ShooterConstants.kStowPivotPosition));
                 m_followerPivotMotor.setControl(m_positionRequest.withPosition(ShooterConstants.kStowPivotPosition));
            }
            case SCORE -> {
                m_leaderPivotMotor.setControl(m_positionRequest.withPosition(m_targetPivotPosition));
                m_followerPivotMotor.setControl(m_positionRequest.withPosition(m_targetPivotPosition));
            }
            case LOB -> {
                // m_leaderPivotMotor.setControl(m_positionRequest.withPosition(0.45));
                // m_followerPivotMotor.setControl(m_positionRequest.withPosition(0.45));
            }
        }
        

        switch (m_shooterState) {
            case ZERO -> {
                setShooterVelocity(ShooterConstants.kIdleShooterRps);
            }
            case SCORE -> {
                setShooterVelocity(m_targetShooterRps);
            }
             case LOB -> {
                setShooterVelocity(ShooterConstants.kLobShooterRps);
            }
            case SEND ->{
                setShooterVelocity(ShooterConstants.kSendShooterRps);
            }
            
        }
        
        m_indexerMotor.setVoltage(m_indexerState.volts);
        SmartDashboard.putNumber("shooter current draw", getAvgShooterCurrentDraw());
        SmartDashboard.putNumber("shooter position", m_leaderPivotMotor.getPosition().getValueAsDouble());
        SmartDashboard.putNumber(ShooterConstants.kShooterTargetPositionKey, m_positionRequest.Position);
        SmartDashboard.putNumber("Actual shooter speed", this.m_leftLeaderShooterMotor.getVelocity().getValueAsDouble());
    }

    private void setShooterVelocity(double velocityRps) {
        m_leftLeaderShooterMotor.setControl(m_velocityRequest.withVelocity(velocityRps));
        m_backLeftFollowerShooterMotor.setControl(m_velocityRequest.withVelocity(velocityRps));
        m_backRightFollowerShooterMotor.setControl(m_velocityRequest.withVelocity(velocityRps));
        m_rightFollowerShooterMotor.setControl(m_velocityRequest.withVelocity(velocityRps));
    }
}
