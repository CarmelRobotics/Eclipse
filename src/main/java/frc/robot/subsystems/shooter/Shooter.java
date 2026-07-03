package frc.robot.subsystems.shooter;

import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.MotorAlignmentValue;

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
    // Overcurrent protection trackers
    private double m_pivotOvercurrentStartTime = 0;
    private double m_indexerOvercurrentStartTime = 0;

    private final CommandSwerveDrivetrain m_drive;

    public Shooter(CommandSwerveDrivetrain drive) {
        m_drive = drive;
        SmartDashboard.putNumber(ShooterConstants.kPivotOffsetKey, 0);
        SmartDashboard.putNumber(ShooterConstants.kShooterRpsOffsetKey, 0);
        SmartDashboard.putNumber(ShooterConstants.kTimeOfFlightOffsetKey, 0);
    // Tunables for frame-clear position and current limits
    SmartDashboard.putNumber(ShooterConstants.kShotBlockPivotPositionKey, ShooterConstants.kShotBlockPivotPosition);
    SmartDashboard.putNumber(ShooterConstants.kPivotCurrentLimitKey, 40.0);
    SmartDashboard.putNumber(ShooterConstants.kPivotCurrentTimeoutKey, 0.25);
    SmartDashboard.putNumber(ShooterConstants.kIndexerCurrentLimitKey, 25.0);
    SmartDashboard.putNumber(ShooterConstants.kIndexerCurrentTimeoutKey, 0.2);

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
        // Convert motor position to output (pivot) rotations using gear ratio
        double currentPivotOutputRot = m_leaderPivotMotor.getPosition().getValueAsDouble() / ShooterConstants.kPivotGearRatio;
        boolean pivotReady = Math.abs(currentPivotOutputRot - m_targetPivotPosition)
            <= ShooterConstants.kPivotReadyToleranceRotations;

        // Published individually so a stuck shot can be diagnosed from the dashboard
        // instead of just seeing the combined "ready to shoot" go false.
        SmartDashboard.putBoolean("ready/shot in range", shotInRange);
        SmartDashboard.putBoolean("ready/aimed", aimed);
        SmartDashboard.putBoolean("ready/shooter at speed", shooterReady);
        SmartDashboard.putBoolean("ready/pivot in position", pivotReady);

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
    // Allow frame-clear to be tuned on dashboard
        m_targetShooterRps = ShooterConstants.getScoreShooterRps(shotDistance) + shooterRpsOffset;

        SmartDashboard.putNumber("shot compensated distance", shotDistance);
        SmartDashboard.putNumber("shot time of flight", m_drive.getShotTimeOfFlightSeconds());
        SmartDashboard.putNumber("interpolated pivot position", m_targetPivotPosition);
        SmartDashboard.putNumber("interpolated shooter rps", m_targetShooterRps);
        SmartDashboard.putBoolean("ready to shoot", readyToShoot());


        switch (m_pivotState) {
            
            case STOW -> {
                 m_leaderPivotMotor.setControl(m_positionRequest.withPosition(ShooterConstants.kStowPivotPosition * ShooterConstants.kPivotGearRatio));
                 m_followerPivotMotor.setControl(m_positionRequest.withPosition(ShooterConstants.kStowPivotPosition * ShooterConstants.kPivotGearRatio));
            }
            case SCORE -> {
                m_leaderPivotMotor.setControl(m_positionRequest.withPosition(m_targetPivotPosition * ShooterConstants.kPivotGearRatio));
                m_followerPivotMotor.setControl(m_positionRequest.withPosition(m_targetPivotPosition * ShooterConstants.kPivotGearRatio));
            }
            case LOB -> {
                // m_leaderPivotMotor.setControl(m_positionRequest.withPosition(0.45));
                // m_followerPivotMotor.setControl(m_positionRequest.withPosition(0.45));
            }
            case SHOT_BLOCK -> {
                double framePos = SmartDashboard.getNumber(ShooterConstants.kShotBlockPivotPositionKey, ShooterConstants.kShotBlockPivotPosition);
                m_leaderPivotMotor.setControl(m_positionRequest.withPosition(framePos * ShooterConstants.kPivotGearRatio));
                m_followerPivotMotor.setControl(m_positionRequest.withPosition(framePos * ShooterConstants.kPivotGearRatio));
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
    // Publish pivot position in output (pivot) rotations for clarity
    double pivotOutputRot = m_leaderPivotMotor.getPosition().getValueAsDouble() / ShooterConstants.kPivotGearRatio;
    SmartDashboard.putNumber("shooter position", pivotOutputRot);
        // Overcurrent checks (simple): pivot and indexer
        double now = edu.wpi.first.wpilibj.Timer.getFPGATimestamp();
        double pivotCurrent = Math.max(m_leaderPivotMotor.getSupplyCurrent().getValueAsDouble(), m_followerPivotMotor.getSupplyCurrent().getValueAsDouble());
        double pivotLimit = SmartDashboard.getNumber(ShooterConstants.kPivotCurrentLimitKey, 40.0);
        double pivotTimeout = SmartDashboard.getNumber(ShooterConstants.kPivotCurrentTimeoutKey, 0.25);
        if (pivotCurrent > pivotLimit) {
            if (m_pivotOvercurrentStartTime == 0) m_pivotOvercurrentStartTime = now;
            else if (now - m_pivotOvercurrentStartTime > pivotTimeout) {
                // trip: stop pivot and stow
                m_leaderPivotMotor.setVoltage(0);
                m_followerPivotMotor.setVoltage(0);
                m_pivotState = PivotState.STOW;
                SmartDashboard.putBoolean("pivot overcurrent tripped", true);
            }
        } else {
            m_pivotOvercurrentStartTime = 0;
        }

        double indexerCurrent = m_indexerMotor.getSupplyCurrent().getValueAsDouble();
        double indexerLimit = SmartDashboard.getNumber(ShooterConstants.kIndexerCurrentLimitKey, 25.0);
        double indexerTimeout = SmartDashboard.getNumber(ShooterConstants.kIndexerCurrentTimeoutKey, 0.2);
        if (indexerCurrent > indexerLimit) {
            if (m_indexerOvercurrentStartTime == 0) m_indexerOvercurrentStartTime = now;
            else if (now - m_indexerOvercurrentStartTime > indexerTimeout) {
                // stop feeding
                m_indexerMotor.setVoltage(0);
                m_indexerState = IndexerState.ZERO;
                SmartDashboard.putBoolean("indexer overcurrent tripped", true);
            }
        } else {
            m_indexerOvercurrentStartTime = 0;
        }
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
