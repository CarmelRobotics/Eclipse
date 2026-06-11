package frc.robot.subsystems.shooter;

import static edu.wpi.first.units.Units.RotationsPerSecond;

import java.util.function.Supplier;

import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.MotorAlignmentValue;

//import dev.doglog.DogLog;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.drive.CommandSwerveDrivetrain;
import frc.robot.subsystems.shooter.ShooterConstants.IndexerState;
import frc.robot.subsystems.shooter.ShooterConstants.PivotState;
import frc.robot.subsystems.shooter.ShooterConstants.ShooterState;

public class Shooter extends SubsystemBase {
   // private ShooterCalculator calc = new ShooterCalculator();

    private final TalonFX m_indexerMotor = new TalonFX(ShooterConstants.kIndexerMotorId);

    public static InterpolatingDoubleTreeMap PivotPositionMap =  new InterpolatingDoubleTreeMap();
     
    
    
    public static InterpolatingDoubleTreeMap ShooterVelocityRPSMap = new InterpolatingDoubleTreeMap();

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

    private CommandSwerveDrivetrain m_drive;

    public Shooter(Supplier<Pose2d> poseSupplier, CommandSwerveDrivetrain drive) {
        m_poseSupplier = poseSupplier;
        setHubPosition(DriverStation.getAlliance().orElse(Alliance.Blue));

        m_drive = drive;

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

        PivotPositionMap.put(2.08, .35);
        PivotPositionMap.put(2.66,.15);
        PivotPositionMap.put(3.5, .3);
        ShooterVelocityRPSMap.put(1.97,7.0);
        ShooterVelocityRPSMap.put(2.23,7.25);
       // ShooterVelocityRPSMap.put(3.5, 7.8);
        

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

    public Command zero(){
        return Commands.runOnce(()->{
            this.m_followerPivotMotor.setPosition(0);
            this.m_leaderPivotMotor.setPosition(0);
        });
    }
    public double f(double x) {
        return  (Math.pow(x, 1.4)) + 4.75;
    }

    private void shoot() {
        final double distance = m_poseSupplier.get().getTranslation().getDistance(m_hubPosition);
        final AngularVelocity velocity = RotationsPerSecond.of(ShooterVelocityRPSMap.get(distance));
        m_leftLeaderShooterMotor.setControl(m_velocityRequest.withVelocity(velocity));
    }

    private void pivot() {
        final double distance = m_poseSupplier.get().getTranslation().getDistance(m_hubPosition);
        final double position = PivotPositionMap.get(distance);
        m_leaderPivotMotor.setControl(m_positionRequest.withPosition(position));
    }

    public double getAvgShooterCurrentDraw(){
        var sum = m_leftLeaderShooterMotor.getSupplyCurrent().getValueAsDouble() + m_backLeftFollowerShooterMotor.getSupplyCurrent().getValueAsDouble() + m_rightFollowerShooterMotor.getSupplyCurrent().getValueAsDouble() + m_backRightFollowerShooterMotor.getSupplyCurrent().getValueAsDouble();
        return sum / 4;

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

        SmartDashboard.putNumber("interpolated velocity", f(m_drive.getDistanceToClosestHub()));


        switch (m_pivotState) {
            
            case STOW -> {
                 m_leaderPivotMotor.setControl(m_positionRequest.withPosition(-0));
                 m_followerPivotMotor.setControl(m_positionRequest.withPosition(-0));
            }
            case SCORE -> {
                //var angle = ShooterCalculator.calculate(m_drive.getDistanceToClosestHub(), 0).angleDegrees();

                m_leaderPivotMotor.setControl(m_positionRequest.withPosition(0));
                m_followerPivotMotor.setControl(m_positionRequest.withPosition(0));
                //   m_leaderPivotMotor.setControl(m_positionRequest.withPosition(.3));
                //   m_followerPivotMotor.setControl(m_positionRequest.withPosition(.3));

            }
            case LOB -> {
                // m_leaderPivotMotor.setControl(m_positionRequest.withPosition(0.45));
                // m_followerPivotMotor.setControl(m_positionRequest.withPosition(0.45));
            }
        }
        

        switch (m_shooterState) {
            case ZERO -> {
                m_leftLeaderShooterMotor.setControl(m_velocityRequest.withVelocity(0.5));
                m_backLeftFollowerShooterMotor.setControl(m_velocityRequest.withVelocity(0.5));
                m_backRightFollowerShooterMotor.setControl(m_velocityRequest.withVelocity(0.5));
                m_rightFollowerShooterMotor.setControl(m_velocityRequest.withVelocity(0.5));
            }
            case SCORE -> {
               

                m_leftLeaderShooterMotor.setControl(m_velocityRequest.withVelocity(47));
                m_backLeftFollowerShooterMotor.setControl(m_velocityRequest.withVelocity(47));
                m_backRightFollowerShooterMotor.setControl(m_velocityRequest.withVelocity(47));
                m_rightFollowerShooterMotor.setControl(m_velocityRequest.withVelocity(47));

                // m_leftLeaderShooterMotor.setControl(m_velocityRequest.withVelocity(7.8));
                // m_backLeftFollowerShooterMotor.setControl(m_velocityRequest.withVelocity(7.8));
                // m_backRightFollowerShooterMotor.setControl(m_velocityRequest.withVelocity(7.8));
                // m_rightFollowerShooterMotor.setControl(m_velocityRequest.withVelocity(7.8));
            }
             case LOB -> {
                m_leftLeaderShooterMotor.setControl(m_velocityRequest.withVelocity(40));
                m_backLeftFollowerShooterMotor.setControl(m_velocityRequest.withVelocity(40));
                m_backRightFollowerShooterMotor.setControl(m_velocityRequest.withVelocity(40));
                m_rightFollowerShooterMotor.setControl(m_velocityRequest.withVelocity(40));
            }
            case SEND ->{
                m_leftLeaderShooterMotor.setControl(m_velocityRequest.withVelocity(90));
                m_backLeftFollowerShooterMotor.setControl(m_velocityRequest.withVelocity(90));
                m_backRightFollowerShooterMotor.setControl(m_velocityRequest.withVelocity(90));
                m_rightFollowerShooterMotor.setControl(m_velocityRequest.withVelocity(90));
            }
            
        }
        
        m_indexerMotor.setVoltage(m_indexerState.velocity.magnitude());
       // DogLog.log("Shooter avg current draw", getAvgShooterCurrentDraw(), "amps");
       SmartDashboard.putNumber("shooter current draw", getAvgShooterCurrentDraw());
       SmartDashboard.putNumber("shooter position", m_leaderPivotMotor.getPosition().getValueAsDouble());
        SmartDashboard.putNumber(ShooterConstants.kShooterTargetPositionKey, m_positionRequest.Position);
        SmartDashboard.putNumber("Actual shooter speed", this.m_leftLeaderShooterMotor.getVelocity().getValueAsDouble());
    }
}