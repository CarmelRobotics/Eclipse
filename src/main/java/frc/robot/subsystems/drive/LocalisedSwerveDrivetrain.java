package frc.robot.subsystems.drive;

import java.util.function.Function;
import java.util.function.Supplier;

import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.swerve.SwerveDrivetrain;
import com.ctre.phoenix6.swerve.SwerveRequest;

import com.pathplanner.lib.auto.AutoBuilder;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.Subsystem;

public class LocalisedSwerveDrivetrain extends SwerveDrivetrain<TalonFX, TalonFX, CANcoder> implements Subsystem {
    private final SwerveRequest.ApplyRobotSpeeds m_robotSpeedsRequest = new SwerveRequest.ApplyRobotSpeeds();
    private final Field2d m_field = new Field2d();
    private Function<String, LimelightHelpers.PoseEstimate> limelightGetBotPoseEstimate;

    public LocalisedSwerveDrivetrain() {
        super(
            TalonFX::new, TalonFX::new, CANcoder::new,
            LocalisedSwerveConstants.DrivetrainConstants,
            LocalisedSwerveConstants.FrontLeft, LocalisedSwerveConstants.FrontRight,
            LocalisedSwerveConstants.BackLeft, LocalisedSwerveConstants.BackRight
        );

        for (LimelightInfo limelight : LocalisedSwerveConstants.kLimelights) {
            LimelightHelpers.setCameraPose_RobotSpace(
                limelight.name(),
                limelight.forward(),
                limelight.side(),
                limelight.up(),
                limelight.roll(),
                limelight.pitch(),
                limelight.yaw()
            );
        }

        AutoBuilder.configure(
            () -> getState().Pose,
            this::resetPose,
            () -> getState().Speeds,
            (speeds, feedforwards) -> setControl(
                m_robotSpeedsRequest.withSpeeds(ChassisSpeeds.discretize(speeds, 0.020))
                    .withWheelForceFeedforwardsX(feedforwards.robotRelativeForcesXNewtons())
                    .withWheelForceFeedforwardsY(feedforwards.robotRelativeForcesYNewtons())
            ),
            LocalisedSwerveConstants.PathController,
            LocalisedSwerveConstants.RobotConfig,
            () -> DriverStation.getAlliance().orElse(Alliance.Blue) == Alliance.Red,
            this
        );

        setPerspective(DriverStation.getAlliance().orElse(Alliance.Blue));

        SmartDashboard.putData(LocalisedSwerveConstants.kFieldKey, m_field);
    }

    public Command applyRequest(Supplier<SwerveRequest> requestSupplier) {
        return Commands.run(() -> setControl(requestSupplier.get()), this);
    }

    public void setPerspective(Alliance alliance) {
        setOperatorPerspectiveForward(alliance == Alliance.Red ? Rotation2d.k180deg : Rotation2d.kZero);
        limelightGetBotPoseEstimate = alliance == Alliance.Red 
            ? LimelightHelpers::getBotPoseEstimate_wpiRed_MegaTag2
            : LimelightHelpers::getBotPoseEstimate_wpiBlue_MegaTag2;
    }

    @Override
    public void periodic() {
        for (LimelightInfo limelight : LocalisedSwerveConstants.kLimelights) {

            if (LimelightHelpers.getTV(limelight.name())) {
                LimelightHelpers.SetRobotOrientation(
                    limelight.name(), getState().Pose.getRotation().getDegrees(), 
                    getPigeon2().getAngularVelocityZWorld().getValueAsDouble(), 
                    0, 0, 0, 0
                );

                final LimelightHelpers.PoseEstimate estimate = limelightGetBotPoseEstimate.apply(limelight.name());

                final double xyStdDev = estimate.tagCount >= 2 ? 0.5 : 0.6 * (estimate.avgTagDist * 0.5);

                if (estimate.avgTagDist < 0.075 || estimate.avgTagDist > 5) {
                    continue;
                }

                addVisionMeasurement(estimate.pose, estimate.timestampSeconds, VecBuilder.fill(xyStdDev, xyStdDev, 99999));
            }
        }

        m_field.setRobotPose(getState().Pose);
    }
}
