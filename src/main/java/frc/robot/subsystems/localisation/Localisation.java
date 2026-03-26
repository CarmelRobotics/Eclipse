package frc.robot.subsystems.localisation;

import java.util.function.Function;

import com.ctre.phoenix6.swerve.SwerveRequest;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.commands.PathfindingCommand;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.drive.CommandSwerveDrivetrain;

public class Localisation extends SubsystemBase {
    private final CommandSwerveDrivetrain m_drivetrain;
    private final SwerveRequest.ApplyRobotSpeeds m_robotSpeedsRequest = new SwerveRequest.ApplyRobotSpeeds();
    private final Field2d m_field = new Field2d();
    private Function<String, LimelightHelpers.PoseEstimate> limelightGetBotPoseEstimate;

    public Localisation(CommandSwerveDrivetrain drive) {
        m_drivetrain = drive;

        for (LimelightInfo limelight : LocalisationConstants.kLimelights) {
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
            this::getPose,
            this::resetPose,
            () -> m_drivetrain.getState().Speeds,
            (speeds, feedforwards) -> m_drivetrain.setControl(
                m_robotSpeedsRequest.withSpeeds(ChassisSpeeds.discretize(speeds, 0.020))
                    .withWheelForceFeedforwardsX(feedforwards.robotRelativeForcesXNewtons())
                    .withWheelForceFeedforwardsY(feedforwards.robotRelativeForcesYNewtons())
            ),
            LocalisationConstants.PathController,
            LocalisationConstants.RobotConfig,
            () -> DriverStation.getAlliance().orElse(Alliance.Blue) == Alliance.Red,
            m_drivetrain
        );

        setPerspective(DriverStation.getAlliance().orElse(Alliance.Blue));

        SmartDashboard.putData(LocalisationConstants.kFieldKey, m_field);
        CommandScheduler.getInstance().schedule(PathfindingCommand.warmupCommand());
    }

    public Pose2d getPose() {
        return m_drivetrain.getState().Pose;
    }

    public void resetPose(Pose2d newPose) {
        m_drivetrain.resetPose(newPose);
    }

    public void setPerspective(Alliance alliance) {
        final Rotation2d perspective = alliance == Alliance.Red ? Rotation2d.k180deg : Rotation2d.kZero;
        m_drivetrain.setOperatorPerspectiveForward(perspective);
        limelightGetBotPoseEstimate = alliance == Alliance.Red
            ? LimelightHelpers::getBotPoseEstimate_wpiBlue
            : LimelightHelpers::getBotPoseEstimate_wpiRed;
        resetPose(new Pose2d(0, 0, perspective));
    }

    public void periodic() {
        for (LimelightInfo limelight : LocalisationConstants.kLimelights) {
            LimelightHelpers.SetRobotOrientation(
                    limelight.name(), getPose().getRotation().getDegrees(), 
                    m_drivetrain.getPigeon2().getAngularVelocityZWorld().getValueAsDouble(), 
                    0, 0, 0, 0
                );

            final LimelightHelpers.PoseEstimate estimate = limelightGetBotPoseEstimate.apply(limelight.name());

            if (estimate == null) {
                continue;
            }

            if (estimate.tagCount > 0) {
                final double xyStdDev;
                if (estimate.avgTagDist > 0.125 && estimate.avgTagArea < 2.5) {
                    xyStdDev = 0.25;
                } else {
                    xyStdDev = Math.pow(0.5, estimate.avgTagDist + 1);
                }

                m_drivetrain.addVisionMeasurement(estimate.pose, estimate.timestampSeconds, VecBuilder.fill(xyStdDev, xyStdDev, Double.MAX_VALUE));
            }
        }

        m_field.setRobotPose(getPose());
    }
}
