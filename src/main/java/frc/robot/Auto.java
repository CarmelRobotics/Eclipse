package frc.robot;

import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.drive.CommandSwerveDrivetrain;
import frc.robot.subsystems.shooter.Shooter;
import frc.robot.subsystems.shooter.ShooterConstants.KickerState;
import frc.robot.subsystems.shooter.ShooterConstants.PivotState;
import frc.robot.subsystems.shooter.ShooterConstants.ShooterState;

public class Auto {
    public static final Command eightAuto(CommandSwerveDrivetrain drivetrain, SwerveRequest.FieldCentric driveRequest, Shooter shooter) {
        return Commands.runOnce(drivetrain::seedFieldCentric, drivetrain)
                .andThen(drivetrain.applyRequest(() -> driveRequest
                        .withVelocityX(0.25 * Constants.kMaxSpeed)
                        .withVelocityY(0)
                        .withRotationalRate(0.05 * Constants.kMaxAngularRate)).withTimeout(0.75))
                .andThen(drivetrain.applyRequest(() -> driveRequest
                        .withVelocityX(0)
                        .withVelocityY(0)
                        .withRotationalRate(0)).withTimeout(1))
                .andThen(shooter.setState(PivotState.SCORE, ShooterState.SCORE, KickerState.FEED));
    }
}
