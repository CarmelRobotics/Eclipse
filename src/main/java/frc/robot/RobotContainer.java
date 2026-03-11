// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenix6.swerve.SwerveRequest;
import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.commands.PathPlannerAuto;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.trajectory.PathPlannerTrajectory;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.RobotModeTriggers;
import frc.robot.subsystems.drive.CommandSwerveDrivetrain;
import frc.robot.subsystems.drive.TunerConstants;
import frc.robot.subsystems.feeder.Feeder;
import frc.robot.subsystems.feeder.FeederConstants.FeederState;
import frc.robot.subsystems.lintake.Lintake;
import frc.robot.subsystems.lintake.LintakeConstants.PinionState;
import frc.robot.subsystems.lintake.LintakeConstants.RollerState;
import frc.robot.subsystems.localisation.Localisation;
import frc.robot.subsystems.shooter.Shooter;
import frc.robot.subsystems.shooter.ShooterConstants.IndexerState;
import frc.robot.subsystems.shooter.ShooterConstants.PivotState;
import frc.robot.subsystems.shooter.ShooterConstants.ShooterState;

public class RobotContainer {
  private PathPlannerAuto traj;
  private final CommandSwerveDrivetrain m_drivetrain = TunerConstants.createDrivetrain();
  private final Localisation m_localisation = new Localisation(m_drivetrain);
  private final Lintake m_lintake = new Lintake();
  private final Shooter m_shooter = new Shooter(() -> new Pose2d());
  private final Feeder m_feeder = new Feeder();
  private final CommandXboxController m_controller = new CommandXboxController(0);

  public RobotContainer() {
    traj = new PathPlannerAuto("testauto");
    configureBindings();
  }

  private void configureBindings() {
    final SwerveRequest.FieldCentric driveRequest = new SwerveRequest.FieldCentric()
            .withDeadband(Constants.kMaxSpeed * 0.1).withRotationalDeadband(Constants.kMaxAngularRate * 0.1)
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage);
    
    final SwerveRequest idle = new SwerveRequest.Idle();

    m_drivetrain.setDefaultCommand(
            m_drivetrain.applyRequest(() ->
                driveRequest.withVelocityX(-m_controller.getLeftY() * Constants.kMaxSpeed)
                    .withVelocityY(-m_controller.getLeftX() * Constants.kMaxSpeed)
                    .withRotationalRate(-m_controller.getRightX() * Constants.kMaxAngularRate)
            )
        );

    RobotModeTriggers.disabled().whileTrue(
            m_drivetrain.applyRequest(() -> idle).ignoringDisable(true)
        );
    
    /*
     * Pov Down: Reset robot rotation to zero
     * Left Trigger: Set lintake to ground and then set roller to intake, afterwards set the roller to zero
     * Left Bumper: Set lintake to ground
     * Right Trigger: Set pivot to score, feeder to score, indexer to score, shooter to score, and then reset all to zero.
     * Right Bumper: Set lintake to stow.
     */
    
    m_controller.povDown().onTrue(m_drivetrain.runOnce(m_drivetrain::seedFieldCentric));
    m_controller.leftTrigger().whileTrue(Commands.runEnd(() -> {
      m_lintake.setState(RollerState.INTAKE);
    }, () -> {
      m_lintake.setState(RollerState.ZERO);
    }, m_lintake));
   m_controller.rightTrigger().whileTrue(
    Commands.sequence(
        Commands.runOnce(() -> {
            m_shooter.setState(PivotState.SCORE);
            m_shooter.setState(ShooterState.SCORE);
        }, m_shooter),

        Commands.waitSeconds(0.5),

        Commands.run(() -> {
            m_shooter.setState(IndexerState.SCORE);
        }, m_shooter)
    ).finallyDo(() -> {
        m_shooter.setState(IndexerState.ZERO);
        m_shooter.setState(ShooterState.ZERO);
        m_shooter.setState(PivotState.STOW);
    })
);
    m_controller.rightBumper().onTrue(m_lintake.retract());
    m_controller.leftBumper().onTrue(m_lintake.extend());
  }

  public Command getAutonomousCommand() {
    return m_drivetrain.setPose(new Pose2d(3.536,7.365,Rotation2d.fromDegrees(0))).andThen(Commands.runOnce(()->m_lintake.setState(PinionState.GROUND)).andThen(AutoBuilder.pathfindToPose(new Pose2d(5.806,7.365, Rotation2d.fromDegrees(0)), new PathConstraints(1,1,540,540))));
  }
}
