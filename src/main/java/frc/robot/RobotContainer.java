// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.RotationsPerSecond;

import com.ctre.phoenix6.swerve.SwerveRequest;
import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;

import edu.wpi.first.math.geometry.Pose2d;
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
import frc.robot.subsystems.shooter.Shooter;
import frc.robot.subsystems.shooter.ShooterConstants.IndexerState;
import frc.robot.subsystems.shooter.ShooterConstants.PivotState;
import frc.robot.subsystems.shooter.ShooterConstants.ShooterState;

public class RobotContainer {
  private static final double MaxSpeed = 1.0 * TunerConstants.kSpeedAt12Volts.in(MetersPerSecond); // kSpeedAt12Volts desired top speed
  private static final double MaxAngularRate = RotationsPerSecond.of(0.75).in(RadiansPerSecond); // 3/4 of a rotation per second max angular velocity

  /* Setting up bindings for necessary control of the swerve drive platform */
  private final SwerveRequest.FieldCentric m_drive = new SwerveRequest.FieldCentric()
            .withDeadband(MaxSpeed * 0.1).withRotationalDeadband(MaxAngularRate * 0.1) // Add a 10% deadband
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage); // Use open-loop control for drive motors

  private final CommandSwerveDrivetrain m_drivetrain = TunerConstants.createDrivetrain();

  private final Lintake m_lintake = new Lintake();
  private final Shooter m_shooter = new Shooter(() -> new Pose2d());
  private final Feeder m_feeder = new Feeder();
  private final CommandXboxController m_controller = new CommandXboxController(0);

  public RobotContainer() {
    configureBindings();
  }

  private void configureBindings() {
    m_drivetrain.setDefaultCommand(
            // Drivetrain will execute this command periodically
            m_drivetrain.applyRequest(() ->
                m_drive.withVelocityX(-m_controller.getLeftY() * MaxSpeed) // Drive forward with negative Y (forward)
                    .withVelocityY(-m_controller.getLeftX() * MaxSpeed) // Drive left with negative X (left)
                    .withRotationalRate(-m_controller.getRightX() * MaxAngularRate) // Drive counterclockwise with negative X (left)
            )
        );


    final SwerveRequest idle = new SwerveRequest.Idle();
    RobotModeTriggers.disabled().whileTrue(
            m_drivetrain.applyRequest(() -> idle).ignoringDisable(true)
        );
        
    
    m_controller.povDown().onTrue(m_drivetrain.runOnce(m_drivetrain::seedFieldCentric));

    m_controller.leftBumper().onTrue(Commands.run(() -> m_lintake.setState(PinionState.GROUND), m_lintake));
    m_controller.rightBumper().onTrue(Commands.run(() -> m_lintake.setState(PinionState.STOW), m_lintake));
    m_controller.rightTrigger().onTrue(Commands.run(() -> m_shooter.setState(PivotState.SCORE), m_shooter));
    m_controller.leftTrigger().onTrue(Commands.run(() -> m_shooter.setState(PivotState.STOW), m_shooter));
    m_controller.a().onTrue(Commands.run(() -> m_shooter.setState(ShooterState.SCORE), m_shooter));
    m_controller.b().onTrue(Commands.run(() -> m_shooter.setState(ShooterState.ZERO), m_shooter));
    m_controller.x().onTrue(Commands.run(() -> m_shooter.setState(IndexerState.SCORE), m_shooter));
    m_controller.y().onTrue(Commands.run(() -> m_shooter.setState(IndexerState.ZERO), m_shooter));
    /*
    m_controller.povUp().onTrue(Commands.run(() -> m_feeder.setState(FeederState.SCORE), m_feeder));
    m_controller.povDown().onTrue(Commands.run(() -> m_feeder.setState(FeederState.ZERO), m_feeder));
    */
    m_controller.povLeft().onTrue(Commands.run(() -> m_lintake.setState(RollerState.INTAKE), m_lintake));
    m_controller.povRight().onTrue(Commands.run(() -> m_lintake.setState(RollerState.ZERO), m_lintake));
  }

  public Command getAutonomousCommand() {
    return Commands.print("No autonomous command configured");
  }
}
