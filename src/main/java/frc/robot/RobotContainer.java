// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenix6.SignalLogger;
import com.ctre.phoenix6.swerve.SwerveRequest;
import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.commands.PathPlannerAuto;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.trajectory.PathPlannerTrajectory;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
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
  SendableChooser<Command> autoSelection = AutoBuilder.buildAutoChooser();
  
  //private final Localisation m_localisation = new Localisation(m_drivetrain);
  private final Lintake m_lintake = new Lintake();
  private final Shooter m_shooter = new Shooter(() -> new Pose2d(),m_drivetrain);
  //private final Feeder m_feeder = new Feeder();

  private final CommandXboxController m_controller = new CommandXboxController(0);

    final SwerveRequest.FieldCentric driveRequest = new SwerveRequest.FieldCentric()
            .withDeadband(Constants.kMaxSpeed * 0.1).withRotationalDeadband(Constants.kMaxAngularRate * 0.1)
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage);

  public RobotContainer() {
    SmartDashboard.putData(autoSelection);

   
    SignalLogger.enableAutoLogging(false);

    traj = new PathPlannerAuto("testauto");

    //PathPlannerPath doubleswipeone = PathPlannerPath.fromPathFile("doubleswipe1");
    NamedCommands.registerCommand("zerodrive", m_drivetrain.runOnce(m_drivetrain::seedFieldCentric));
    NamedCommands.registerCommand("intake deploy", m_lintake.setState(PinionState.GROUND));
    NamedCommands.registerCommand("intake retract", m_lintake.setState(PinionState.STOW));
    NamedCommands.registerCommand("intake run", Commands.runOnce(()->m_lintake.setState(RollerState.INTAKE)));
    NamedCommands.registerCommand("shoot",  Commands.sequence(Commands.runOnce(() -> {
            m_shooter.setState(PivotState.SCORE);
            m_shooter.setState(ShooterState.SCORE);
        }, m_shooter),

        Commands.waitSeconds(0.5),

        Commands.runOnce(() -> {
            m_shooter.setState(IndexerState.SCORE);
        }, m_shooter)
    ));
    NamedCommands.registerCommand("stopshoot", Commands.runOnce(() -> {
        m_shooter.setState(IndexerState.ZERO);
        m_shooter.setState(ShooterState.ZERO);
        m_shooter.setState(PivotState.STOW);
    }));
    
    configureBindings();
  }

  private void configureBindings() {
    
    
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
    //m_controller.povLeft().onTrue(m_shooter.zero());
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
m_controller.povUp().whileTrue(
    Commands.sequence(
        Commands.runOnce(() -> {
            m_shooter.setState(PivotState.LOB);
            m_shooter.setState(ShooterState.LOB);
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
    m_controller.leftBumper().onTrue(m_lintake.setState(PinionState.GROUND));
    m_controller.rightBumper().onTrue(m_lintake.setState(PinionState.STOW));
   // m_controller.a().onTrue(m_lintake.extend());
  }

  public Command getAutonomousCommand() {
    return Commands.sequence(
        m_drivetrain.applyRequest(() -> driveRequest
            .withVelocityX(0.5 * Constants.kMaxSpeed)
            .withVelocityY(0)
            .withRotationalRate(0)
        ).withTimeout(1).andThen(m_drivetrain.applyRequest(() -> driveRequest
            .withVelocityX(0)
            .withVelocityY(0)
            .withRotationalRate(0))
        ).withTimeout(1),
        Commands.sequence(
            Commands.runOnce(() -> {
            m_shooter.setState(PivotState.SCORE);
            m_shooter.setState(ShooterState.SCORE);
        }, m_shooter),

        Commands.waitSeconds(0.5),

        Commands.run(() -> {
            m_shooter.setState(IndexerState.SCORE);
        }, m_shooter)
        )
    );

    /* 
    PathPlannerPath auto;

    try {
        auto = PathPlannerPath.fromPathFile("eightauto");
        m_drivetrain.setPose(auto.getStartingHolonomicPose().get());
    } catch (Exception e) {
        return Commands.print("IO Error");
    }
    
    //return autoSelection.getSelected();

    return Commands.sequence(Commands.runOnce(() -> {
            m_shooter.setState(PivotState.SCORE);
            m_shooter.setState(ShooterState.SCORE);
        }, m_shooter),

        Commands.waitSeconds(0.5),

        Commands.runOnce(() -> {
            m_shooter.setState(IndexerState.SCORE);
        }, m_shooter)
    );
    */
  }
}
