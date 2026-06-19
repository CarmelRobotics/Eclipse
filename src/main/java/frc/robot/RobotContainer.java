// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenix6.SignalLogger;
import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;

import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.RobotModeTriggers;
import frc.robot.subsystems.drive.CommandSwerveDrivetrain;
import frc.robot.subsystems.drive.TunerConstants;
import frc.robot.subsystems.lintake.Lintake;
import frc.robot.subsystems.lintake.LintakeConstants.PinionState;
import frc.robot.subsystems.lintake.LintakeConstants.RollerState;
import frc.robot.subsystems.shooter.Shooter;
import frc.robot.subsystems.shooter.ShooterConstants;
import frc.robot.subsystems.shooter.ShooterConstants.IndexerState;
import frc.robot.subsystems.shooter.ShooterConstants.PivotState;
import frc.robot.subsystems.shooter.ShooterConstants.ShooterState;

public class RobotContainer {
  private final CommandSwerveDrivetrain m_drivetrain = TunerConstants.createDrivetrain();
  private final Lintake m_lintake = new Lintake();
  private final Shooter m_shooter = new Shooter(m_drivetrain);
  private final CommandXboxController m_controller = new CommandXboxController(0);

  private final SwerveRequest.FieldCentric driveRequest = new SwerveRequest.FieldCentric()
      .withDeadband(Constants.kMaxSpeed * 0.1)
      .withRotationalDeadband(Constants.kMaxAngularRate * 0.1)
      .withDriveRequestType(DriveRequestType.OpenLoopVoltage);

  private final SendableChooser<Command> autoSelection;

  public RobotContainer() {
    SignalLogger.enableAutoLogging(false);
    registerNamedCommands();

    autoSelection = AutoBuilder.buildAutoChooser();
    SmartDashboard.putData("Auto", autoSelection);

    configureBindings();
  }

  private void registerNamedCommands() {
    NamedCommands.registerCommand("zerodrive", Commands.none());
    NamedCommands.registerCommand("intake deploy", m_lintake.setState(PinionState.GROUND));
    NamedCommands.registerCommand("intake retract", m_lintake.setState(PinionState.STOW));
    NamedCommands.registerCommand("intake run", Commands.runOnce(() -> m_lintake.setState(RollerState.INTAKE)));
    NamedCommands.registerCommand("shoot", timedShotCommand(PivotState.SCORE, ShooterState.SCORE, 1.5));
    NamedCommands.registerCommand("stopshoot", stopShooterCommand());
  }

  private void configureBindings() {
    final SwerveRequest idle = new SwerveRequest.Idle();

    m_drivetrain.setDefaultCommand(
        m_drivetrain.applyRequest(() -> driveRequest
            .withVelocityX(driverXVelocity())
            .withVelocityY(driverYVelocity())
            .withRotationalRate(driverRotationalRate())
        )
    );

    RobotModeTriggers.disabled().whileTrue(
        m_drivetrain.applyRequest(() -> idle).ignoringDisable(true)
    );

    m_controller.povDown().onTrue(m_drivetrain.runOnce(m_drivetrain::seedFieldCentric));
    m_controller.leftBumper().onTrue(m_lintake.setState(PinionState.GROUND));
    m_controller.rightBumper().onTrue(m_lintake.setState(PinionState.STOW));
    m_controller.x().onTrue(m_lintake.setState(PinionState.AGITATE));

    m_controller.leftTrigger().whileTrue(
        rollerWhileHeldCommand(RollerState.INTAKE)
    );
    m_controller.povLeft().whileTrue(
        rollerWhileHeldCommand(RollerState.EJECT)
    );

    m_controller.a().whileTrue(
        m_drivetrain.faceHubCommand(this::driverXVelocity, this::driverYVelocity)
    );
    m_controller.rightTrigger().whileTrue(
        Commands.parallel(
            m_drivetrain.faceHubCommand(this::driverXVelocity, this::driverYVelocity),
            heldShotCommand(PivotState.SCORE, ShooterState.SCORE)
        )
    );
    m_controller.povUp().whileTrue(
        heldShotCommand(PivotState.LOB, ShooterState.LOB)
    );
    m_controller.povRight().whileTrue(
        heldShotCommand(PivotState.LOB, ShooterState.SEND)
    );
  }

  private Command rollerWhileHeldCommand(RollerState state) {
    return Commands.runEnd(
        () -> m_lintake.setState(state),
        () -> m_lintake.setState(RollerState.ZERO),
        m_lintake
    );
  }

  private Command timedShotCommand(PivotState pivotState, ShooterState shooterState, double feedSeconds) {
    return Commands.sequence(
        prepareShotCommand(pivotState, shooterState),
        Commands.waitUntil(m_shooter::readyToShoot).withTimeout(ShooterConstants.kShotSpinupTimeoutSeconds),
        Commands.runEnd(
            this::feedIfReady,
            this::stopShooter,
            m_shooter
        ).withTimeout(feedSeconds)
    );
  }

  private Command heldShotCommand(PivotState pivotState, ShooterState shooterState) {
    return Commands.sequence(
        prepareShotCommand(pivotState, shooterState),
        Commands.waitUntil(m_shooter::readyToShoot).withTimeout(ShooterConstants.kShotSpinupTimeoutSeconds),
        Commands.runEnd(
            this::feedIfReady,
            this::stopShooter,
            m_shooter
        )
    );
  }

  private Command prepareShotCommand(PivotState pivotState, ShooterState shooterState) {
    return Commands.runOnce(() -> {
      m_shooter.setState(pivotState);
      m_shooter.setState(shooterState);
    }, m_shooter);
  }

  private Command stopShooterCommand() {
    return Commands.runOnce(this::stopShooter, m_shooter);
  }

  private void stopShooter() {
    m_shooter.setState(IndexerState.ZERO);
    m_shooter.setState(ShooterState.ZERO);
    m_shooter.setState(PivotState.STOW);
  }

  private void feedIfReady() {
    m_shooter.setState(m_shooter.readyToShoot() ? IndexerState.SCORE : IndexerState.ZERO);
  }

  private double driverXVelocity() {
    return -m_controller.getLeftY() * Constants.kMaxSpeed;
  }

  private double driverYVelocity() {
    return -m_controller.getLeftX() * Constants.kMaxSpeed;
  }

  private double driverRotationalRate() {
    return -m_controller.getRightX() * Constants.kMaxAngularRate;
  }

  public Command getAutonomousCommand() {
    return autoSelection.getSelected();
  }
}
