// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenix6.SignalLogger;
import com.ctre.phoenix6.swerve.SwerveRequest;
import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.RobotModeTriggers;
import frc.robot.subsystems.drive.CommandSwerveDrivetrain;
import frc.robot.subsystems.drive.TunerConstants;
import frc.robot.subsystems.lintake.Lintake;
import frc.robot.subsystems.lintake.LintakeConstants.PinionState;
import frc.robot.subsystems.lintake.LintakeConstants.RollerState;
import frc.robot.subsystems.shooter.Shooter;
import frc.robot.subsystems.shooter.ShooterConstants.KickerState;
import frc.robot.subsystems.shooter.ShooterConstants.PivotState;
import frc.robot.subsystems.shooter.ShooterConstants.ShooterState;

public class RobotContainer {
    private final CommandXboxController m_controller = new CommandXboxController(0);

    private final CommandSwerveDrivetrain m_drivetrain = TunerConstants.createDrivetrain();
    private final Lintake m_lintake = new Lintake();
    private final Shooter m_shooter = new Shooter();

    private final SwerveRequest.FieldCentric m_driveRequest = new SwerveRequest.FieldCentric()
            .withDeadband(Constants.kMaxSpeed * 0.1).withRotationalDeadband(Constants.kMaxAngularRate * 0.1)
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage);

    public RobotContainer() {
        // We do not have enough storage for CTRE auto logging.
        SignalLogger.enableAutoLogging(false);

        configureBindings();
    }

    private void configureBindings() {
        final SwerveRequest idle = new SwerveRequest.Idle();

        m_drivetrain.setDefaultCommand(
                m_drivetrain.applyRequest(() -> m_driveRequest.withVelocityX(-m_controller.getLeftY() * Constants.kMaxSpeed)
                                .withVelocityY(-m_controller.getLeftX() * Constants.kMaxSpeed)
                                .withRotationalRate(-m_controller.getRightX() * Constants.kMaxAngularRate)));

        RobotModeTriggers.disabled().whileTrue(
                m_drivetrain.applyRequest(() -> idle).ignoringDisable(true));

        /*
         * Pov Down: On pressed, seed field centric (set roboot rotation to zero relative to where it is).
         * 
         * Left Trigger: While held, lintake rollers to intake voltage, returns to idle voltage when released.
         * Pov Left: While held, lintake rollers to eject voltage, returns to idle voltage when released.
         * Left Bumper: On pressed, set the lintake pinion to deploy angle.
         * Right Bumper: On pressed, set the lintake pinion to stow angle.
         * X: While held, set the lintake pinion to stow and deploy angles to agitate balls, returns to origin when released.
         * 
         * Right Trigger: While held, pivots to score, sets rollers to score, then feeds balls into shooter, returns to origin when released.
         * Pov Up: While held, pivots to lob, sets rollers to lob, then feeds balls into shooter, returns to origin when released.
         * Pov Righ: While held, pivots to send, sets rollers to send, then feeds balls into shooter, returns to origin when released.
         */
        m_controller.povDown().onTrue(m_drivetrain.runOnce(m_drivetrain::seedFieldCentric));

        m_controller.leftTrigger().whileTrue(m_lintake.setState(RollerState.INTAKE))
                .onFalse(m_lintake.setState(RollerState.IDLE));
        
        m_controller.povLeft().whileTrue(m_lintake.setState(RollerState.EJECT))
                .onFalse(m_lintake.setState(RollerState.IDLE));

        m_controller.leftBumper().onTrue(m_lintake.setState(PinionState.DEPLOY));
        m_controller.rightBumper().onTrue(m_lintake.setState(PinionState.STOW));
        m_controller.x().whileTrue(m_lintake.setState(PinionState.AGITATE));

        m_controller.rightTrigger().whileTrue(m_shooter.setState(PivotState.SCORE, ShooterState.SCORE, KickerState.FEED))
            .onFalse(m_shooter.setState(PivotState.STOW, ShooterState.IDLE, KickerState.IDLE));
        m_controller.povUp().whileTrue(m_shooter.setState(PivotState.LOB, ShooterState.LOB, KickerState.FEED))
            .onFalse(m_shooter.setState(PivotState.STOW, ShooterState.IDLE, KickerState.IDLE));
        m_controller.povRight().whileTrue(m_shooter.setState(PivotState.SEND, ShooterState.SEND, KickerState.FEED))
            .onFalse(m_shooter.setState(PivotState.STOW, ShooterState.IDLE, KickerState.IDLE));
    }

    public Command getAutonomousCommand() {
        return Auto.eightAuto(m_drivetrain, m_driveRequest, m_shooter);
    }
}
