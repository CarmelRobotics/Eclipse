// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenix6.SignalLogger;
import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.RobotModeTriggers;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
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

  // Shared snap request for trench heading lock.
  // Reused every loop to avoid allocations; PID configured once in configureBindings().
  private final SwerveRequest.FieldCentricFacingAngle trenchSnapRequest =
      new SwerveRequest.FieldCentricFacingAngle();

  // Trench snap hysteresis
  private Rotation2d m_trenchLockHeading = null;
  private static final double kTrenchHysteresisDegrees = 30.0; // changeable if desired

  // Trench Y-coordinate bounds (blue-origin field coordinates).
  // Left trench wall is at ~7.62m, right trench wall is at ~0.38m.
  // The 0.3m buffer means the lock engages just before the robot is fully inside.
  private static final double LEFT_TRENCH_Y_MIN  = 7.3;
  private static final double RIGHT_TRENCH_Y_MAX = 0.7;

  private final SendableChooser<Command> autoSelection;

  public RobotContainer() {
    SignalLogger.enableAutoLogging(false);
    registerNamedCommands();

    autoSelection = AutoBuilder.buildAutoChooser();
    SmartDashboard.putData("Auto", autoSelection);

    configureBindings();
    configureSysIdButtons();
  }

  // Flywheel SysId as clickable dashboard buttons (no controller conflict). Run each
  // with the robot enabled in Test mode, shooter clamped and empty, then analyze the
  // resulting .hoot in Tuner X to get kS/kV/kA for the ShooterConfigs.
  private void configureSysIdButtons() {
    SmartDashboard.putData("SysId/Shooter Quasistatic Fwd",
        m_shooter.sysIdQuasistatic(SysIdRoutine.Direction.kForward));
    SmartDashboard.putData("SysId/Shooter Quasistatic Rev",
        m_shooter.sysIdQuasistatic(SysIdRoutine.Direction.kReverse));
    SmartDashboard.putData("SysId/Shooter Dynamic Fwd",
        m_shooter.sysIdDynamic(SysIdRoutine.Direction.kForward));
    SmartDashboard.putData("SysId/Shooter Dynamic Rev",
        m_shooter.sysIdDynamic(SysIdRoutine.Direction.kReverse));
  }

  private void registerNamedCommands() {
    NamedCommands.registerCommand("zerodrive", Commands.none());
    NamedCommands.registerCommand("intake deploy", m_lintake.setState(PinionState.GROUND));
    NamedCommands.registerCommand("intake retract", m_lintake.setState(PinionState.STOW));
    NamedCommands.registerCommand("intake run", Commands.runOnce(() -> m_lintake.setState(RollerState.INTAKE)));
    NamedCommands.registerCommand("intake stop", Commands.runOnce(() -> m_lintake.setState(RollerState.ZERO)));
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

    // --- Trench heading lock ---
    // Configure the heading PID once here. Same gains as faceHubCommand.
    trenchSnapRequest.HeadingController.setP(8.0);
    trenchSnapRequest.HeadingController.enableContinuousInput(-Math.PI, Math.PI);

    // True when the robot's Y coordinate is inside either trench zone.
    Trigger inTrench = new Trigger(() -> {
      double y = m_drivetrain.getState().Pose.getY();
      return y >= LEFT_TRENCH_Y_MIN || y <= RIGHT_TRENCH_Y_MAX;
    });

    // While in the trench:
    //   - lock heading to 180° (blue) / 0° (red) so intake faces into the trench
    //   - stow the shooter pivot so it doesn't clip the trench bar
    //   - deploy the lintake to GROUND so the driver can just drive through and pick up
    // onFalse undoes the mechanical actions when the robot exits.
    inTrench.whileTrue(
        Commands.parallel(
            // Drive with heading locked to trench angle; driver still controls translation.
            m_drivetrain.applyRequest(() -> {
        // Snap to whichever heading (0° or 180°) the robot is currently closer to.
        Rotation2d currentRot = m_drivetrain.getState().Pose.getRotation();
        Rotation2d target0 = Rotation2d.fromDegrees(0);
        Rotation2d target180 = Rotation2d.fromDegrees(180);
              double diff0 = Math.abs(MathUtil.angleModulus(currentRot.minus(target0).getRadians()));
              double diff180 = Math.abs(MathUtil.angleModulus(currentRot.minus(target180).getRadians()));
              Rotation2d nearest = diff0 <= diff180 ? target0 : target180;
              double hystRad = Math.toRadians(kTrenchHysteresisDegrees);
              if (m_trenchLockHeading == null) {
                m_trenchLockHeading = nearest;
              } else {
                // Only flip if the nearest heading is sufficiently farther from the current lock
                double distToCurrent = Math.abs(MathUtil.angleModulus(currentRot.minus(m_trenchLockHeading).getRadians()));
                if (nearest != m_trenchLockHeading && distToCurrent > hystRad) {
                  m_trenchLockHeading = nearest;
                }
              }
              Rotation2d lockHeading = m_trenchLockHeading;
              return trenchSnapRequest
                  .withVelocityX(driverXVelocity())
                  .withVelocityY(driverYVelocity())
                  .withDeadband(Constants.kMaxSpeed * 0.1)
                  .withDriveRequestType(DriveRequestType.OpenLoopVoltage)
                  .withTargetDirection(lockHeading);
            }),
            // Stow shooter so it clears the trench bar
            Commands.runOnce(() -> m_shooter.setState(PivotState.STOW), m_shooter),
            // Deploy lintake to ground for immediate intaking as you drive through
            m_lintake.setState(PinionState.GROUND)
        )
    );

    // When exiting the trench, stow the lintake back.
    // Shooter stays stowed — driver can re-spin it with rightTrigger as normal.
    inTrench.onFalse(m_lintake.setState(PinionState.STOW));

    // --- Standard driver bindings (unchanged) ---
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
  // While Y is held, move shooter pivot to clear the shot blocker and stow the lintake.
  m_controller.y().whileTrue(
    Commands.runEnd(
      () -> {
        m_shooter.setState(PivotState.SHOT_BLOCK);
        m_lintake.setState(PinionState.STOW);
      },
      () -> {
        m_shooter.setState(PivotState.STOW);
        m_lintake.setState(PinionState.STOW);
      },
      m_shooter, m_lintake
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

  // Shot sequence: aim/spin up, wait until readyToShoot() OR the spinup timeout
  // expires, then feed. The waitUntil completes on whichever comes first, so if the
  // shot never satisfies every readiness gate it force-feeds after
  // kShotSpinupTimeoutSeconds rather than hanging with the flywheel spinning forever.
  private Command timedShotCommand(PivotState pivotState, ShooterState shooterState, double feedSeconds) {
    return Commands.sequence(
        prepareShotCommand(pivotState, shooterState),
        Commands.waitUntil(m_shooter::readyToShoot).withTimeout(ShooterConstants.kShotSpinupTimeoutSeconds),
        Commands.run(this::feed, m_shooter).withTimeout(feedSeconds)
    ).finallyDo(interrupted -> stopShooter());
  }

  private Command heldShotCommand(PivotState pivotState, ShooterState shooterState) {
    return Commands.sequence(
        prepareShotCommand(pivotState, shooterState),
        Commands.waitUntil(m_shooter::readyToShoot).withTimeout(ShooterConstants.kShotSpinupTimeoutSeconds),
        Commands.run(this::feed, m_shooter)
    ).finallyDo(interrupted -> stopShooter());
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

  // Unconditionally run the indexer. Only reached after the readyToShoot()/timeout
  // wait, so this is where the "shoot anyway after the timeout" behavior happens.
  private void feed() {
    m_shooter.setState(IndexerState.SCORE);
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