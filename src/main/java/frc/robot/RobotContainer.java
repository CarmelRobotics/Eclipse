// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenix6.SignalLogger;
import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;

import java.util.function.BooleanSupplier;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.RobotModeTriggers;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.subsystems.diagnostics.SystemsCheck;
import frc.robot.subsystems.drive.CommandSwerveDrivetrain;
import frc.robot.subsystems.drive.TunerConstants;
import frc.robot.subsystems.lintake.Lintake;
import frc.robot.subsystems.lintake.LintakeConstants.PinionState;
import frc.robot.subsystems.lintake.LintakeConstants.RollerState;
import frc.robot.subsystems.shooter.Shooter;
import frc.robot.subsystems.shooter.ShooterConstants;
import frc.robot.util.LoggedTunableNumber;
import frc.robot.subsystems.shooter.ShooterConstants.IndexerState;
import frc.robot.subsystems.shooter.ShooterConstants.PivotState;
import frc.robot.subsystems.shooter.ShooterConstants.ShooterState;

public class RobotContainer {
  private final CommandSwerveDrivetrain m_drivetrain = TunerConstants.createDrivetrain();
  private final Lintake m_lintake = new Lintake();
  private final Shooter m_shooter = new Shooter(m_drivetrain);
  private final SystemsCheck m_systemsCheck = new SystemsCheck(m_drivetrain, m_shooter, m_lintake);
  private final CommandXboxController m_controller = new CommandXboxController(0);

  private final SwerveRequest.FieldCentric driveRequest = new SwerveRequest.FieldCentric()
      .withDeadband(Constants.kMaxSpeed * 0.1)
      .withRotationalDeadband(Constants.kMaxAngularRate * 0.1)
      .withDriveRequestType(DriveRequestType.OpenLoopVoltage);

  // Shared heading-locked request for the driver assists (trench + tower sweep).
  // PID configured once in configureBindings().
  private final SwerveRequest.FieldCentricFacingAngle assistSnapRequest =
      new SwerveRequest.FieldCentricFacingAngle();

  // X-lock: crosses all four wheels toward the robot center so it resists being pushed.
  private final SwerveRequest.SwerveDriveBrake brakeRequest = new SwerveRequest.SwerveDriveBrake();

  // === Driver assist zone geometry (blue-origin meters) ===
  // Each structure has an INSIDE zone (full assist: heading locked to the nearest 90,
  // translation locked to the axis you're facing) and a NEAR halo around it (assist as
  // a suggestion: heading snaps only while the driver isn't rotating, translation free).
  //
  // Geometry from the official 2026-rebuilt-welded AprilTag layout + game manual:
  //   - Each alliance's barrier line (guardrail-trench-bump-hub-bump-trench-guardrail)
  //     is 47 in deep in X; the hub face tags give the spans exactly (blue 4.022-5.229,
  //     red 11.312-12.519). Trench drive-under corridors are the 65.65 in (1.668 m)
  //     of that line nearest each guardrail; field width is 8.069 m.
  //   - Towers sit ON the alliance walls between DS2/DS3: 49.25 in wide, base reaching
  //     39 in onto the field. Wall tag midpoints give the centers (blue y 3.962,
  //     red y 4.108).
  private static final double kFieldWidth = 8.069;
  private static final double kFieldLength = 16.541;
  private static final double kBlueLineXMin = 4.02;
  private static final double kBlueLineXMax = 5.23;
  private static final double kRedLineXMin = 11.31;
  private static final double kRedLineXMax = 12.52;
  private static final double kTrenchDepthFromWall = 1.668;   // 65.65 in
  private static final double kLeftTrenchYMin = kFieldWidth - kTrenchDepthFromWall;  // 6.40
  private static final double kRightTrenchYMax = kTrenchDepthFromWall;
  private static final double kTowerReachX = 0.99;            // 39 in off the alliance wall
  private static final double kTowerHalfWidthY = 0.63;        // 49.25 in / 2
  private static final double kBlueTowerCenterY = 3.962;
  private static final double kRedTowerCenterY = 4.108;
  // Width of the "suggestion" halo around each structure.
  private static final double kAssistNearMargin = 0.8;
  // Re-snap to a different 90-degree increment only after rotating this far past the
  // current lock (hysteresis; must be > 45 so the lock can't flicker at the boundary).
  private static final double kAssistResnapDegrees = 60.0;

  private enum AssistZone { NONE, TRENCH_NEAR, TRENCH_IN, TOWER_NEAR, TOWER_IN }

  private Rotation2d m_assistLockHeading = null;

  // === Simulation pose setter (dashboard-based) ===
  // Set the robot's pose on the field without needing the separate sim GUI. Edit these
  // dashboard values, then click "Sim/Apply Pose" to teleport the robot there.
  private final LoggedTunableNumber m_simPoseX = new LoggedTunableNumber("Sim/PoseX", 4.6);
  private final LoggedTunableNumber m_simPoseY = new LoggedTunableNumber("Sim/PoseY", 0.75);
  private final LoggedTunableNumber m_simPoseHeadingDeg = new LoggedTunableNumber("Sim/PoseHeadingDeg", 0);

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
    // Named commands for autonomous routines (referenced by PathPlanner autos).
    NamedCommands.registerCommand("zerodrive", Commands.none());
    NamedCommands.registerCommand("intake deploy", m_lintake.setState(PinionState.GROUND));
    NamedCommands.registerCommand("intake retract", m_lintake.setState(PinionState.STOW));
    // Roller commands are runOnce lambdas since setState(RollerState) is void (not Command).
    NamedCommands.registerCommand("intake run", Commands.runOnce(() -> m_lintake.setState(RollerState.INTAKE)));
    NamedCommands.registerCommand("intake stop", Commands.runOnce(() -> m_lintake.setState(RollerState.ZERO)));
    // Shoot: prepare, wait until ready-or-timeout, feed. 1.5 s feed window in auto.
    NamedCommands.registerCommand("shoot", timedShotCommand(PivotState.SCORE, ShooterState.SCORE, 1.5));
    // Dump: same, but a long 2.5 s feed to empty a full hopper (preload + a collect
    // cycle) rather than the ~8-ball quick shot. Used by the multi-cycle trench autos.
    NamedCommands.registerCommand("dump", timedShotCommand(PivotState.SCORE, ShooterState.SCORE, 2.5));
    NamedCommands.registerCommand("stopshoot", stopShooterCommand());
  }

  private void configureBindings() {
    final SwerveRequest idle = new SwerveRequest.Idle();

    // === Drive default command ===
    // Field-centric swerve with driver translation and rotation, deadbands on both.
    m_drivetrain.setDefaultCommand(
        m_drivetrain.applyRequest(() -> driveRequest
            .withVelocityX(driverXVelocity())
            .withVelocityY(driverYVelocity())
            .withRotationalRate(driverRotationalRate())
        )
    );

    // When disabled, request idle (zero voltage out), ignoring the disabled() gate.
    RobotModeTriggers.disabled().whileTrue(
        m_drivetrain.applyRequest(() -> idle).ignoringDisable(true)
    );


    // === Driver assists: trench + tower sweep ===
    assistSnapRequest.HeadingController.setPID(8, 0, 0.2);
    assistSnapRequest.HeadingController.enableContinuousInput(-Math.PI, Math.PI);

    // Aiming outranks the assist: while right-trigger / A (hub aim) or POV-up (pass aim)
    // is held, the aim command owns the drivetrain (so you can pop out of the trench and
    // immediately shoot or ferry).
    Trigger aimHeld = m_controller.rightTrigger().or(m_controller.a()).or(m_controller.povUp());
    // X (the X-lock brake) is folded into the trigger rather than left to interrupt the
    // assist command: whileTrue only re-schedules on a fresh rising edge, so a brake tap
    // that merely INTERRUPTED the assist would kill it until the driver left and re-entered
    // the zone. With X in the trigger, releasing the brake IS a rising edge and the assist
    // resumes immediately.
    Trigger assistActive = new Trigger(() -> currentAssistZone() != AssistZone.NONE)
        .and(aimHeld.negate())
        .and(m_controller.x().negate());

    assistActive.whileTrue(m_drivetrain.applyRequest(this::assistDriveRequest));
    assistActive.onFalse(Commands.runOnce(() -> m_assistLockHeading = null));

    // Mechanisms only when actually INSIDE a structure: stow the shooter to clear the
    // bar and deploy the intake so driving through sweeps fuel. Intake restows on exit;
    // shooter stays stowed until the driver re-aims.
    Trigger insideStructure = new Trigger(() -> {
      AssistZone zone = currentAssistZone();
      return zone == AssistZone.TRENCH_IN || zone == AssistZone.TOWER_IN;
    });
    insideStructure.onTrue(Commands.runOnce(() -> {
      m_shooter.setState(PivotState.STOW);
      m_lintake.setPinionState(PinionState.GROUND);
    }));
    insideStructure.onFalse(Commands.runOnce(() -> m_lintake.setPinionState(PinionState.STOW)));

    // --- Standard driver bindings (unchanged) ---
    m_controller.povDown().onTrue(m_drivetrain.runOnce(m_drivetrain::seedFieldCentric));
    m_controller.leftBumper().onTrue(m_lintake.setState(PinionState.GROUND));
    m_controller.rightBumper().onTrue(m_lintake.setState(PinionState.STOW));

    m_controller.leftTrigger().whileTrue(
        rollerWhileHeldCommand(RollerState.INTAKE)
    );
    m_controller.povLeft().whileTrue(
        rollerWhileHeldCommand(RollerState.EJECT)
    );

    m_controller.a().whileTrue(
        Commands.parallel(
            m_drivetrain.faceHubCommand(this::shootingXVelocity, this::shootingYVelocity),
            rumbleWhenReady(m_shooter::readyToShoot)
        )
    );
    // Right trigger: context-aware score. Inside hub range (5 m) -> full hub shot;
    // beyond it -> smart pass to the alliance-zone corner. The choice LATCHES at the
    // moment the trigger is pulled so the mode can't flip mid-hold at the range
    // boundary; release and re-pull to re-decide. "aim/auto mode" on the dashboard
    // shows live which mode a pull would pick.
    m_controller.rightTrigger().whileTrue(
        Commands.either(
            Commands.parallel(
                m_drivetrain.faceHubCommand(this::shootingXVelocity, this::shootingYVelocity),
                heldShotCommand(PivotState.SCORE, ShooterState.SCORE),
                rumbleWhenReady(m_shooter::readyToShoot)
            ),
            Commands.parallel(
                m_drivetrain.facePassTargetCommand(this::shootingXVelocity, this::shootingYVelocity),
                heldPassCommand(),
                rumbleWhenReady(m_shooter::readyToPass)
            ),
            () -> m_drivetrain.getShotDistance() <= ShooterConstants.kMaxShotDistanceMeters
                && m_drivetrain.isInAllianceZone()
        )
    );
    // While Y is held, move shooter pivot to clear the shot blocker and stow the lintake.
    m_controller.y().whileTrue(
        Commands.runEnd(
            () -> {
              m_shooter.setState(PivotState.SHOT_BLOCK);
              m_lintake.setPinionState(PinionState.STOW);
            },
            () -> {
              m_shooter.setState(PivotState.STOW);
              m_lintake.setPinionState(PinionState.STOW);
            },
            m_shooter, m_lintake
        )
    );
    // Smart pass: heading-locks to the alliance-zone corner on the robot's current side
    // of the field, spins to the distance-interpolated ferry speed, and feeds once
    // actually spun up + aimed (2 s force-feed backstop).
    m_controller.povUp().whileTrue(
        Commands.parallel(
            m_drivetrain.facePassTargetCommand(this::driverXVelocity, this::driverYVelocity),
            heldPassCommand()
        )
    );
    // Manual flat-out ferry kept as a fallback (fixed 90 RPS, no aim assist).
    m_controller.povRight().whileTrue(
        heldShotCommand(PivotState.LOB, ShooterState.SEND)
    );

    // Clear jam: reverse flywheel and indexer to back out a stuck ball.
    // Hold the button until the jam clears, then release.
    m_controller.b().whileTrue(m_shooter.clearJamCommand());

    // X-lock defense brake: hold X to lock the wheels into an X so the robot can't be
    // shoved off its spot. Release to drive normally again. (X was freed up when agitate
    // became automatic during shots.)
    m_controller.x().whileTrue(m_drivetrain.applyRequest(() -> brakeRequest));

    // === Simulation pose setter (dashboard button) ===
    // Edit Sim/PoseX, Sim/PoseY, Sim/PoseHeadingDeg on the dashboard, then click
    // Sim/ApplyPose to teleport the robot there. All dashboard-driven, no controller.
    SmartDashboard.putData("Sim/ApplyPose", Commands.runOnce(() -> {
      m_drivetrain.resetPose(new edu.wpi.first.math.geometry.Pose2d(
        m_simPoseX.get(),
        m_simPoseY.get(),
        Rotation2d.fromDegrees(m_simPoseHeadingDeg.get())
      ));
    }, m_drivetrain));

    // === Systems check (pit self-test) ===
    // Read-only pass/fail sweep of every CTRE device + camera + gyro + battery. Click
    // SystemsCheck/Run on the dashboard any time (works while disabled), or just enter Test
    // mode and it runs automatically. Verdict lands in SystemsCheck/Result (+ /Report), and
    // the live Faults/* flags update every loop regardless.
    SmartDashboard.putData("SystemsCheck/Run", m_systemsCheck.fullCheckCommand());
    RobotModeTriggers.test().onTrue(m_systemsCheck.fullCheckCommand());

    // Active motor test: actually spins every mechanism + the drivetrain one at a time to
    // catch a connected-but-dead motor. THE ROBOT MOVES (drivetrain step) -- keep it on
    // blocks or a clear runway. Deliberate button only, never auto-run, and it no-ops unless
    // the robot is enabled. Per-motor PASS/FAIL lands under SystemsCheck/*.
    SmartDashboard.putData("SystemsCheck/RunMotorTest", m_systemsCheck.motorTestCommand());
  }

  // Classify the robot's position against the assist zones. INSIDE beats NEAR when
  // both could apply; trench beats tower (they don't overlap on the real field).
  private AssistZone currentAssistZone() {
    var pose = m_drivetrain.getState().Pose;
    double x = pose.getX();
    double y = pose.getY();

    // Trench corridors: on either barrier line in X, within trench depth of either wall.
    boolean trenchX = (x >= kBlueLineXMin && x <= kBlueLineXMax)
        || (x >= kRedLineXMin && x <= kRedLineXMax);
    boolean trenchNearX = (x >= kBlueLineXMin - kAssistNearMargin && x <= kBlueLineXMax + kAssistNearMargin)
        || (x >= kRedLineXMin - kAssistNearMargin && x <= kRedLineXMax + kAssistNearMargin);
    boolean trenchIn = trenchX && (y >= kLeftTrenchYMin || y <= kRightTrenchYMax);
    boolean trenchNear = trenchNearX
        && (y >= kLeftTrenchYMin - kAssistNearMargin || y <= kRightTrenchYMax + kAssistNearMargin);

    // Towers: against either alliance wall, centered on that wall's tower.
    boolean towerIn =
        (x <= kTowerReachX && Math.abs(y - kBlueTowerCenterY) <= kTowerHalfWidthY)
        || (x >= kFieldLength - kTowerReachX && Math.abs(y - kRedTowerCenterY) <= kTowerHalfWidthY);
    boolean towerNear =
        (x <= kTowerReachX + kAssistNearMargin
            && Math.abs(y - kBlueTowerCenterY) <= kTowerHalfWidthY + kAssistNearMargin)
        || (x >= kFieldLength - kTowerReachX - kAssistNearMargin
            && Math.abs(y - kRedTowerCenterY) <= kTowerHalfWidthY + kAssistNearMargin);

    AssistZone zone = trenchIn ? AssistZone.TRENCH_IN
        : towerIn ? AssistZone.TOWER_IN
        : trenchNear ? AssistZone.TRENCH_NEAR
        : towerNear ? AssistZone.TOWER_NEAR
        : AssistZone.NONE;
    SmartDashboard.putString("assist/zone", zone.toString());
    return zone;
  }

  // Drive request while an assist zone is active.
  //   NEAR (halo): the assist is a suggestion -- heading snaps to the nearest 90 only
  //     while the rotation stick is centered; any rotation input hands heading straight
  //     back to the driver. Translation is always fully manual.
  //   INSIDE: heading locked to the nearest 90 (0/90/180/270 -- 90s so you can line up
  //     a shot straight out of the trench) and translation locked to the axis you're
  //     facing, so the robot glides through without drifting into the walls.
  private SwerveRequest assistDriveRequest() {
    AssistZone zone = currentAssistZone();
    boolean inside = zone == AssistZone.TRENCH_IN || zone == AssistZone.TOWER_IN;

    boolean driverRotating = Math.abs(m_controller.getRightX()) > 0.1;
    if (!inside && driverRotating) {
      m_assistLockHeading = null;  // re-snap fresh once they let go of the stick
      return driveRequest
          .withVelocityX(driverXVelocity())
          .withVelocityY(driverYVelocity())
          .withRotationalRate(driverRotationalRate());
    }

    // Snap to the nearest 90-degree increment, with hysteresis: once locked, only
    // re-snap after the robot has rotated well past the 45-degree boundary.
    Rotation2d currentRot = m_drivetrain.getState().Pose.getRotation();
    if (m_assistLockHeading == null
        || Math.abs(MathUtil.angleModulus(currentRot.minus(m_assistLockHeading).getRadians()))
            > Math.toRadians(kAssistResnapDegrees)) {
      m_assistLockHeading = Rotation2d.fromDegrees(Math.round(currentRot.getDegrees() / 90.0) * 90.0);
    }

    double vx = driverXVelocity();
    double vy = driverYVelocity();
    if (inside) {
      // Lock translation to the facing axis: facing 0/180 keeps X and zeroes Y,
      // facing 90/270 keeps Y and zeroes X.
      double lockDeg = Math.abs(MathUtil.inputModulus(m_assistLockHeading.getDegrees(), -180, 180));
      boolean facingAlongX = lockDeg < 45.0 || lockDeg > 135.0;
      if (facingAlongX) {
        vy = 0;
      } else {
        vx = 0;
      }
    }

    return assistSnapRequest
        .withVelocityX(vx)
        .withVelocityY(vy)
        .withDeadband(Constants.kMaxSpeed * 0.1)
        .withDriveRequestType(DriveRequestType.OpenLoopVoltage)
        .withTargetDirection(m_assistLockHeading);
  }

  // Buzz the controller while `ready` is true so the driver feels the exact window the
  // shot is locked -- no need to watch the dashboard while driving and aiming. Steady
  // (not pulsed) on purpose: it marks the whole valid firing window, not just the edge.
  // Clears rumble on end (aim released) so it can never get stuck buzzing.
  private Command rumbleWhenReady(BooleanSupplier ready) {
    return Commands.run(
        () -> m_controller.getHID().setRumble(RumbleType.kBothRumble, ready.getAsBoolean() ? 1.0 : 0.0)
    ).finallyDo(() -> m_controller.getHID().setRumble(RumbleType.kBothRumble, 0.0));
  }

  private Command rollerWhileHeldCommand(RollerState state) {
    return Commands.runEnd(
        () -> m_lintake.setState(state),
        () -> m_lintake.setState(RollerState.ZERO),
        m_lintake
    );
  }

  // === Lintake agitation pump ===
  // While a shot runs, bounce the pinion between AGITATE and GROUND so the fuel pile
  // keeps flowing into the indexer instead of bridging over it (30 balls in a hopper
  // pack together fast once the bottom layer drains). Deliberately requirement-free:
  // it only writes the pinion state, so the roller command (left trigger) and the
  // shot command keep running alongside it without cancelling anything. The pinion is
  // left deployed wherever the pump ends; the bumpers still own stow/deploy.
  private Command lintakePumpCommand() {
    return Commands.sequence(
        Commands.runOnce(() -> m_lintake.setPinionState(PinionState.AGITATE)),
        Commands.waitSeconds(0.3),
        Commands.runOnce(() -> m_lintake.setPinionState(PinionState.GROUND)),
        Commands.waitSeconds(0.3)
    ).repeatedly();
  }

  // === Timed shot (autonomous) ===
  // Shot sequence: prepare (set pivot + spin up) → wait until readyToShoot() OR 1.25 s
  // timeout expires (whichever first) → feed for feedSeconds. The timeout ensures shots
  // never hang with the flywheel running forever; after the timeout, feed unconditionally.
  // The lintake pump runs alongside the whole sequence and dies with it (deadline).
  // finallyDo() guarantees motors stop even if the command is interrupted mid-sequence.
  private Command timedShotCommand(PivotState pivotState, ShooterState shooterState, double feedSeconds) {
    return Commands.deadline(
        Commands.sequence(
            prepareShotCommand(pivotState, shooterState),
            Commands.waitUntil(m_shooter::readyToShoot).withTimeout(ShooterConstants.kShotSpinupTimeoutSeconds),
            Commands.run(this::feed, m_shooter).withTimeout(feedSeconds)
        ),
        lintakePumpCommand()
    ).finallyDo(interrupted -> stopShooter());
  }

  // === Held shot (teleop) ===
  // Same sequence as timedShotCommand, but the feed runs until the command is manually
  // released by the driver (no withTimeout on feed). Still uses readyToShoot/timeout so
  // the shot can't hang mid-spin.
  private Command heldShotCommand(PivotState pivotState, ShooterState shooterState) {
    return Commands.deadline(
        Commands.sequence(
            prepareShotCommand(pivotState, shooterState),
            Commands.waitUntil(m_shooter::readyToShoot).withTimeout(ShooterConstants.kShotSpinupTimeoutSeconds),
            Commands.run(this::feed, m_shooter)
        ),
        lintakePumpCommand()
    ).finallyDo(interrupted -> stopShooter());
  }

  // === Held pass (teleop) ===
  // Same shape as heldShotCommand, but gated on readyToPass() -- flywheel at the
  // distance-interpolated ferry speed, pivot at max feed, aimed at the pass corner --
  // so passes fire on actual spin-up. Longer timeout backstop: ferry speeds are higher,
  // so spin-up takes longer than a hub shot's.
  private Command heldPassCommand() {
    return Commands.deadline(
        Commands.sequence(
            prepareShotCommand(PivotState.LOB, ShooterState.PASS),
            Commands.waitUntil(m_shooter::readyToPass).withTimeout(ShooterConstants.kPassSpinupTimeoutSeconds),
            Commands.run(this::feed, m_shooter)
        ),
        lintakePumpCommand()
    ).finallyDo(interrupted -> stopShooter());
  }

  // === Shot prep ===
  // Set both pivot and shooter state in one command, run once on shot trigger.
  private Command prepareShotCommand(PivotState pivotState, ShooterState shooterState) {
    return Commands.runOnce(() -> {
      m_shooter.setState(pivotState);
      m_shooter.setState(shooterState);
    }, m_shooter);
  }

  // === Stop shooter (for named commands / direct calls) ===
  private Command stopShooterCommand() {
    return Commands.runOnce(this::stopShooter, m_shooter);
  }

  // === Shooter stop (imperative) ===
  // Zeroes the indexer, flywheel, and pivot in one go. Called by finallyDo() on shot
  // command exit, guaranteeing cleanup no matter where the sequence is interrupted.
  private void stopShooter() {
    m_shooter.setState(IndexerState.ZERO);
    m_shooter.setState(ShooterState.ZERO);
    m_shooter.setState(PivotState.STOW);
  }

  // === Feed (unconditional) ===
  // Run the indexer. Only reached after readyToShoot() *or* the spinup timeout, so this
  // is where the "force-feed after timeout" behavior happens. The command continues until
  // timed out (autonomous) or manually released (teleop).
  private void feed() {
    m_shooter.setState(IndexerState.SCORE);
  }

  private double driverXVelocity() {
    return -m_controller.getLeftY() * Constants.kMaxSpeed;
  }

  private double driverYVelocity() {
    return -m_controller.getLeftX() * Constants.kMaxSpeed;
  }

  // While aiming (hub or pass), scale translation down so shoot-on-the-move is more
  // accurate: less speed = smaller velocity lead and less stick/velocity noise fed into
  // the aim. Full stick still works, just capped at this fraction of top speed. Tune the
  // mobility-vs-accuracy tradeoff live; 1.0 = no reduction.
  private final LoggedTunableNumber m_shootingSpeedScale =
      new LoggedTunableNumber("ShotTuning/ShootingSpeedScale", 0.45);

  private double shootingXVelocity() {
    return driverXVelocity() * m_shootingSpeedScale.get();
  }

  private double shootingYVelocity() {
    return driverYVelocity() * m_shootingSpeedScale.get();
  }

  private double driverRotationalRate() {
    return -m_controller.getRightX() * Constants.kMaxAngularRate;
  }

  public Command getAutonomousCommand() {
    return autoSelection.getSelected();
  }
}