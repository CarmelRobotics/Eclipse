// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenix6.SignalLogger;
import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.util.FlippingUtil;

import java.util.function.BooleanSupplier;
import java.util.IdentityHashMap;
import java.util.Map;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.RobotModeTriggers;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.config.FeatureFlags;
import frc.robot.auto.AutoTiming;
import frc.robot.subsystems.diagnostics.SystemsCheck;
import frc.robot.subsystems.drive.CommandSwerveDrivetrain;
import frc.robot.subsystems.drive.TunerConstants;
import frc.robot.subsystems.power.PowerManager;
import frc.robot.subsystems.power.PowerMode;
import frc.robot.subsystems.lintake.Lintake;
import frc.robot.subsystems.lintake.LintakeConstants.PinionState;
import frc.robot.subsystems.lintake.LintakeConstants.RollerState;
import frc.robot.subsystems.shooter.Shooter;
import frc.robot.subsystems.shooter.ShooterConstants;
import frc.robot.util.LoggedTunableNumber;
import frc.robot.subsystems.shooter.ShooterConstants.IndexerState;
import frc.robot.subsystems.shooter.ShooterConstants.PivotState;
import frc.robot.subsystems.shooter.ShooterConstants.ShooterState;
import frc.robot.subsystems.strategy.ZoneController;

public class RobotContainer {
  // A stale dashboard feature flag must not re-enable driver assists in competition.
  private static final boolean kCompetitionDriverAssists = false;

  private final Timer m_autoTimer = new Timer();
  private boolean m_autoTravelReady = true;
  private final Map<Command, Command> m_guardedAutos = new IdentityHashMap<>();
  private final SwerveRequest.FieldCentricFacingAngle m_autoTravelRequest =
      new SwerveRequest.FieldCentricFacingAngle();
  private final CommandSwerveDrivetrain m_drivetrain = TunerConstants.createDrivetrain();
  private final Lintake m_lintake = new Lintake(m_drivetrain);
  private final Shooter m_shooter = new Shooter(m_drivetrain);
  private final ZoneController m_zoneController = new ZoneController(m_drivetrain);
  private final SystemsCheck m_systemsCheck = new SystemsCheck(m_drivetrain, m_shooter, m_lintake);
  // Drive supply: 91 A only in auto, 40 A otherwise, with low-voltage fallback.
  // Optional flywheel budget changes remain gated by FeatureFlags.POWER_MANAGER.
  private final PowerManager m_powerManager =
      new PowerManager(m_drivetrain, m_shooter, this::desiredPowerMode);
  private final CommandXboxController m_controller = new CommandXboxController(0);

  // Built ONCE: every CommandXboxController button accessor (rightTrigger(), a(), povUp())
  // constructs a brand-new Trigger object per call. desiredPowerMode() runs at 50 Hz from
  // the PowerManager, so calling the accessors inline there allocated three Triggers per
  // loop -- steady garbage churn that worsened the OOM pressure. Poll this shared field
  // instead.
  private final Trigger m_aimHeld =
      m_controller.rightTrigger().or(m_controller.a()).or(m_controller.povUp());

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
    NamedCommands.registerCommand("auto start",
        autoTravelCommand(AutoTiming.INITIAL_STOW_TIMEOUT_SECONDS));
    // Eclipse starts sideways: the rear intake faces inward before the rush.
    // Keep legacy "auto start" at 180 for the other saved routines.
    NamedCommands.registerCommand("auto start left",
        autoTravelCommand(AutoTiming.INITIAL_STOW_TIMEOUT_SECONDS, Rotation2d.fromDegrees(90))
            .beforeStarting(() -> m_lintake.setState(RollerState.ZERO)));
    NamedCommands.registerCommand("auto start right",
        autoTravelCommand(AutoTiming.INITIAL_STOW_TIMEOUT_SECONDS, Rotation2d.fromDegrees(-90))
            .beforeStarting(() -> m_lintake.setState(RollerState.ZERO)));
    NamedCommands.registerCommand("auto travel",
        autoTravelCommand(AutoTiming.TRAVEL_TIMEOUT_SECONDS));
    NamedCommands.registerCommand("auto cycle 2 left", secondCycleCommand(Rotation2d.fromDegrees(-90)));
    NamedCommands.registerCommand("auto cycle 2 right", secondCycleCommand(Rotation2d.fromDegrees(90)));
    // Begin near the scoring bearing, then align to 180 within the clear alliance-side
    // corner. The new path still locks its heading before any trench overlap.
    NamedCommands.registerCommand("max cycle 2 left", secondCycleCommand(Rotation2d.fromDegrees(-45),
        AutoTiming.MAX_AGGRESSION_SECOND_CYCLE_RESERVE_SECONDS));
    NamedCommands.registerCommand("max cycle 2 right", secondCycleCommand(Rotation2d.fromDegrees(45),
        AutoTiming.MAX_AGGRESSION_SECOND_CYCLE_RESERVE_SECONDS));
    NamedCommands.registerCommand("intake deploy", m_lintake.setState(PinionState.GROUND));
    var collectBrake = new SwerveRequest.SwerveDriveBrake();
    NamedCommands.registerCommand("auto collect settle", Commands.deadline(
        Commands.waitSeconds(AutoTiming.COLLECT_SETTLE_SECONDS),
        m_drivetrain.applyRequest(() -> collectBrake)));
    NamedCommands.registerCommand("intake retract", m_lintake.setState(PinionState.STOW));
    // Roller commands are runOnce lambdas since setState(RollerState) is void (not Command).
    NamedCommands.registerCommand("intake run", Commands.runOnce(() -> m_lintake.setState(RollerState.INTAKE)));
    NamedCommands.registerCommand("intake stop", Commands.runOnce(() -> m_lintake.setState(RollerState.ZERO)));
    // Both initial dumps feed for the team's requested five seconds. Stop collecting
    // while unloading/repositioning; the second path restarts intake AFTER the trench.
    NamedCommands.registerCommand("shoot",
        timedShotCommand(PivotState.SCORE, ShooterState.SCORE, AutoTiming.DUMP_FEED_SECONDS)
            .beforeStarting(() -> m_lintake.setState(RollerState.ZERO)));
    NamedCommands.registerCommand("dump", timedShotCommand(PivotState.SCORE, ShooterState.SCORE,
        AutoTiming.DUMP_FEED_SECONDS).beforeStarting(() -> m_lintake.setState(RollerState.ZERO)));
    // Spend all remaining autonomous time unloading after the final collection.
    // The outer 19.8-second guard interrupts this command and cleans up outputs.
    NamedCommands.registerCommand("auto finish shoot", timedShotCommand(
        PivotState.SCORE, ShooterState.SCORE, AutoTiming.DURATION_SECONDS));
    NamedCommands.registerCommand("stopshoot", stopShooterCommand());
  }

  private Command secondCycleCommand(Rotation2d blueHeading) {
    return secondCycleCommand(blueHeading, AutoTiming.SECOND_CYCLE_RESERVE_SECONDS);
  }

  private Command secondCycleCommand(Rotation2d blueHeading, double reserveSeconds) {
    Command finishHere = timedShotCommand(PivotState.SCORE, ShooterState.SCORE,
        AutoTiming.DURATION_SECONDS).beforeStarting(() -> SmartDashboard.putString("Auto/Status",
            "Cycle 2 skipped; finish unloading at first score"));
    return Commands.either(
        Commands.sequence(
            autoTravelCommand(AutoTiming.TRAVEL_TIMEOUT_SECONDS, blueHeading),
            Commands.either(Commands.none(), finishHere,
                () -> AutoTiming.canStartSecondCycle(m_autoTimer.get(), reserveSeconds))
                .onlyIf(() -> m_autoTravelReady)),
        // If the trench detour and a useful final shot cannot fit, keep unloading.
        // The outer guard stops this branch without starting the following path.
        timedShotCommand(PivotState.SCORE, ShooterState.SCORE, AutoTiming.DURATION_SECONDS)
            .beforeStarting(() -> SmartDashboard.putString("Auto/Status", "Cycle 2 skipped before preparation")),
        () -> AutoTiming.canStartSecondCycle(m_autoTimer.get(), reserveSeconds));
  }

  private void configureBindings() {
    final SwerveRequest idle = new SwerveRequest.Idle();
    final Trigger teleop = new Trigger(DriverStation::isTeleopEnabled);

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
    m_autoTravelRequest.HeadingController.setPID(8, 0, 0.2);
    m_autoTravelRequest.HeadingController.enableContinuousInput(-Math.PI, Math.PI);

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
        .and(m_controller.x().negate())
        .and(FeatureFlags.DRIVER_ASSISTS)
        .and(() -> kCompetitionDriverAssists)
        .and(teleop);

    assistActive.whileTrue(m_drivetrain.applyRequest(this::assistDriveRequest));
    assistActive.onFalse(Commands.runOnce(() -> m_assistLockHeading = null));

    // Mechanisms only when actually INSIDE a structure: stow the shooter to clear the
    // bar and deploy the intake so driving through sweeps fuel. Intake restows on exit;
    // shooter stays stowed until the driver re-aims.
    Trigger insideStructure = new Trigger(() -> {
      AssistZone zone = currentAssistZone();
      return zone == AssistZone.TRENCH_IN || zone == AssistZone.TOWER_IN;
    });
    // insideStructure.onTrue(Commands.runOnce(() -> {
    //   m_shooter.setState(PivotState.STOW);
    //   m_lintake.setPinionState(PinionState.GROUND);
    // }));
    //insideStructure.onFalse(Commands.runOnce(() -> m_lintake.setPinionState(PinionState.STOW)));

    // Driver buttons must not interrupt the autonomous command or reset its heading.
    // Explicit absolute heading, independent of our reversed driver-stick perspective.
    // Point robot +X/front AWAY from our alliance wall; can also seed while disabled.
    m_controller.povDown().and(new Trigger(DriverStation::isDisabled).or(teleop))
        .onTrue(m_drivetrain.runOnce(m_drivetrain::seedDownfieldHeading).ignoringDisable(true));
    m_controller.leftBumper().and(teleop).onTrue(m_lintake.setState(PinionState.GROUND));
    m_controller.rightBumper().and(teleop).onTrue(m_lintake.setState(PinionState.STOW));
    m_controller.button(8).and(teleop).onTrue(m_lintake.zero());

    m_controller.leftTrigger().and(teleop).whileTrue(
        rollerWhileHeldCommand(RollerState.INTAKE)
    );
    m_controller.povLeft().and(teleop).whileTrue(
        rollerWhileHeldCommand(RollerState.EJECT)
    );

    m_controller.a().and(teleop).whileTrue(
        Commands.parallel(
            m_drivetrain.faceHubCommand(this::shootingXVelocity, this::shootingYVelocity, this::driverRotationalRate),
            rumbleWhenReady(m_shooter::readyToShoot)
        )
    );
    // Right trigger: context-aware score. Inside hub range (5 m) -> full hub shot;
    // beyond it -> smart pass to the alliance-zone corner. The choice LATCHES at the
    // moment the trigger is pulled so the mode can't flip mid-hold at the range
    // boundary; release and re-pull to re-decide. "aim/auto mode" on the dashboard
    // shows live which mode a pull would pick.
    m_controller.rightTrigger().and(teleop).whileTrue(
        Commands.either(
            Commands.parallel(
                m_drivetrain.faceHubCommand(this::shootingXVelocity, this::shootingYVelocity, this::driverRotationalRate),
                heldShotCommand(PivotState.SCORE, ShooterState.SCORE),
                rumbleWhenReady(m_shooter::readyToShoot)
            ),
            Commands.parallel(
                m_drivetrain.facePassTargetCommand(this::shootingXVelocity, this::shootingYVelocity, this::driverRotationalRate),
                heldPassCommand(),
                rumbleWhenReady(m_shooter::readyToPass)
            ),
            // HUB only when it would actually score: in range, legally in our zone, and our
            // hub active per the SHIFTS game data (shop/no-game-data defaults to active, so
            // practice is unaffected). During the opponent's shift the same pull ferries
            // instead of wasting balls on a dead hub. Latches at the pull like everything
            // else -- a shift flip mid-hold doesn't yank the mode out from under the driver.
            () -> m_drivetrain.getShotDistance() <= ShooterConstants.kMaxShotDistanceMeters
                && (!FeatureFlags.ZONE_CONTROLLER.getAsBoolean() || m_zoneController.shouldShootHub())
                && (m_drivetrain.isInAllianceZone() || !FeatureFlags.ALLIANCE_ZONE_GATE.getAsBoolean())
                && (m_drivetrain.isOurHubActive() || !FeatureFlags.SHIFTS_GATE.getAsBoolean())
        )
    );
    // While Y is held, move shooter pivot to clear the shot blocker and stow the lintake.
    m_controller.y().and(teleop).whileTrue(
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
    m_controller.povUp().and(teleop).whileTrue(
        Commands.parallel(
            m_drivetrain.facePassTargetCommand(this::driverXVelocity, this::driverYVelocity, this::driverRotationalRate),
            heldPassCommand()
        )
    );
    // Manual flat-out ferry kept as a fallback (fixed 90 RPS, no aim assist).
    m_controller.povRight().and(teleop).whileTrue(
        heldFerryCommand()
    );

    // Clear jam: reverse flywheel and indexer to back out a stuck ball.
    // Hold the button until the jam clears, then release.
    m_controller.b().and(teleop).whileTrue(m_shooter.clearJamCommand());

    // X-lock defense brake: hold X to lock the wheels into an X so the robot can't be
    // shoved off its spot. Release to drive normally again. (X was freed up when agitate
    // became automatic during shots.)
    m_controller.x().and(teleop).whileTrue(m_drivetrain.applyRequest(() -> brakeRequest));

    // === Simulation pose setter (dashboard button) ===
    // Edit Sim/PoseX, Sim/PoseY, Sim/PoseHeadingDeg on the dashboard, then click
    // Sim/ApplyPose to teleport the robot there. All dashboard-driven, no controller.
    SmartDashboard.putData("Sim/ApplyPose", Commands.runOnce(() -> {
      if (!edu.wpi.first.wpilibj.RobotBase.isSimulation()) return;
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

    // fieldToOperatorBearing: FieldCentricFacingAngle rotates its target by the operator
    // perspective (180 deg on a Red DS). The lock is a blue-origin field bearing, so it must
    // be pre-rotated or the assist chases a target ~180 deg away and, because the lock
    // re-snaps every 60 deg of rotation, spins in place at full speed forever.
    return assistSnapRequest
        .withVelocityX(vx)
        .withVelocityY(vy)
        .withDeadband(Constants.kMaxSpeed * 0.1)
        .withDriveRequestType(DriveRequestType.OpenLoopVoltage)
        .withTargetDirection(m_drivetrain.fieldToOperatorBearing(m_assistLockHeading));
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
    return Commands.waitSeconds(1).andThen((Commands.sequence(
        Commands.runOnce(() -> m_lintake.setPinionState(PinionState.AGITATE)),
        Commands.waitSeconds(0.3),
        Commands.runOnce(() -> m_lintake.setPinionState(PinionState.GROUND)),
        Commands.waitSeconds(0.3)
    )).repeatedly());
  }

  // === Timed shot (autonomous) ===
  // Shot sequence: prepare (set pivot + spin up) → wait until readyToShoot() OR 1.25 s
  // timeout expires (whichever first) → feed for feedSeconds. The timeout ensures shots
  // never hang with the flywheel running forever. The timeout cannot bypass the outer
  // field-localization guard; missing position/heading aborts instead of feeding blind.
  // The lintake pump runs alongside the whole sequence and dies with it (deadline).
  // finallyDo() guarantees motors stop even if the command is interrupted mid-sequence.
  private Command timedShotCommand(PivotState pivotState, ShooterState shooterState, double feedSeconds) {
    return Commands.deadline(
        Commands.sequence(
            prepareShotCommand(pivotState, shooterState),
            Commands.waitUntil(m_shooter::readyToShoot).withTimeout(ShooterConstants.kShotSpinupTimeoutSeconds),
            m_shooter.feedWithRecoveryCommand().withTimeout(feedSeconds)
        ),
        lintakePumpCommand(),
        m_drivetrain.faceHubCommand(() -> 0.0, () -> 0.0)
    ).onlyWhile(m_drivetrain::isFieldAimingReady)
        .finallyDo(interrupted -> stopShooter())
        .onlyIf(m_drivetrain::isFieldAimingReady)
        .beforeStarting(() -> {
          if (!m_drivetrain.isFieldAimingReady()) m_autoTravelReady = false;
        });
  }

  private Rotation2d autoTravelHeading(Rotation2d blueHeading) {
    return m_drivetrain.isRedAlliance()
        ? FlippingUtil.flipFieldRotation(blueHeading) : blueHeading;
  }

  private boolean readyForAutoTravel(Rotation2d heading) {
    return m_drivetrain.hasAlliance() && m_shooter.isStowedForTravel()
        && Math.abs(m_drivetrain.getState().Pose.getRotation()
            .minus(heading).getDegrees()) <= 4.0
        && Math.abs(m_drivetrain.getState().Speeds.omegaRadiansPerSecond)
            <= AutoTiming.MAX_TRAVEL_ANGULAR_SPEED_RAD_PER_SEC;
  }

  /** Stow and align before moving; a failed clearance check aborts the remaining routine. */
  private Command autoTravelCommand(double timeoutSeconds) {
    return autoTravelCommand(timeoutSeconds, Rotation2d.k180deg);
  }

  private Command autoTravelCommand(double timeoutSeconds, Rotation2d blueHeading) {
    // Hold the selected start heading (+90 left / -90 right for Eclipse), or the
    // cycle-two wall-side reposition heading (-90 left / +90 right), or the new
    // maximum profile's near-shot heading (-45 left / +45 right). Alliance
    // flipping and operator-perspective conversion happen exactly once below.
    return Commands.sequence(
        Commands.runOnce(() -> {
          stopShooter();
          m_lintake.setPinionState(PinionState.GROUND);
        }, m_shooter, m_lintake),
        Commands.deadline(
            Commands.waitUntil(() -> readyForAutoTravel(autoTravelHeading(blueHeading)))
                .withTimeout(timeoutSeconds),
            m_drivetrain.applyRequest(() -> m_autoTravelRequest
                .withVelocityX(0).withVelocityY(0)
                .withTargetDirection(m_drivetrain.fieldToOperatorBearing(autoTravelHeading(blueHeading))))),
        Commands.runOnce(() -> {
          m_autoTravelReady = readyForAutoTravel(autoTravelHeading(blueHeading));
          if (!m_autoTravelReady) {
            SmartDashboard.putString("Auto/Status", "Stopped: shooter not stowed or heading not aligned");
            DriverStation.reportWarning("Auto stopped: travel clearance/alignment timed out", false);
          }
        })
    );
  }

  // === Held shot (teleop) ===
  // Same sequence as timedShotCommand, but the feed runs until the command is manually
  // released by the driver (no withTimeout on feed). Still uses readyToShoot/timeout so
  // the shot can't hang mid-spin.
  private Command heldShotCommand(PivotState pivotState, ShooterState shooterState) {
    return Commands.runOnce(()->m_lintake.setState(RollerState.AGITATE)).andThen(Commands.deadline(
        Commands.sequence(
            prepareShotCommand(pivotState, shooterState),
            Commands.waitUntil(m_shooter::readyToShoot).withTimeout(ShooterConstants.kShotSpinupTimeoutSeconds),
            m_shooter.feedWithRecoveryCommand()
        ),
        lintakePumpCommand()
    )).onlyWhile(m_drivetrain::isFieldAimingReady).finallyDo(interrupted -> {
      stopShooter();
      m_lintake.setPinionState(PinionState.GROUND);
      m_lintake.setState(RollerState.ZERO);
    }).onlyIf(m_drivetrain::isFieldAimingReady);
  }

  // Manual flat-out ferry. This has its own readiness gate because readyToShoot() checks the
  // distance-interpolated SCORE target, which can never match the fixed SEND state at 90 RPS.
  private Command heldFerryCommand() {
    return Commands.runOnce(() -> m_lintake.setState(RollerState.AGITATE)).andThen(Commands.deadline(
        Commands.sequence(
            prepareShotCommand(PivotState.LOB, ShooterState.SEND),
            Commands.waitUntil(m_shooter::readyToFerry)
                .withTimeout(ShooterConstants.kFerrySpinupTimeoutSeconds),
            m_shooter.feedWithRecoveryCommand()
        ),
        lintakePumpCommand()
    )).finallyDo(interrupted -> {
      stopShooter();
      m_lintake.setPinionState(PinionState.GROUND);
      m_lintake.setState(RollerState.ZERO);
    });
  }

  // === Held pass (teleop) ===
  // Same shape as heldShotCommand, but gated on readyToPass() -- flywheel at the
  // distance-interpolated ferry speed, pivot at the flatter-than-LOB pass angle
  // (backspin bounce-back mitigation; see ShooterConstants.kPassPivotPosition), aimed
  // at the pass corner -- so passes fire on actual spin-up. Longer timeout backstop:
  // ferry speeds are higher, so spin-up takes longer than a hub shot's.
  private Command heldPassCommand() {
    return Commands.deadline(
        Commands.sequence(
            prepareShotCommand(PivotState.PASS, ShooterState.PASS),
            Commands.waitUntil(m_shooter::readyToPass).withTimeout(ShooterConstants.kPassSpinupTimeoutSeconds),
            m_shooter.feedWithRecoveryCommand()
        ),
        lintakePumpCommand()
    ).onlyWhile(m_drivetrain::isFieldAimingReady)
        .finallyDo(interrupted -> stopShooter()).onlyIf(m_drivetrain::isFieldAimingReady);
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

  // Optional flywheel budget selection; the separate drive policy always restores
  // 40 A supply outside auto. Stability filtering prevents config writes on stick jitter.
  private PowerMode desiredPowerMode() {
    if (DriverStation.isAutonomousEnabled()) {
      return PowerMode.AUTO;
    }
    if (m_aimHeld.getAsBoolean()) {
      return PowerMode.SCORING;
    }
    double stickMagnitude = Math.hypot(m_controller.getLeftX(), m_controller.getLeftY());
    if (stickMagnitude > 0.9) {
      return PowerMode.SPRINT;
    }
    return PowerMode.IDLE;
  }

  public Command getAutonomousCommand() {
    Command selected = autoSelection.getSelected();
    if (selected == null) {
      selected = Commands.none();
    }
    // WPILib commands may be composed only once. Cache the wrapper so repeated practice
    // enables of the same chooser option do not throw an "already composed" exception.
    return m_guardedAutos.computeIfAbsent(selected, this::guardAutonomousCommand);
  }

  public void prepareForMatch() { m_drivetrain.prepareForMatch(); }

  private Command guardAutonomousCommand(Command selected) {
    return AutoTiming.guard(selected, () -> m_autoTravelReady, () -> {
          m_autoTravelReady = true;
          m_autoTimer.restart();
          SmartDashboard.putString("Auto/Status", "Running");
        }, this::stopAutoOutputs)
        // Keep control of the drive after finishing; stick positions must not move it in auto.
        .andThen(m_drivetrain.applyRequest(() -> brakeRequest))
        .finallyDo(interrupted -> stopAutoOutputs());
  }

  private void stopAutoOutputs() {
    if (m_autoTimer.isRunning() && m_autoTravelReady) {
      SmartDashboard.putString("Auto/Status", m_autoTimer.hasElapsed(AutoTiming.DURATION_SECONDS)
          ? "Stopped at autonomous time limit" : "Complete");
    }
    stopShooter();
    m_lintake.setState(RollerState.ZERO);
    m_lintake.setPinionState(PinionState.GROUND);
    m_drivetrain.setControl(brakeRequest);
    m_autoTimer.stop();
  }
}
