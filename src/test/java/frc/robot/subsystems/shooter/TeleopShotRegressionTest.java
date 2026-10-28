package frc.robot.subsystems.shooter;

import static org.junit.jupiter.api.Assertions.*;

import com.ctre.phoenix6.SignalLogger;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.ParentDevice;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.swerve.SwerveRequest;
import com.pathplanner.lib.auto.AutoBuilder;
import edu.wpi.first.hal.AllianceStationID;
import edu.wpi.first.hal.HAL;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Notifier;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.simulation.DriverStationSim;
import edu.wpi.first.wpilibj.simulation.SimHooks;
import edu.wpi.first.wpilibj.simulation.XboxControllerSim;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.robot.RobotContainer;
import frc.robot.Constants;
import frc.robot.subsystems.drive.CommandSwerveDrivetrain;
import frc.robot.subsystems.localisation.LocalisationConstants;
import frc.robot.subsystems.shooter.ShooterConstants.IndexerState;
import frc.robot.subsystems.shooter.ShooterConstants.ShooterState;
import java.lang.reflect.Field;
import java.util.concurrent.ExecutorService;
import org.junit.jupiter.api.Test;
import org.junit.jupiter.params.ParameterizedTest;
import org.junit.jupiter.params.provider.EnumSource;

/** Actual Xbox right-trigger binding and native simulated motor requests; no GUI or network. */
class TeleopShotRegressionTest {
  @ParameterizedTest
  @EnumSource(value = AllianceStationID.class, names = {"Blue1", "Red1"})
  void atomicVisionShootsWithoutGlobalHeadingAndExplicitTowerRetainsManualYaw(AllianceStationID station)
      throws Exception {
    assertTrue(HAL.initialize(500, 0));
    SimHooks.resumeTiming();
    SignalLogger.enableAutoLogging(false);
    AutoBuilder.resetForTesting();
    var scheduler = CommandScheduler.getInstance();
    scheduler.cancelAll();
    scheduler.getDefaultButtonLoop().clear();
    DriverStationSim.resetData();
    DriverStationSim.setDsAttached(true);
    DriverStationSim.setFmsAttached(true);
    DriverStationSim.setAllianceStationId(station);
    DriverStationSim.setEnabled(false);
    refresh();
    NetworkTableInstance.getDefault().getTable(LocalisationConstants.kLimelights[0].name()).getEntry("botpose_wpiblue")
        .setDoubleArray(new double[11]);
    var camera = NetworkTableInstance.getDefault().getTable(LocalisationConstants.kLimelights[0].name());
    camera.getEntry("tv").setDouble(0);
    camera.getEntry("botpose_targetspace").setDoubleArray(new double[0]);
    camera.getEntry("rawfiducials").setDoubleArray(new double[0]);
    // An old dashboard value must not override the newly compiled tower range.
    SmartDashboard.putNumber("ShotTuning/ManualShotDistanceM", 2.75);
    SmartDashboard.putNumber(ShooterConstants.kPivotOffsetKey, 0);
    SmartDashboard.putNumber(ShooterConstants.kShooterRpsOffsetKey, 0);
    var controller = new XboxControllerSim(0);
    controller.setAxisCount(6);
    controller.setButtonCount(10);
    controller.setPOVCount(1);
    controller.setRightTriggerAxis(0);
    controller.setRightX(0);
    controller.setAButton(false);
    controller.setPOV(-1);
    RobotContainer container = null;
    try {
      container = new RobotContainer();
      var drive = (CommandSwerveDrivetrain) field(container, "m_drivetrain");
      ((Notifier) field(drive, "m_simNotifier")).close();
      var shooter = (Shooter) field(container, "m_shooter");
      var flywheel = (TalonFX) field(shooter, "m_leftLeaderShooterMotor");
      var pivot = (TalonFX) field(shooter, "m_leaderPivotMotor");
      var towerDrive = (SwerveRequest.FieldCentric) field(container, "driveRequest");
      var snap = (SwerveRequest.FieldCentricFacingAngle) field(drive, "snapRequest");
      snap.TargetDirection = Rotation2d.fromDegrees(20);
      DriverStationSim.setEnabled(true);
      DriverStationSim.setAutonomous(false);
      tick(scheduler, 3);
      assertFalse(drive.isFieldAimingReady());
      // Missing vision must not fall back to the tower range or force-feed after timeout.
      controller.setRightTriggerAxis(1);
      tick(scheduler, 70);
      assertTrue(shooter.isVisionShotActive());
      assertFalse(shooter.isManualShotActive());
      assertFalse(shooter.readyToShoot());
      assertEquals(IndexerState.ZERO, field(shooter, "m_indexerState"));
      assertEquals(ShooterConstants.kIdleShooterRps,
          assertInstanceOf(VelocityVoltage.class, flywheel.getAppliedControl()).Velocity, .01);
      // Blue hub tag is accepted even with FMS Red: only range is used; no field-pose target selection.
      tickVision(scheduler, camera, 3.0, 10);
      assertFalse(drive.isFieldAimingReady(), "Vision-relative aim must not require a separate global heading seed");
      verifyRangeTargets(flywheel, pivot, 3.0, 0, 0);
      var visionPreset = ManualShotPreset.create(3, 0, 0);
      for (int i = 0; i < 12; i++) {
        // Raw rotor simulation is pre-inversion; both leader motors are Clockwise_Positive.
        String[] wheelFields = {"m_leftLeaderShooterMotor", "m_backLeftFollowerShooterMotor",
            "m_rightFollowerShooterMotor", "m_backRightFollowerShooterMotor"};
        for (int wheel = 0; wheel < wheelFields.length; wheel++) {
          var motor = (TalonFX) field(shooter, wheelFields[wheel]);
          motor.getSimState().setRotorVelocity((wheel < 2 ? -1 : 1) * visionPreset.wheelRps());
          motor.getVelocity().refresh();
        }
        pivot.getSimState().setRawRotorPosition(-visionPreset.pivotRotations() * ShooterConstants.kPivotGearRatio);
        var followerPivot = (TalonFX) field(shooter, "m_followerPivotMotor");
        followerPivot.getSimState().setRawRotorPosition(visionPreset.pivotRotations() * ShooterConstants.kPivotGearRatio);
        followerPivot.getPosition().refresh();
        flywheel.getVelocity().refresh();
        pivot.getPosition().refresh();
        tickVision(scheduler, camera, 3.0, 1);
        if (shooter.readyToShoot()) break;
      }
      assertTrue(shooter.readyToShoot(), () -> "Fresh vision must permit ready: velocity="
          + flywheel.getVelocity().getValueAsDouble() + ", pivot=" + pivot.getPosition().getValueAsDouble()
          + ", range=" + SmartDashboard.getBoolean("Shooter/VisionRange/Valid", false));
      tickVision(scheduler, camera, 3.0, 1);
      assertEquals(IndexerState.SCORE, field(shooter, "m_indexerState"));
      var retryBudget = Shooter.class.getDeclaredField("m_feedRecoveryAttempts");
      retryBudget.setAccessible(true);
      retryBudget.setInt(shooter, 1);
      var rearRight = (TalonFX) field(shooter, "m_backRightFollowerShooterMotor");
      rearRight.getSimState().setRotorVelocity(0);
      rearRight.getVelocity().refresh();
      tickVision(scheduler, camera, 3.0, 4);
      assertFalse(shooter.readyToShoot(), "A failed follower wheel must stop vision feeding");
      assertEquals(IndexerState.ZERO, field(shooter, "m_indexerState"));
      assertEquals(1, retryBudget.getInt(shooter), "Readiness pauses must preserve jam retries");
      rearRight.getSimState().setRotorVelocity(visionPreset.wheelRps());
      rearRight.getVelocity().refresh();
      tickVision(scheduler, camera, 3.0, 4);
      assertTrue(shooter.readyToShoot());
      assertEquals(IndexerState.SCORE, field(shooter, "m_indexerState"));
      assertEquals(1, retryBudget.getInt(shooter));
      drive.resetPose(new Pose2d(14, 2, Rotation2d.fromDegrees(37)));
      controller.setRightX(.25);
      tickVision(scheduler, camera, 3.0, 3);
      verifyRangeTargets(flywheel, pivot, 3.0, 0, 0);
      verifyManualYaw(drive, snap, (SwerveRequest.FieldCentric) field(drive, "m_unlocalizedDrive"));
      publishVision(camera, 3.1);
      tick(scheduler, 2);
      verifyRangeTargets(flywheel, pivot, 3.045, 0, 0); // Filter one distinct camera frame, not each robot loop.
      tick(scheduler, 18); // No fresh frame: stale data must stop feed, never select tower.
      assertFalse(shooter.readyToShoot());
      assertFalse(shooter.isManualShotActive());
      assertEquals(IndexerState.ZERO, field(shooter, "m_indexerState"));
      controller.setRightTriggerAxis(0);
      controller.setRightX(0);
      tick(scheduler, 3);
      assertFalse(shooter.isVisionShotActive());
      flywheel.getSimState().setRotorVelocity(0);
      pivot.getSimState().setRawRotorPosition(0);
      tick(scheduler, 3);
      flywheel.getVelocity().refresh();
      pivot.getPosition().refresh();
      // Explicit fallback: A must be held before pulling the trigger.
      controller.setAButton(true);
      controller.setRightTriggerAxis(1);
      tick(scheduler, 6);
      assertTrue(shooter.isManualShotActive(), "Uninitialized pose must not swallow trigger press");
      verifyTowerTargets(flywheel, pivot, 0, 0);
      assertEquals(IndexerState.ZERO, field(shooter, "m_indexerState"), "Do not feed before spin-up");
      assertEquals("TOWER PRESET - aim with right stick", SmartDashboard.getString("Shooter/ControlMode", ""));
      assertEquals("VISION AUTO-AIM / A+TRIGGER MANUAL TOWER", SmartDashboard.getString("aim/auto mode", ""));
      // Acquiring position/heading and editing the dashboard while held must NOT
      // switch to auto-aim or change the tower shot's already-latched preset/trims.
      drive.resetPose(new Pose2d(14, 2, Rotation2d.k180deg));
      SmartDashboard.putNumber("ShotTuning/ManualShotDistanceM", 4.5);
      SmartDashboard.putNumber(ShooterConstants.kPivotOffsetKey, .002);
      SmartDashboard.putNumber(ShooterConstants.kShooterRpsOffsetKey, 1);
      tick(scheduler, 61);
      assertTrue(shooter.isManualShotActive());
      verifyTowerTargets(flywheel, pivot, 0, 0);
      assertEquals(IndexerState.SCORE, field(shooter, "m_indexerState"),
          "Manual feed must proceed after the existing bounded spin-up wait");
      controller.setRightTriggerAxis(0);
      tick(scheduler, 3);
      assertFalse(shooter.isManualShotActive());
      assertEquals(ShooterState.ZERO, field(shooter, "m_shooterState"));
      assertEquals(IndexerState.ZERO, field(shooter, "m_indexerState"));
      assertEquals("IDLE", SmartDashboard.getString("Shooter/ControlMode", ""));
      // Deliberately out of hub range and outside the alliance zone. Even a fully
      // initialized pose must not change right trigger into a pass or automatic shot.
      drive.resetPose(new Pose2d(8.27, .25, Rotation2d.k180deg));
      controller.setRightTriggerAxis(1);
      controller.setRightX(.25);
      tick(scheduler, 6);
      assertTrue(drive.isFieldAimingReady());
      assertTrue(drive.getShotDistance() > ShooterConstants.kMaxShotDistanceMeters);
      assertTrue(shooter.isManualShotActive(), "Localized presses must still use the fixed tower preset");
      verifyTowerTargets(flywheel, pivot, .002, 1);
      assertEquals("TOWER PRESET - aim with right stick", SmartDashboard.getString("Shooter/ControlMode", ""));
      verifyManualYaw(drive, snap, towerDrive);
      // Mode transition must cancel the teleop shot, including with trigger held.
      DriverStationSim.setAutonomous(true);
      tick(scheduler, 3);
      assertFalse(shooter.isManualShotActive());
      assertEquals(IndexerState.ZERO, field(shooter, "m_indexerState"));
      assertTrue(drive.isAutoAlignmentEnabled(), "Autonomous heading control remains enabled");
      controller.setRightTriggerAxis(0);
      DriverStationSim.setAutonomous(false);
      tick(scheduler, 3);
      controller.setAButton(true);
      tick(scheduler, 4);
      verifyManualYaw(drive, snap, (SwerveRequest.FieldCentric) field(drive, "m_unlocalizedDrive"));
      controller.setAButton(false);
      tick(scheduler, 3);
      controller.setPOV(0);
      tick(scheduler, 4);
      verifyManualYaw(drive, snap, (SwerveRequest.FieldCentric) field(drive, "m_unlocalizedDrive"));
      controller.setPOV(-1);
      tick(scheduler, 3);
      controller.setPOV(90);
      tick(scheduler, 6);
      assertFalse(shooter.isManualShotActive(), "Separate manual ferry must clear the tower preset");
      assertEquals(ShooterConstants.kSendShooterRps,
          assertInstanceOf(VelocityVoltage.class, flywheel.getAppliedControl()).Velocity, .01);
      assertEquals(ShooterConstants.kLobPivotPosition * ShooterConstants.kPivotGearRatio,
          assertInstanceOf(MotionMagicVoltage.class, pivot.getAppliedControl()).Position, .0001);
      controller.setPOV(-1);
      tick(scheduler, 3);
      assertEquals(ShooterState.ZERO, field(shooter, "m_shooterState"));
      assertEquals(IndexerState.ZERO, field(shooter, "m_indexerState"));
    } finally {
      controller.setRightTriggerAxis(0);
      controller.setRightX(0);
      controller.setAButton(false);
      controller.setPOV(-1);
      scheduler.cancelAll();
      scheduler.getDefaultButtonLoop().clear();
      if (container != null) {
        var power = field(container, "m_powerManager");
        ((ExecutorService) field(power, "m_executor")).shutdownNow();
        for (Field declared : RobotContainer.class.getDeclaredFields()) {
          declared.setAccessible(true);
          Object object = declared.get(container);
          if (object instanceof Subsystem subsystem) scheduler.unregisterSubsystem(subsystem);
          if (object instanceof CommandSwerveDrivetrain drive) drive.close();
          if (object instanceof Shooter || object instanceof frc.robot.subsystems.lintake.Lintake) {
            for (Field deviceField : object.getClass().getDeclaredFields()) {
              deviceField.setAccessible(true);
              Object device = deviceField.get(object);
              if (device instanceof ParentDevice && device instanceof AutoCloseable closeable) closeable.close();
            }
          }
        }
      }
      AutoBuilder.resetForTesting();
      camera.getEntry("tv").setDouble(0);
      SmartDashboard.putNumber("ShotTuning/ManualShotDistanceM", 2.75);
      SmartDashboard.putNumber(ShooterConstants.kPivotOffsetKey, 0);
      SmartDashboard.putNumber(ShooterConstants.kShooterRpsOffsetKey, 0);
      DriverStationSim.resetData();
      refresh();
    }
  }

  @Test void manualPresetUsesExistingTableAndRejectsNonfiniteOrExcessiveInputs() {
    var preset = ManualShotPreset.create(2.75, .002, 1);
    assertEquals(ShooterConstants.getScorePivotPosition(2.75) + .002, preset.pivotRotations(), 1e-9);
    assertEquals(ShooterConstants.getScoreShooterRps(2.75) + 1, preset.wheelRps(), 1e-9);
    for (double distance : new double[] {Double.NaN, Double.NEGATIVE_INFINITY, -100, 100}) {
      var safe = ManualShotPreset.create(distance, Double.NaN, Double.NaN);
      assertTrue(safe.distanceMeters() >= 1.5 && safe.distanceMeters() <= 5);
      assertTrue(Double.isFinite(safe.pivotRotations()));
      assertTrue(Double.isFinite(safe.wheelRps()));
    }
    var excessive = ManualShotPreset.create(5, 99, 9999);
    assertEquals(.1, excessive.pivotRotations(), 1e-9);
    assertEquals(ShooterConstants.kSendShooterRps, excessive.wheelRps(), 1e-9);
  }

  @Test void towerPresetUsesUprightToHubCenterDistanceAndExistingTable() {
    double drawingDistance = (115.05 + 47.0 / 2) * .0254;
    assertEquals(drawingDistance, ShooterConstants.kTowerShotDistanceMeters, .001);
    var preset = ManualShotPreset.tower(0, 0);
    assertEquals(3.52, preset.distanceMeters(), 1e-9);
    assertEquals(.07614, preset.pivotRotations(), 1e-9);
    assertEquals(46.55, preset.wheelRps(), 1e-9);
    var trimmed = ManualShotPreset.tower(.002, 1);
    assertEquals(.07814, trimmed.pivotRotations(), 1e-9);
    assertEquals(47.55, trimmed.wheelRps(), 1e-9);
  }

  private static void verifyTowerTargets(TalonFX flywheel, TalonFX pivot,
      double pivotTrim, double rpsTrim) {
    verifyRangeTargets(flywheel, pivot, ShooterConstants.kTowerShotDistanceMeters, pivotTrim, rpsTrim);
  }

  private static void verifyRangeTargets(TalonFX flywheel, TalonFX pivot, double distance,
      double pivotTrim, double rpsTrim) {
    var expected = ManualShotPreset.create(distance, pivotTrim, rpsTrim);
    assertEquals(expected.wheelRps(),
        assertInstanceOf(VelocityVoltage.class, flywheel.getAppliedControl()).Velocity, .01);
    assertEquals(expected.pivotRotations() * ShooterConstants.kPivotGearRatio,
        assertInstanceOf(MotionMagicVoltage.class, pivot.getAppliedControl()).Position, .0001);
  }

  private static void publishVision(edu.wpi.first.networktables.NetworkTable camera, double distance) {
    // Small image-area noise; geometry is constant. Native pipeline tests separately
    // cover bit-identical packets with real keepDuplicates publishers.
    double marker = Timer.getFPGATimestamp() * 1e-8;
    double trim = SmartDashboard.getNumber("ShotTuning/HeadingOffsetDeg", 6.7);
    camera.getEntry("botpose_wpiblue").setDoubleArray(new double[] {
        ShooterConstants.kBlueHubPosition.getX() - distance, ShooterConstants.kBlueHubPosition.getY(),
        0, 0, 0, trim, 15, 1, 0, 2, .5 + marker,
        31, 0, 0, .5 + marker, 2, 2, .05}); // A tower tag, not a hub tag.
  }

  private static void tickVision(CommandScheduler scheduler,
      edu.wpi.first.networktables.NetworkTable camera, double distance, int frames) {
    for (int i = 0; i < frames; i++) {
      publishVision(camera, distance);
      tick(scheduler, 1);
    }
  }

  private static Object field(Object object, String name) throws Exception {
    Field field = object.getClass().getDeclaredField(name);
    field.setAccessible(true);
    return field.get(object);
  }

  private static void verifyManualYaw(CommandSwerveDrivetrain drive,
      SwerveRequest.FieldCentricFacingAngle snap, SwerveRequest.FieldCentric request) {
    assertTrue(Constants.kTeleopAutoAlignEnabled);
    assertTrue(drive.isAutoAlignmentEnabled());
    assertEquals(-.25 * Constants.kMaxAngularRate, request.RotationalRate, 1e-9,
        "A, right trigger, and POV-up must retain the driver's requested yaw");
    assertEquals(20, snap.TargetDirection.getDegrees(), 1e-9,
        "Driver override must not update a field-heading-lock target");
  }

  private static void refresh() {
    DriverStationSim.notifyNewData();
    DriverStation.refreshData();
  }

  private static void tick(CommandScheduler scheduler, int frames) {
    for (int i = 0; i < frames; i++) {
      refresh();
      scheduler.run();
      Timer.delay(.02);
    }
  }
}
