package frc.robot.subsystems.shooter;

import static org.junit.jupiter.api.Assertions.*;

import com.ctre.phoenix6.SignalLogger;
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

/** Actual Xbox right-trigger binding and native simulated motor requests; no GUI or network. */
class TeleopShotRegressionTest {
  @Test void rightTriggerStillShootsAndAllTeleopAimButtonsPreserveManualYaw()
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
    DriverStationSim.setAllianceStationId(AllianceStationID.Red1);
    DriverStationSim.setEnabled(false);
    refresh();
    NetworkTableInstance.getDefault().getTable(LocalisationConstants.kLimelights[0].name()).getEntry("botpose_wpiblue")
        .setDoubleArray(new double[11]);
    SmartDashboard.putNumber(ManualShotPreset.DISTANCE_KEY, 2.75);
    SmartDashboard.putNumber(ShooterConstants.kPivotOffsetKey, 0);
    SmartDashboard.putNumber(ShooterConstants.kShooterRpsOffsetKey, 0);
    var controller = new XboxControllerSim(0);
    controller.setAxisCount(6);
    controller.setButtonCount(10);
    controller.setPOVCount(1);
    controller.setRightTriggerAxis(0);
    controller.setPOV(-1);
    RobotContainer container = null;
    try {
      container = new RobotContainer();
      var drive = (CommandSwerveDrivetrain) field(container, "m_drivetrain");
      ((Notifier) field(drive, "m_simNotifier")).close();
      var shooter = (Shooter) field(container, "m_shooter");
      var flywheel = (TalonFX) field(shooter, "m_leftLeaderShooterMotor");
      DriverStationSim.setEnabled(true);
      DriverStationSim.setAutonomous(false);
      tick(scheduler, 3);
      assertFalse(drive.isFieldAimingReady());
      controller.setRightTriggerAxis(1);
      tick(scheduler, 6);
      assertTrue(shooter.isManualShotActive(), "Uninitialized pose must not swallow trigger press");
      var request = assertInstanceOf(VelocityVoltage.class, flywheel.getAppliedControl());
      assertEquals(ShooterConstants.getScoreShooterRps(2.75), request.Velocity, .01);
      assertEquals(IndexerState.ZERO, field(shooter, "m_indexerState"), "Do not feed before spin-up");
      assertTrue(SmartDashboard.getString("Shooter/ControlMode", "").startsWith("MANUAL"));
      // Acquiring position/heading and editing the dashboard while held must NOT
      // switch to auto-aim or change this manual shot's already-latched preset.
      drive.resetPose(new Pose2d(14, 2, Rotation2d.k180deg));
      SmartDashboard.putNumber(ManualShotPreset.DISTANCE_KEY, 4.5);
      tick(scheduler, 61);
      assertTrue(shooter.isManualShotActive());
      assertEquals(ShooterConstants.getScoreShooterRps(2.75),
          assertInstanceOf(VelocityVoltage.class, flywheel.getAppliedControl()).Velocity, .01);
      assertEquals(IndexerState.SCORE, field(shooter, "m_indexerState"),
          "Manual feed must proceed after the existing bounded spin-up wait");
      controller.setRightTriggerAxis(0);
      tick(scheduler, 3);
      assertFalse(shooter.isManualShotActive());
      assertEquals(ShooterState.ZERO, field(shooter, "m_shooterState"));
      assertEquals(IndexerState.ZERO, field(shooter, "m_indexerState"));
      controller.setRightTriggerAxis(1);
      controller.setRightX(.25);
      var snap = (SwerveRequest.FieldCentricFacingAngle) field(drive, "snapRequest");
      snap.TargetDirection = Rotation2d.fromDegrees(20);
      tick(scheduler, 6);
      assertTrue(drive.isFieldAimingReady());
      assertFalse(shooter.isManualShotActive(), "Next press can select pose-based range without auto-align");
      assertEquals("MANUAL AIM / AUTO RANGE", SmartDashboard.getString("Shooter/ControlMode", ""));
      verifyManualYaw(drive, snap);
      controller.setRightTriggerAxis(0);
      tick(scheduler, 3);
      controller.setAButton(true);
      tick(scheduler, 4);
      verifyManualYaw(drive, snap);
      controller.setAButton(false);
      tick(scheduler, 3);
      controller.setPOV(0);
      tick(scheduler, 4);
      verifyManualYaw(drive, snap);
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
      SmartDashboard.putNumber(ManualShotPreset.DISTANCE_KEY, ManualShotPreset.DEFAULT_DISTANCE_METERS);
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

  private static Object field(Object object, String name) throws Exception {
    Field field = object.getClass().getDeclaredField(name);
    field.setAccessible(true);
    return field.get(object);
  }

  private static void verifyManualYaw(CommandSwerveDrivetrain drive,
      SwerveRequest.FieldCentricFacingAngle snap) throws Exception {
    assertFalse(Constants.kTeleopAutoAlignEnabled);
    assertFalse(drive.isAutoAlignmentEnabled());
    var request = (SwerveRequest.FieldCentric) field(drive, "m_unlocalizedDrive");
    assertEquals(-.25 * Constants.kMaxAngularRate, request.RotationalRate, 1e-9,
        "A, right trigger, and POV-up must retain the driver's requested yaw");
    assertEquals(20, snap.TargetDirection.getDegrees(), 1e-9,
        "Teleop must not update a heading-lock target, even when fully localized");
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
