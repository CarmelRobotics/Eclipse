package frc.robot.subsystems.localisation;

import static org.junit.jupiter.api.Assertions.*;
import com.ctre.phoenix6.SignalLogger;
import com.ctre.phoenix6.swerve.SwerveRequest;
import com.pathplanner.lib.auto.AutoBuilder;
import edu.wpi.first.hal.AllianceStationID;
import edu.wpi.first.hal.HAL;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.PubSubOption;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Notifier;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.simulation.DriverStationSim;
import edu.wpi.first.wpilibj.simulation.SimHooks;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.subsystems.drive.CommandSwerveDrivetrain;
import frc.robot.subsystems.drive.TunerConstants;
import frc.robot.subsystems.shooter.ShooterConstants;
import org.junit.jupiter.params.ParameterizedTest;
import org.junit.jupiter.params.provider.EnumSource;

class AutoAimDriveIntegrationTest {
  @ParameterizedTest
  @EnumSource(value = AllianceStationID.class, names = {"Blue1", "Red1"})
  void realRequestRotatesCorrectWayOnBothAlliancesAndDriverCanOverride(AllianceStationID station)
      throws Exception {
    assertTrue(HAL.initialize(500, 0));
    SimHooks.resumeTiming();
    SignalLogger.enableAutoLogging(false);
    AutoBuilder.resetForTesting();
    DriverStationSim.resetData();
    DriverStationSim.setDsAttached(true);
    DriverStationSim.setAllianceStationId(station);
    DriverStationSim.setEnabled(true);
    DriverStationSim.setAutonomous(false);
    DriverStationSim.notifyNewData();
    DriverStation.refreshData();
    SmartDashboard.putNumber("ShotTuning/HeadingOffsetDeg", 0);
    boolean red = station == AllianceStationID.Red1;
    var hub = red ? ShooterConstants.kRedHubPosition : ShooterConstants.kBlueHubPosition;
    var camera = NetworkTableInstance.getDefault().getTable(LocalisationConstants.kLimelights[0].name());
    try (var publisher = camera.getDoubleArrayTopic("botpose_wpiblue")
            .publish(PubSubOption.keepDuplicates(true));
        var drive = TunerConstants.createDrivetrain()) {
      var notifier = CommandSwerveDrivetrain.class.getDeclaredField("m_simNotifier");
      notifier.setAccessible(true);
      ((Notifier) notifier.get(drive)).close();
      double[] manualRotation = {0};
      var command = drive.faceHubCommand(() -> 0.0, () -> 0.0, () -> manualRotation[0]);
      command.initialize();
      var requestField = CommandSwerveDrivetrain.class.getDeclaredField("m_unlocalizedDrive");
      requestField.setAccessible(true);
      var request = (SwerveRequest.FieldCentric) requestField.get(drive);
      for (int i = 0; i < 15; i++) {
        publisher.set(new double[] {hub.getX() + (red ? 3 : -3), hub.getY(), 0, 0, 0,
            (red ? 180 : 0) - 15, 10, 1, 0, 2, .5,
            red ? 16 : 31, 0, 0, .5, 2, 2, .05});
        drive.periodic();
        command.execute();
        Timer.delay(.02);
      }
      assertTrue(drive.getVisionAim().valid(), drive.getVisionAimStatus());
      assertFalse(drive.isFieldAimingReady(), "Turn control must not require a seeded global heading");
      assertEquals(15, drive.getVisionAimErrorDegrees(), .01);
      assertTrue(request.RotationalRate > 0, "Positive CCW correction for target on robot's left");
      assertTrue(request.RotationalRate <= ShooterConstants.kVisionAimMaxOmega);
      assertEquals("Aligning chassis to hub", drive.getVisionFeedBlockReason());
      manualRotation[0] = -.9;
      command.execute();
      assertEquals(-.9, request.RotationalRate, 1e-9);
      assertEquals("Driver rotation override", drive.getVisionFeedBlockReason());
      manualRotation[0] = 0;
      // No new packet, even though the camera heartbeat continues to increment.
      for (int i = 0; i < 20; i++) {
        camera.getEntry("hb").setDouble(i);
        drive.periodic();
        command.execute();
        Timer.delay(.02);
      }
      assertFalse(drive.getVisionAim().valid());
      assertEquals(0, request.RotationalRate, 1e-9);
      command.end(true);
      publisher.set(new double[11]);
      CommandScheduler.getInstance().unregisterSubsystem(drive);
    } finally {
      AutoBuilder.resetForTesting();
      SmartDashboard.putNumber("ShotTuning/HeadingOffsetDeg", 6.7);
      DriverStationSim.resetData();
      DriverStationSim.notifyNewData();
      DriverStation.refreshData();
    }
  }
}
