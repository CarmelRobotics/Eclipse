package frc.robot.subsystems.localisation;

import static org.junit.jupiter.api.Assertions.*;

import com.ctre.phoenix6.SignalLogger;
import com.pathplanner.lib.auto.AutoBuilder;
import com.ctre.phoenix6.swerve.SwerveRequest;
import edu.wpi.first.hal.AllianceStationID;
import edu.wpi.first.hal.HAL;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Notifier;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.simulation.DriverStationSim;
import edu.wpi.first.wpilibj.simulation.SimHooks;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.subsystems.drive.CommandSwerveDrivetrain;
import frc.robot.subsystems.drive.TunerConstants;
import org.junit.jupiter.api.Test;

/** Production subsystem + real Phoenix simulator + local synthetic NT camera, no GUI/network. */
class TeleopLocalizationIntegrationTest {
  @Test void redFmsWithoutAutoRejectsMirrorInitializesFromRedAndKeepsOdometryOnTagLoss()
      throws Exception {
    assertTrue(HAL.initialize(500, 0));
    SimHooks.resumeTiming();
    SignalLogger.enableAutoLogging(false);
    AutoBuilder.resetForTesting();
    DriverStationSim.resetData();
    DriverStationSim.setDsAttached(true);
    DriverStationSim.setFmsAttached(true);
    DriverStationSim.setAllianceStationId(AllianceStationID.Red1);
    DriverStationSim.setEnabled(false);
    DriverStationSim.notifyNewData();
    DriverStation.refreshData();
    SmartDashboard.putBoolean("FeatureFlags/VisionFusion", true);
    var camera = NetworkTableInstance.getDefault().getTable(LocalisationConstants.kLimelights[0].name());
    camera.getEntry("botpose_wpiblue").setDoubleArray(new double[11]);
    try (var drive = TunerConstants.createDrivetrain()) {
      var sim = CommandSwerveDrivetrain.class.getDeclaredField("m_simNotifier");
      sim.setAccessible(true);
      ((Notifier) sim.get(drive)).close();
      Timer.delay(.15);
      drive.prepareForMatch();
      assertEquals(11.925, drive.getHubPosition().getX(), 1e-9);
      for (int i = 0; i < 7; i++) publish(drive, 2.541 + i * .0001, 6.069, 2);
      assertFalse(drive.isFieldAimingReady());
      assertTrue(drive.getState().Pose.getX() < 1, "Mirrored camera must not initialize pose");
      for (int i = 0; i < 7; i++) publish(drive, 14 + i * .0001, 2, 2);
      assertEquals(14, drive.getState().Pose.getX(), .03, "Real red XY acquired without auto");
      assertFalse(drive.isFieldAimingReady(), "Vision XY must not invent gyro alignment");
      drive.seedDownfieldHeading();
      Timer.delay(.04);
      drive.periodic();
      assertTrue(drive.isFieldAimingReady());
      assertEquals(180, Math.abs(drive.getState().Pose.getRotation().getDegrees()), .2);
      verifyAimRequest(drive);
      DriverStationSim.setEnabled(true);
      DriverStationSim.setAutonomous(false);
      DriverStationSim.notifyNewData();
      DriverStation.refreshData();
      for (int i = 0; i < 7; i++) publish(drive, 2.541 + i * .0001, 6.069, 2);
      assertEquals(14, drive.getState().Pose.getX(), .05, "No enabled cross-field teleport");
      publish(drive, 0, 0, 0);
      assertTrue(drive.isFieldAimingReady(), "Tag loss keeps referenced odometry available");
      assertEquals(14, drive.getState().Pose.getX(), .05);
      DriverStationSim.setFmsAttached(false);
      DriverStationSim.setAllianceStationId(AllianceStationID.Unknown);
      DriverStationSim.notifyNewData();
      DriverStation.refreshData();
      drive.periodic();
      assertEquals(11.925, drive.getHubPosition().getX(), 1e-9,
          "FMS/alliance packet loss must not change red target to blue");
      // Publish the decimated diagnostic deterministically, not by test loop count.
      for (int i = 0; i < 5; i++) drive.periodic();
      assertEquals("Red", SmartDashboard.getString("Localization/Alliance", ""));
      DriverStationSim.setEnabled(false);
      DriverStationSim.setAllianceStationId(AllianceStationID.Blue1);
      DriverStationSim.notifyNewData();
      DriverStation.refreshData();
      drive.periodic();
      drive.resetPose(new Pose2d(2, 2, Rotation2d.kZero));
      drive.periodic();
      assertEquals(4.625, drive.getHubPosition().getX(), 1e-9);
      assertTrue(drive.isFieldAimingReady(), "Explicit auto-style reset initializes both axes");
      verifyAimRequest(drive);
      CommandScheduler.getInstance().unregisterSubsystem(drive);
    } finally {
      CommandScheduler.getInstance().cancelAll();
      AutoBuilder.resetForTesting();
      camera.getEntry("botpose_wpiblue").setDoubleArray(new double[11]);
      DriverStationSim.resetData();
      DriverStationSim.notifyNewData();
      DriverStation.refreshData();
    }
  }

  private static void publish(CommandSwerveDrivetrain drive, double x, double y, int tags) {
    NetworkTableInstance.getDefault().getTable(LocalisationConstants.kLimelights[0].name()).getEntry("botpose_wpiblue")
        .setDoubleArray(new double[] {x, y, 0, 0, 0, 0, 20, tags, 1, 2, .5});
    Timer.delay(.045);
    drive.periodic();
  }

  private static void verifyAimRequest(CommandSwerveDrivetrain drive) throws Exception {
    boolean wasEnabled = DriverStation.isEnabled();
    boolean wasAuto = DriverStation.isAutonomous();
    DriverStationSim.setAutonomous(true);
    DriverStationSim.setEnabled(true);
    DriverStationSim.notifyNewData();
    DriverStation.refreshData();
    drive.periodic();
    assertTrue(drive.isAutoAlignmentEnabled(), "Autonomous heading control must remain enabled");
    var command = drive.faceHubCommand(() -> 0.0, () -> 0.0);
    command.initialize();
    command.execute();
    var requestField = CommandSwerveDrivetrain.class.getDeclaredField("snapRequest");
    requestField.setAccessible(true);
    var request = (SwerveRequest.FieldCentricFacingAngle) requestField.get(drive);
    var appliedFieldHeading = request.TargetDirection.plus(drive.getOperatorForwardDirection());
    assertEquals(0, MathUtil.angleModulus(appliedFieldHeading
        .minus(drive.getHubHeading()).getRadians()), 1e-9,
        "The request must cancel the custom driver perspective exactly once");
    double rawBearing = drive.getHubPosition().minus(drive.getState().Pose.getTranslation())
        .getAngle().getRadians();
    double trim = Math.toRadians(SmartDashboard.getNumber("ShotTuning/HeadingOffsetDeg", 6.7));
    assertEquals(0, MathUtil.angleModulus(drive.getHubHeading().getRadians() - rawBearing - trim), .01,
        "Stationary shot bearing must point at our own alliance hub");
    command.end(true);
    DriverStationSim.setEnabled(wasEnabled);
    DriverStationSim.setAutonomous(wasAuto);
    DriverStationSim.notifyNewData();
    DriverStation.refreshData();
  }
}
