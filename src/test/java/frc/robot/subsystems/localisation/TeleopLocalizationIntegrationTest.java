package frc.robot.subsystems.localisation;

import static org.junit.jupiter.api.Assertions.*;

import com.ctre.phoenix6.SignalLogger;
import com.pathplanner.lib.auto.AutoBuilder;
import com.ctre.phoenix6.swerve.SwerveRequest;
import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
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
import frc.robot.subsystems.shooter.ShooterConstants;
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
      assertEquals(ShooterConstants.kRedHubPosition.getX(), drive.getHubPosition().getX(), 1e-9);
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
      // A first validated multi-tag frame should immediately pull a pose that
      // wandered toward the stands back to the field.
      drive.resetPose(new Pose2d(15.5, 7.9, Rotation2d.k180deg));
      publish(drive, 14, 2, 2, 180);
      Timer.delay(.06);
      drive.periodic();
      assertTrue(drive.getState().Pose.getY() < 3,
          "First valid multi-tag frame should correct drift immediately: y="
              + drive.getState().Pose.getY() + ", status="
              + SmartDashboard.getString("Localization/Vision status", "unknown"));
      for (int i = 1; i < 7; i++) publish(drive, 14 + i * .0001, 2, 2, 180);
      assertTrue(drive.getState().Pose.getY() < 3,
          "Confirmed vision must recover large enabled odometry drift");
      double correctedX = drive.getState().Pose.getX();
      for (int i = 0; i < 7; i++) publish(drive, 2.541 + i * .0001, 6.069, 2);
      assertEquals(correctedX, drive.getState().Pose.getX(), .05, "No enabled cross-field teleport");
      publish(drive, 0, 0, 0);
      assertTrue(drive.isFieldAimingReady(), "Tag loss keeps referenced odometry available");
      assertEquals(correctedX, drive.getState().Pose.getX(), .05);
      drive.resetPose(new Pose2d(15.5, 7.9, Rotation2d.k180deg));
      publishSingleRedTag(drive, 14);
      Timer.delay(.06);
      drive.periodic();
      assertTrue(drive.getState().Pose.getY() < 3,
          "First valid single-tag frame should correct drift immediately: y="
              + drive.getState().Pose.getY() + ", status="
              + SmartDashboard.getString("Localization/Vision status", "unknown") + ", input="
              + SmartDashboard.getString("Vision/InputStatus", "unknown"));
      for (int i = 1; i < 7; i++) publishSingleRedTag(drive, 14 + i * .0001);
      assertTrue(drive.getState().Pose.getY() < 3,
          "A verified single tag must also recover large enabled XY drift: y="
              + drive.getState().Pose.getY() + ", vision="
              + SmartDashboard.getString("Localization/Vision status", "unknown") + ", input="
              + SmartDashboard.getString("Vision/InputStatus", "unknown"));
      DriverStationSim.setFmsAttached(false);
      DriverStationSim.setAllianceStationId(AllianceStationID.Unknown);
      DriverStationSim.notifyNewData();
      DriverStation.refreshData();
      drive.periodic();
      assertEquals(ShooterConstants.kRedHubPosition.getX(), drive.getHubPosition().getX(), 1e-9,
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
      assertEquals(ShooterConstants.kBlueHubPosition.getX(), drive.getHubPosition().getX(), 1e-9);
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
    publish(drive, x, y, tags, 0);
  }

  private static void publish(CommandSwerveDrivetrain drive, double x, double y, int tags,
      double headingDegrees) {
    NetworkTableInstance.getDefault().getTable(LocalisationConstants.kLimelights[0].name()).getEntry("botpose_wpiblue")
        .setDoubleArray(new double[] {x, y, 0, 0, 0, headingDegrees, 20, tags, 1, 2, .5});
    Timer.delay(.045);
    drive.periodic();
  }

  private static void publishSingleRedTag(CommandSwerveDrivetrain drive, double x) {
    var layout = AprilTagFieldLayout.loadField(AprilTagFields.k2026RebuiltWelded);
    int nearestId = 0;
    double nearestRange = Double.POSITIVE_INFINITY;
    for (int id = 1; id <= 32; id++) {
      var tag = layout.getTagPose(id);
      if (tag.isEmpty()) continue;
      double range = Math.sqrt(Math.pow(tag.get().getX() - x, 2)
          + Math.pow(tag.get().getY() - 2, 2) + Math.pow(tag.get().getZ(), 2));
      if (range < nearestRange) {
        nearestRange = range;
        nearestId = id;
      }
    }
    assertTrue(nearestRange < 3.0);
    NetworkTableInstance.getDefault().getTable(LocalisationConstants.kLimelights[0].name())
        .getEntry("botpose_wpiblue").setDoubleArray(new double[] {
            x, 2, 0, 0, 0, 180, 20, 1, 0, nearestRange, .5,
            nearestId, 0, 0, .5, nearestRange, nearestRange, .08});
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
    // Run the command's beforeStarting sequence as the scheduler would, then the request.
    for (int i = 0; i < 3; i++) command.execute();
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
