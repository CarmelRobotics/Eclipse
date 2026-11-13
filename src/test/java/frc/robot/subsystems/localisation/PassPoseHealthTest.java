package frc.robot.subsystems.localisation;

import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertTrue;

import com.ctre.phoenix6.SignalLogger;
import com.pathplanner.lib.auto.AutoBuilder;
import edu.wpi.first.hal.AllianceStationID;
import edu.wpi.first.hal.HAL;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Notifier;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.simulation.DriverStationSim;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.subsystems.drive.CommandSwerveDrivetrain;
import frc.robot.subsystems.drive.TunerConstants;
import org.junit.jupiter.api.Test;

class PassPoseHealthTest {
  @Test void staleVisionOrOffFieldPoseStopsCornerAiming() throws Exception {
    assertTrue(HAL.initialize(500, 0));
    SignalLogger.enableAutoLogging(false);
    AutoBuilder.resetForTesting();
    DriverStationSim.resetData();
    DriverStationSim.setDsAttached(true);
    DriverStationSim.setAllianceStationId(AllianceStationID.Blue1);
    DriverStationSim.notifyNewData();
    DriverStation.refreshData();
    SmartDashboard.putBoolean("FeatureFlags/VisionFusion", true);
    try (var drive = TunerConstants.createDrivetrain()) {
      var notifier = CommandSwerveDrivetrain.class.getDeclaredField("m_simNotifier");
      notifier.setAccessible(true);
      ((Notifier) notifier.get(drive)).close();
      drive.isRedAlliance();
      drive.resetPose(new Pose2d(3, 2, Rotation2d.kZero));
      var lastVision = CommandSwerveDrivetrain.class.getDeclaredField("m_lastPassVisionTime");
      lastVision.setAccessible(true);
      assertFalse(drive.isPassPoseTrustworthy(), "A pose reset alone cannot validate a pass");
      lastVision.setDouble(drive, Timer.getFPGATimestamp());
      assertTrue(drive.isPassPoseTrustworthy());
      lastVision.setDouble(drive, Timer.getFPGATimestamp() - 3);
      assertFalse(drive.isPassPoseTrustworthy(), "Stale vision selects manual passing");
      lastVision.setDouble(drive, Timer.getFPGATimestamp());
      drive.resetPose(new Pose2d(17, 2, Rotation2d.kZero));
      assertFalse(drive.isPassPoseTrustworthy(), "Off-field coordinates cannot aim a corner pass");
    } finally {
      AutoBuilder.resetForTesting();
      DriverStationSim.resetData();
    }
  }
}
