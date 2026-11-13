package frc.robot.subsystems.shooter;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertTrue;

import com.ctre.phoenix6.SignalLogger;
import com.ctre.phoenix6.hardware.ParentDevice;
import com.pathplanner.lib.auto.AutoBuilder;
import edu.wpi.first.hal.HAL;
import edu.wpi.first.wpilibj.Timer;
import frc.robot.subsystems.drive.TunerConstants;
import frc.robot.subsystems.shooter.ShooterConstants.IndexerState;
import frc.robot.subsystems.shooter.ShooterConstants.PivotState;
import frc.robot.subsystems.shooter.ShooterConstants.ShooterState;
import java.lang.reflect.Field;
import org.junit.jupiter.api.Test;

class FeedFallbackTest {
  @Test void missedFlywheelDipsWarnWithoutStoppingAndElectricalFaultDoesStop() throws Exception {
    assertTrue(HAL.initialize(500, 0));
    SignalLogger.enableAutoLogging(false);
    AutoBuilder.resetForTesting();
    try (var drive = TunerConstants.createDrivetrain()) {
      var shooter = new Shooter(drive);
      try {
        shooter.setState(IndexerState.ZERO, PivotState.SCORE, ShooterState.SCORE);
        var run = Shooter.class.getDeclaredMethod("runFeedWithRecovery");
        run.setAccessible(true);
        run.invoke(shooter);
        Field start = Shooter.class.getDeclaredField("m_feedRecoveryPhaseStart");
        start.setAccessible(true);
        Field attempts = Shooter.class.getDeclaredField("m_feedRecoveryAttempts");
        attempts.setAccessible(true);
        attempts.setInt(shooter, 50);
        start.setDouble(shooter, Timer.getFPGATimestamp() - 2);
        run.invoke(shooter);
        assertEquals(IndexerState.SCORE, field(shooter, "m_indexerState"));
        assertEquals(51, attempts.getInt(shooter));
        Field electricalFault = Shooter.class.getDeclaredField("m_indexerOvercurrentTripped");
        electricalFault.setAccessible(true);
        electricalFault.setBoolean(shooter, true);
        run.invoke(shooter);
        assertEquals(IndexerState.ZERO, field(shooter, "m_indexerState"));
        var stop = Shooter.class.getDeclaredMethod("stopFeedWithRecovery");
        stop.setAccessible(true);
        stop.invoke(shooter);
        assertEquals(false, electricalFault.getBoolean(shooter));
      } finally {
        for (Field deviceField : Shooter.class.getDeclaredFields()) {
          deviceField.setAccessible(true);
          Object device = deviceField.get(shooter);
          if (device instanceof ParentDevice && device instanceof AutoCloseable closeable)
            closeable.close();
        }
      }
    } finally {
      AutoBuilder.resetForTesting();
    }
  }

  private static Object field(Object owner, String name) throws Exception {
    Field value = owner.getClass().getDeclaredField(name);
    value.setAccessible(true);
    return value.get(owner);
  }
}
