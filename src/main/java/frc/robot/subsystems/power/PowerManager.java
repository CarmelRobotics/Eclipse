package frc.robot.subsystems.power;

import java.util.concurrent.ExecutorService;
import java.util.concurrent.Executors;
import java.util.concurrent.atomic.AtomicBoolean;
import java.util.function.Supplier;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.config.FeatureFlags;
import frc.robot.subsystems.drive.CommandSwerveDrivetrain;
import frc.robot.subsystems.shooter.Shooter;
import frc.robot.util.RobotLogger;

/**
 * Dynamic current budgeting. Instead of one static worst-case supply limit per motor group,
 * the robot switches between named {@link PowerMode}s that reallocate the battery between the
 * drivetrain and the flywheel depending on what it is doing right now (idling, aiming,
 * sprinting, running auto). Modeled on team 581's PowerManager, scoped to the two motor
 * groups that actually contend for the battery on this robot.
 *
 * <p><b>Why a background thread:</b> applying a current-limit config over CAN takes on the
 * order of a millisecond per motor and blocks. Doing eight of them inline would overrun the
 * 20 ms loop, so re-application is handed to a single-thread executor and only fires when the
 * mode actually changes (with a short stability filter so stick jitter can't spam config
 * writes).
 *
 * <p>The team-requested drive policy is separate from the optional flywheel budgets:
 * 91 A supply only in enabled autonomous, 40 A otherwise, reduced to 40 A below 8 V.
 * Flywheel budget changes remain gated by {@link FeatureFlags#POWER_MANAGER}.
 */
public class PowerManager extends SubsystemBase {
  // How many consecutive loops the desired mode must hold before we re-apply, so a stick
  // hovering at the SPRINT threshold (or a rapidly tapped trigger) can't spam CAN writes.
  private static final int kStabilityLoops = 5; // ~100 ms

  private final CommandSwerveDrivetrain m_drivetrain;
  private final Shooter m_shooter;
  private final Supplier<PowerMode> m_desiredMode;
  private final ExecutorService m_executor = Executors.newSingleThreadExecutor();
  private final DriveCurrentPolicy m_drivePolicy = new DriveCurrentPolicy();
  private final AtomicBoolean m_updateScheduled = new AtomicBoolean();
  private volatile double m_requestedDrive = DriveCurrentPolicy.NORMAL_SUPPLY_AMPS;
  private volatile double m_requestedFlywheel = PowerMode.IDLE.flywheelSupplyCurrent;
  private volatile boolean m_driveApplied;
  private boolean m_limitsRequested;

  private PowerMode m_candidate = PowerMode.IDLE;
  private int m_stableCount = 0;
  private int m_diagLoopCount = 0;  // decimates the Desired-mode dashboard write to 10 Hz
  private PowerMode m_applied = null;  // null so the first resolved mode always applies once

  public PowerManager(
      CommandSwerveDrivetrain drivetrain, Shooter shooter, Supplier<PowerMode> desiredMode) {
    m_drivetrain = drivetrain;
    m_shooter = shooter;
    m_desiredMode = desiredMode;
  }

  @Override
  public void periodic() {
    // The flag controls optional flywheel reallocation, not the auto-only drive policy.
    PowerMode desired =
        FeatureFlags.POWER_MANAGER.getAsBoolean() ? m_desiredMode.get() : PowerMode.IDLE;
    // 10 Hz diagnostic (every 5th loop); the mode logic below still runs every loop.
    if (++m_diagLoopCount % 5 == 0) {
      SmartDashboard.putString("PowerManager/Desired", desired.toString());
    }

    // Stability filter: only a mode that has held for kStabilityLoops is eligible to apply.
    if (desired == m_candidate) {
      m_stableCount++;
    } else {
      m_candidate = desired;
      m_stableCount = 0;
    }

    double driveAmps = m_drivePolicy.resolve(DriverStation.isAutonomousEnabled(),
        RobotController.getBatteryVoltage(), Timer.getFPGATimestamp());
    boolean driveChanged = !m_limitsRequested || driveAmps != m_requestedDrive;
    boolean flywheelChanged = m_candidate != m_applied && m_stableCount >= kStabilityLoops;
    boolean retryDrive = !m_driveApplied && m_diagLoopCount % 25 == 0;
    if (!driveChanged && !flywheelChanged && !retryDrive) {
      return;
    }
    if (flywheelChanged) m_applied = m_candidate;
    m_requestedDrive = driveAmps;
    m_requestedFlywheel = m_applied == null ? PowerMode.IDLE.flywheelSupplyCurrent
        : m_applied.flywheelSupplyCurrent;
    m_limitsRequested = true;
    SmartDashboard.putString("PowerManager/Applied", m_applied == null ? "IDLE" : m_applied.toString());
    SmartDashboard.putNumber("PowerManager/DriveSupplyA", m_requestedDrive);
    SmartDashboard.putNumber("PowerManager/FlywheelSupplyA", m_requestedFlywheel);
    RobotLogger.log("PowerManager/Mode", m_applied == null ? "IDLE" : m_applied.toString());
    RobotLogger.log("PowerManager/DriveSupplyA", m_requestedDrive);
    RobotLogger.log("PowerManager/FlywheelSupplyA", m_requestedFlywheel);
    requestUpdate();
  }

  private void requestUpdate() {
    if (!m_updateScheduled.compareAndSet(false, true)) return;
    m_executor.execute(() -> {
      double drive = m_requestedDrive;
      double flywheel = m_requestedFlywheel;
      try {
        // Coalesce pending changes; a queued auto request must not restore 91 A in teleop.
        m_driveApplied = m_drivetrain.applyDriveSupplyCurrentLimit(drive);
        SmartDashboard.putBoolean("PowerManager/DriveLimitsApplied", m_driveApplied);
        m_shooter.applyFlywheelSupplyCurrentLimit(flywheel);
      } catch (RuntimeException error) {
        m_driveApplied = false;
        DriverStation.reportError("Current-limit update failed: " + error.getMessage(), false);
      } finally {
        m_updateScheduled.set(false);
        if (drive != m_requestedDrive || flywheel != m_requestedFlywheel) requestUpdate();
      }
    });
  }
}
