package frc.robot.subsystems.power;

import java.util.concurrent.ExecutorService;
import java.util.concurrent.Executors;
import java.util.function.Supplier;

import dev.doglog.DogLog;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.config.FeatureFlags;
import frc.robot.subsystems.drive.CommandSwerveDrivetrain;
import frc.robot.subsystems.shooter.Shooter;

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
 * <p><b>Safety:</b> gated by {@link FeatureFlags#POWER_MANAGER}. When the flag is off the mode
 * is pinned to {@code IDLE}, whose limits equal the compiled static defaults -- so disabling
 * the flag (its default state) restores exactly the known-good static behavior, and enabling
 * it is a deliberate, reversible field decision.
 */
public class PowerManager extends SubsystemBase {
  // How many consecutive loops the desired mode must hold before we re-apply, so a stick
  // hovering at the SPRINT threshold (or a rapidly tapped trigger) can't spam CAN writes.
  private static final int kStabilityLoops = 5; // ~100 ms

  private final CommandSwerveDrivetrain m_drivetrain;
  private final Shooter m_shooter;
  private final Supplier<PowerMode> m_desiredMode;
  private final ExecutorService m_executor = Executors.newSingleThreadExecutor();

  private PowerMode m_candidate = PowerMode.IDLE;
  private int m_stableCount = 0;
  private PowerMode m_applied = null;  // null so the first resolved mode always applies once

  public PowerManager(
      CommandSwerveDrivetrain drivetrain, Shooter shooter, Supplier<PowerMode> desiredMode) {
    m_drivetrain = drivetrain;
    m_shooter = shooter;
    m_desiredMode = desiredMode;
  }

  @Override
  public void periodic() {
    // Flag off -> pin IDLE (= static defaults), so turning the feature off always restores
    // the known-good limits rather than freezing at whatever mode was last applied.
    PowerMode desired =
        FeatureFlags.POWER_MANAGER.getAsBoolean() ? m_desiredMode.get() : PowerMode.IDLE;
    SmartDashboard.putString("PowerManager/Desired", desired.toString());

    // Stability filter: only a mode that has held for kStabilityLoops is eligible to apply.
    if (desired == m_candidate) {
      m_stableCount++;
    } else {
      m_candidate = desired;
      m_stableCount = 0;
    }

    if (m_candidate == m_applied || m_stableCount < kStabilityLoops) {
      return;
    }
    m_applied = m_candidate;
    SmartDashboard.putString("PowerManager/Applied", m_applied.toString());
    DogLog.log("PowerManager/Mode", m_applied.toString());
    DogLog.log("PowerManager/DriveSupplyA", m_applied.driveSupplyCurrent);
    DogLog.log("PowerManager/FlywheelSupplyA", m_applied.flywheelSupplyCurrent);

    final double driveAmps = m_applied.driveSupplyCurrent;
    final double flywheelAmps = m_applied.flywheelSupplyCurrent;
    m_executor.execute(
        () -> {
          m_drivetrain.applyDriveSupplyCurrentLimit(driveAmps);
          m_shooter.applyFlywheelSupplyCurrentLimit(flywheelAmps);
        });
  }
}
