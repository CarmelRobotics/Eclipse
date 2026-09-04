package frc.robot.config;

import frc.robot.util.FeatureFlag;
import java.util.function.BooleanSupplier;

/**
 * Live dashboard toggles for field debugging. Flip one off under {@code FeatureFlags/*} on
 * the dashboard to disable a behavior without redeploying. Every flag defaults to the normal
 * robot; turning one OFF is a deliberate intervention.
 *
 * <p>Typical field uses:
 * <ul>
 *   <li>{@code VisionFusion} off — camera is feeding bad poses; run pure odometry.</li>
 *   <li>{@code ShootOnTheMove} off — aim as if stationary while diagnosing lead/aim.</li>
 *   <li>{@code ShiftsGate} / {@code AllianceZoneGate} off — force the right trigger to offer
 *       a hub shot regardless of SHIFTS state or field position (e.g. bad game data, or
 *       bench testing from anywhere).</li>
 *   <li>{@code PowerManager} off — fall back to the static, known-good current limits.</li>
 * </ul>
 */
public final class FeatureFlags {
  private FeatureFlags() {}

  /** Fuse Limelight pose into odometry. Off = wheel odometry only (still aims, just drifts). */
  public static final BooleanSupplier VISION_FUSION = FeatureFlag.of("VisionFusion", true);

  /** Motion-compensate the aim. Off = aim at the hub as if the robot were stationary. */
  public static final BooleanSupplier SHOOT_ON_THE_MOVE = FeatureFlag.of("ShootOnTheMove", true);

  /** Compensate the shot vector and pivot for Pigeon pitch/roll. */
  public static final BooleanSupplier TILT_COMPENSATION = FeatureFlag.of("TiltCompensation", true);

  /** Add a short acceleration lead after a sudden drivetrain disturbance/impact. */
  public static final BooleanSupplier IMPACT_COMPENSATION = FeatureFlag.of("ImpactCompensation", true);

  /** Require our hub to be SHIFTS-active for a hub shot. Off = the range/zone decide alone. */
  public static final BooleanSupplier SHIFTS_GATE = FeatureFlag.of("ShiftsGate", true);

  /** Require being in the alliance zone for a hub shot. Off = range decides alone. */
  public static final BooleanSupplier ALLIANCE_ZONE_GATE = FeatureFlag.of("AllianceZoneGate", true);

  /** Pre-spin the flywheel near the hub. Off = the drum only spins on a shot command. */
  public static final BooleanSupplier IDLE_SPIN = FeatureFlag.of("IdleSpin", true);

  /** Indexer closed-loop velocity control (steady feed cadence under load). Off = legacy
   *  open-loop voltage. Kept as a flag so shot consistency can be A/B'd directly at the field. */
  public static final BooleanSupplier INDEXER_VELOCITY_CONTROL =
      FeatureFlag.of("IndexerVelocityControl", true);

  /** Trench/tower driver assists. Off = fully manual driving everywhere. */
  public static final BooleanSupplier DRIVER_ASSISTS = FeatureFlag.of("DriverAssists", true);

  /** Dynamic current budgeting. Off = the static compiled current limits (IDLE mode). Starts
   *  OFF: it re-allocates current limits at runtime and hasn't been field-validated -- flip it
   *  on at the field once you're watching the battery traces. */
  public static final BooleanSupplier POWER_MANAGER = FeatureFlag.of("PowerManager", false);
}
