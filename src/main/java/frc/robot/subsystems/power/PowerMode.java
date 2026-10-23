package frc.robot.subsystems.power;

/**
 * Optional flywheel supply budgets, gated by FeatureFlags.POWER_MANAGER.
 * Drive supply is controlled exclusively by DriveCurrentPolicy (91 A auto / 40 A other).
 */
public enum PowerMode {
  IDLE(35),
  SCORING(50),
  SPRINT(20),
  AUTO(45);

  final double flywheelSupplyCurrent;

  PowerMode(double flywheelSupplyCurrent) {
    this.flywheelSupplyCurrent = flywheelSupplyCurrent;
  }
}
