package frc.robot.subsystems.power;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;

/** Auto-only drive supply boost with voltage hysteresis; independent of dashboard feature flags. */
public final class DriveCurrentPolicy {
  public static final double NORMAL_SUPPLY_AMPS = 40;
  public static final double AUTO_SUPPLY_AMPS = 91;
  public static final double STATOR_AMPS = 120;
  public static final double LOW_VOLTAGE = 8.0;
  private boolean reduced;
  private double recoverySince = Double.NaN;

  /** Applied again on the CAN worker to reject a stale boost after mode/voltage changes. */
  public static double permittedRequest(double requested, boolean autonomousEnabled, double voltage) {
    double maximum = autonomousEnabled && Double.isFinite(voltage) && voltage >= LOW_VOLTAGE
        ? AUTO_SUPPLY_AMPS : NORMAL_SUPPLY_AMPS;
    return Double.isFinite(requested) ? Math.max(0, Math.min(requested, maximum)) : NORMAL_SUPPLY_AMPS;
  }

  /** Complete runtime config; do not inherit Phoenix's timed 40 A lower-limit default. */
  public static CurrentLimitsConfigs currentLimits(
      double requested, boolean autonomousEnabled, double voltage) {
    double supply = permittedRequest(requested, autonomousEnabled, voltage);
    return new CurrentLimitsConfigs()
        .withSupplyCurrentLimit(supply)
        .withSupplyCurrentLimitEnable(true)
        .withSupplyCurrentLowerLimit(supply)
        .withSupplyCurrentLowerTime(0)
        .withStatorCurrentLimit(STATOR_AMPS)
        .withStatorCurrentLimitEnable(true);
  }

  public double resolve(boolean autonomousEnabled, double voltage, double nowSeconds) {
    if (!autonomousEnabled) {
      reduced = false;
      recoverySince = Double.NaN;
      return NORMAL_SUPPLY_AMPS;
    }
    if (!Double.isFinite(voltage) || voltage < LOW_VOLTAGE) {
      reduced = true;
      recoverySince = Double.NaN;
    } else if (reduced) {
      if (voltage >= 9.5) {
        if (!Double.isFinite(recoverySince)) recoverySince = nowSeconds;
        if (nowSeconds - recoverySince >= 0.5) reduced = false;
      } else {
        recoverySince = Double.NaN;
      }
    }
    return reduced ? NORMAL_SUPPLY_AMPS : AUTO_SUPPLY_AMPS;
  }
}
