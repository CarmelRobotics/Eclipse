package frc.robot.subsystems.shooter;

import edu.wpi.first.math.MathUtil;

/** An explicitly manual, latched shot; never derives range from untrusted odometry. */
public record ManualShotPreset(double distanceMeters, double pivotRotations, double wheelRps) {
  public static final double DEFAULT_DISTANCE_METERS = 2.75;
  public static final String DISTANCE_KEY = "ShotTuning/ManualShotDistanceM";

  public static ManualShotPreset create(double distance, double pivotOffset, double rpsOffset) {
    double range = Double.isFinite(distance) ? MathUtil.clamp(distance, 1.5, 5.0)
        : DEFAULT_DISTANCE_METERS;
    double pivot = ShooterConstants.getScorePivotPosition(range)
        + (Double.isFinite(pivotOffset) ? pivotOffset : 0);
    double rps = ShooterConstants.getScoreShooterRps(range)
        + (Double.isFinite(rpsOffset) ? rpsOffset : 0);
    return new ManualShotPreset(range, MathUtil.clamp(pivot, 0, 0.10),
        MathUtil.clamp(rps, 5, ShooterConstants.kSendShooterRps));
  }
}
