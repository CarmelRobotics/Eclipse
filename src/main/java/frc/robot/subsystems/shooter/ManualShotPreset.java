package frc.robot.subsystems.shooter;

import edu.wpi.first.math.MathUtil;

/** An explicitly manual, latched shot; never derives range from untrusted odometry. */
public record ManualShotPreset(double distanceMeters, double pivotRotations, double wheelRps) {
  public static final double DEFAULT_DISTANCE_METERS = ShooterConstants.kTowerShotDistanceMeters;

  /** Fixed tower range; stale dashboard range values must not select a different shot. */
  public static ManualShotPreset tower(double pivotOffset, double rpsOffset) {
    return create(ShooterConstants.kTowerShotDistanceMeters, pivotOffset, rpsOffset);
  }

  // Vision shots use a slightly lower pivot position than the field-tuned table.
  // The hood model in ShooterConstants is 70 degrees at stow and one launch
  // degree steeper for each output degree the pivot moves back toward stow.
  private static final double VISION_PIVOT_REDUCTION_ROT = 0.008; // 2.88 output degrees
  private static final double RELEASE_TO_HUB_HEIGHT_METERS = 1.524;

  public static ManualShotPreset vision(double distance, double pivotOffset, double rpsOffset) {
    ManualShotPreset baseline = create(distance, pivotOffset, 0);
    double pivot = Math.max(0, baseline.pivotRotations() - VISION_PIVOT_REDUCTION_ROT);
    double oldAngle = Math.toRadians(70 - 360 * baseline.pivotRotations());
    double newAngle = Math.toRadians(70 - 360 * pivot);
    double range = baseline.distanceMeters();
    // v^2 = g*d^2 / (2*cos(theta)^2*(d*tan(theta)-height)).
    // The ratio preserves the table's measured wheel-to-ball speed calibration.
    double oldDenominator = Math.pow(Math.cos(oldAngle), 2)
        * (range * Math.tan(oldAngle) - RELEASE_TO_HUB_HEIGHT_METERS);
    double newDenominator = Math.pow(Math.cos(newAngle), 2)
        * (range * Math.tan(newAngle) - RELEASE_TO_HUB_HEIGHT_METERS);
    double rps = baseline.wheelRps();
    if (oldDenominator > 0 && newDenominator > 0) {
      rps *= Math.sqrt(oldDenominator / newDenominator);
    }
    rps += Double.isFinite(rpsOffset) ? rpsOffset : 0;
    return new ManualShotPreset(range, pivot,
        MathUtil.clamp(rps, 5, ShooterConstants.kSendShooterRps));
  }

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
