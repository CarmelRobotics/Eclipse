package frc.robot.util;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import java.util.function.BooleanSupplier;

/**
 * A boolean that can be toggled live from the dashboard, for enabling/disabling a feature
 * at the field without a redeploy. Boolean analog of {@link LoggedTunableNumber}, modeled on
 * team 581's FeatureFlag.
 *
 * <p>Unlike tunable numbers, feature flags are <b>always live</b> (not gated by tuning mode):
 * the entire point is to intervene during a practice or match when a feature misbehaves --
 * e.g. the camera is feeding garbage, so you flip {@code FeatureFlags/VisionFusion} off and
 * run on odometry. Compiled defaults are the normal "feature on" state, so a flag nobody
 * touches behaves exactly as before.
 */
public final class FeatureFlag {
  private FeatureFlag() {}

  /** Creates a dashboard-backed flag under {@code FeatureFlags/<name>} and returns a
   *  supplier that reads its current value. Seeds the key without clobbering an existing
   *  value so a mid-session toggle survives a code restart. */
  public static BooleanSupplier of(String name, boolean defaultValue) {
    String key = "FeatureFlags/" + name;
    SmartDashboard.putBoolean(key, SmartDashboard.getBoolean(key, defaultValue));
    return () -> SmartDashboard.getBoolean(key, defaultValue);
  }
}
