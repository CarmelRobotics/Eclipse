package frc.robot.util;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/**
 * A number that can be edited live from the dashboard while tuning, and is locked to its
 * compile-time default (and cheap to read) once tuning mode is off. Modeled on the common
 * FRC LoggedTunableNumber pattern used by 6328/1678/2910, backed here by SmartDashboard.
 *
 * <p>Usage:
 * <pre>
 *   private final LoggedTunableNumber kP = new LoggedTunableNumber("Shooter/kP", 0.3);
 *   ...
 *   if (kP.hasChanged()) reapplyGains(kP.get());
 * </pre>
 *
 * Call {@link #setTuningMode(boolean)} once at startup; set it false for competition so
 * the dashboard can't accidentally change a value mid-match.
 */
public class LoggedTunableNumber {
    private static boolean tuningMode = true;

    private final String key;
    private final double defaultValue;
    private double lastValue;
    private boolean firstCheck = true;

    public LoggedTunableNumber(String key, double defaultValue) {
        this.key = key;
        this.defaultValue = defaultValue;
        this.lastValue = defaultValue;
        if (tuningMode) {
            // Seed the dashboard entry without clobbering a value already present.
            SmartDashboard.putNumber(key, SmartDashboard.getNumber(key, defaultValue));
        }
    }

    /** Global switch. When false, every tunable returns its default and ignores the dashboard. */
    public static void setTuningMode(boolean enabled) {
        tuningMode = enabled;
    }

    public double get() {
        return tuningMode ? SmartDashboard.getNumber(key, defaultValue) : defaultValue;
    }

    /**
     * True on the first call (so callers can apply an initial config), and thereafter
     * only when the dashboard value has changed since the last check. Gate expensive work
     * (e.g. re-applying motor configs over CAN) behind this.
     */
    public boolean hasChanged() {
        double current = get();
        if (firstCheck || current != lastValue) {
            firstCheck = false;
            lastValue = current;
            return true;
        }
        return false;
    }
}
