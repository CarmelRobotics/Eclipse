package frc.robot.util;

import edu.wpi.first.util.datalog.BooleanLogEntry;
import edu.wpi.first.util.datalog.DataLog;
import edu.wpi.first.util.datalog.DoubleLogEntry;
import edu.wpi.first.util.datalog.IntegerLogEntry;
import edu.wpi.first.util.datalog.StringLogEntry;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import java.util.HashMap;
import java.util.Map;

/** Small, low-rate WPILib data logger for competition diagnostics. */
public final class RobotLogger {
  private static final double kPeriodicLogIntervalSeconds = 0.1; // 10 Hz

  private static final Map<String, DoubleLogEntry> s_doubleEntries = new HashMap<>();
  private static final Map<String, BooleanLogEntry> s_booleanEntries = new HashMap<>();
  private static final Map<String, IntegerLogEntry> s_integerEntries = new HashMap<>();
  private static final Map<String, StringLogEntry> s_stringEntries = new HashMap<>();

  private static DataLog s_log;
  private static boolean s_started;
  private static boolean s_periodicTick;
  private static double s_nextPeriodicLogTime;

  private RobotLogger() {}

  /** Start WPILib's file logger without mirroring all NetworkTables traffic into the log. */
  public static void start() {
    if (s_started) {
      return;
    }

    DataLogManager.start();
    DataLogManager.logNetworkTables(false);
    DataLogManager.logConsoleOutput(false);
    s_log = DataLogManager.getLog();
    s_started = true;
  }

  /** Call once at the beginning of robotPeriodic(). Enables low-rate logs only while enabled. */
  public static void periodic() {
    s_periodicTick = false;
    if (!s_started || !DriverStation.isEnabled()) {
      return;
    }

    double now = Timer.getFPGATimestamp();
    if (now >= s_nextPeriodicLogTime) {
      s_periodicTick = true;
      s_nextPeriodicLogTime = now + kPeriodicLogIntervalSeconds;
    }
  }

  public static boolean shouldLogPeriodic() {
    return s_periodicTick;
  }

  public static void log(String key, double value) {
    if (!s_started) {
      return;
    }
    s_doubleEntries.computeIfAbsent(key, k -> new DoubleLogEntry(s_log, k)).append(value);
  }

  public static void log(String key, boolean value) {
    if (!s_started) {
      return;
    }
    s_booleanEntries.computeIfAbsent(key, k -> new BooleanLogEntry(s_log, k)).append(value);
  }

  public static void log(String key, int value) {
    if (!s_started) {
      return;
    }
    s_integerEntries.computeIfAbsent(key, k -> new IntegerLogEntry(s_log, k)).append(value);
  }

  public static void log(String key, String value) {
    if (!s_started) {
      return;
    }
    s_stringEntries.computeIfAbsent(key, k -> new StringLogEntry(s_log, k)).append(value);
  }
}
