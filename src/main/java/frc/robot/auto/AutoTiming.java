package frc.robot.auto;

import edu.wpi.first.wpilibj2.command.Command;
import java.util.function.BooleanSupplier;

/** Match-time budget shared by autonomous commands and the deployed-path regression tests. */
public final class AutoTiming {
  private AutoTiming() {}

  public static final double DURATION_SECONDS = 19.8;
  public static final double INITIAL_STOW_TIMEOUT_SECONDS = 0.4;
  public static final double TRAVEL_TIMEOUT_SECONDS = 1.6;
  public static final double MAX_TRAVEL_ANGULAR_SPEED_RAD_PER_SEC = 0.25;
  // Rechecked AFTER actual stow/alignment, not charged a second time as a worst-case
  // timeout. Includes the optimized drive cycle, 1.25 s spin-up, useful final feed
  // and scheduler margin. A slow preparation stays at the first score instead.
  public static final double SECOND_CYCLE_RESERVE_SECONDS = 9.0;
  // New paths only: 7.446 s drive + 1.25 s readiness + .5 s feed + .25 s scheduling.
  public static final double MAX_AGGRESSION_SECOND_CYCLE_RESERVE_SECONDS = 9.45;
  // Team-requested timed unload. This is not a hopper-empty or jam-clear indication.
  public static final double DUMP_FEED_SECONDS = 5.0;
  public static final double MIN_FINAL_FEED_SECONDS = 0.5;
  // Team requested faster loaded crossings. Planar simulation cannot validate
  // pitch, bounce, unloading wheels or fuel retention on a physical bump.
  public static final double BUMP_SPEED_METERS_PER_SECOND = 4.2;
  public static final double COLLECT_SETTLE_SECONDS = 0.15;

  /** Do not leave a successful first score unless there is time to get back and shoot again. */
  public static boolean canStartSecondCycle(double elapsedSeconds) {
    return canStartSecondCycle(elapsedSeconds, SECOND_CYCLE_RESERVE_SECONDS);
  }

  public static boolean canStartSecondCycle(double elapsedSeconds, double reserveSeconds) {
    return Double.isFinite(elapsedSeconds) && elapsedSeconds >= 0
        && Double.isFinite(reserveSeconds) && reserveSeconds > 0 && reserveSeconds < DURATION_SECONDS
        && elapsedSeconds <= DURATION_SECONDS - reserveSeconds;
  }

  public static Command guard(Command selected, BooleanSupplier mayContinue,
      Runnable start, Runnable stop) {
    return selected.onlyWhile(mayContinue).withTimeout(DURATION_SECONDS)
        .beforeStarting(start).finallyDo(interrupted -> stop.run());
  }
}
