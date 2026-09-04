package frc.robot.subsystems.shooter;

import edu.wpi.first.wpilibj.Filesystem;
import java.io.IOException;
import java.nio.file.Files;
import java.nio.file.Path;
import java.util.ArrayList;
import java.util.Comparator;
import java.util.List;
import java.util.Optional;

/** Runtime reader and bilinear interpolator for the offline trajectory map. */
public final class TrajectoryMap {
  private static final double kEpsilon = 1e-9;
  private static final List<Entry> ENTRIES = load();

  private TrajectoryMap() {}

  public record Solution(
      double pivotOutputRotations,
      double wheelRps,
      double tofSeconds,
      double robustnessMarginMeters) {}

  private record Entry(double distance, double radialVelocity, Solution solution) {}

  /**
   * Looks up a shot using distance and radial velocity. Positive radial velocity means the
   * robot is moving toward the target. Empty means the generated map does not cover the
   * requested cell; callers should use the field-calibrated one-dimensional fallback.
   */
  public static Optional<Solution> lookup(double distance, double radialVelocity) {
    if (ENTRIES.isEmpty()) {
      return Optional.empty();
    }

    double minDistance = ENTRIES.stream().mapToDouble(Entry::distance).min().orElse(distance);
    double maxDistance = ENTRIES.stream().mapToDouble(Entry::distance).max().orElse(distance);
    double minVelocity = ENTRIES.stream().mapToDouble(Entry::radialVelocity).min().orElse(radialVelocity);
    double maxVelocity = ENTRIES.stream().mapToDouble(Entry::radialVelocity).max().orElse(radialVelocity);
    double d = clamp(distance, minDistance, maxDistance);
    double v = clamp(radialVelocity, minVelocity, maxVelocity);

    double d0 = ENTRIES.stream().mapToDouble(Entry::distance).filter(x -> x <= d + kEpsilon).max().orElse(d);
    double d1 = ENTRIES.stream().mapToDouble(Entry::distance).filter(x -> x >= d - kEpsilon).min().orElse(d);
    double v0 = ENTRIES.stream().mapToDouble(Entry::radialVelocity).filter(x -> x <= v + kEpsilon).max().orElse(v);
    double v1 = ENTRIES.stream().mapToDouble(Entry::radialVelocity).filter(x -> x >= v - kEpsilon).min().orElse(v);

    Entry a = find(d0, v0);
    Entry b = find(d1, v0);
    Entry c = find(d0, v1);
    Entry e = find(d1, v1);
    if (a == null || b == null || c == null || e == null) {
      // The generator omits physically impossible cells. A nearest valid cell is safer than
      // extrapolating beyond the tested envelope; the caller still publishes the margin.
      return ENTRIES.stream()
          .min(Comparator.comparingDouble(entry ->
              Math.hypot(entry.distance() - d, entry.radialVelocity() - v)))
          .map(Entry::solution);
    }

    double td = d1 == d0 ? 0.0 : (d - d0) / (d1 - d0);
    double tv = v1 == v0 ? 0.0 : (v - v0) / (v1 - v0);
    return Optional.of(bilinear(a.solution(), b.solution(), c.solution(), e.solution(), td, tv));
  }

  private static Solution bilinear(Solution a, Solution b, Solution c, Solution d, double td, double tv) {
    return new Solution(
        interpolate(interpolate(a.pivotOutputRotations(), b.pivotOutputRotations(), td),
            interpolate(c.pivotOutputRotations(), d.pivotOutputRotations(), td), tv),
        interpolate(interpolate(a.wheelRps(), b.wheelRps(), td),
            interpolate(c.wheelRps(), d.wheelRps(), td), tv),
        interpolate(interpolate(a.tofSeconds(), b.tofSeconds(), td),
            interpolate(c.tofSeconds(), d.tofSeconds(), td), tv),
        interpolate(interpolate(a.robustnessMarginMeters(), b.robustnessMarginMeters(), td),
            interpolate(c.robustnessMarginMeters(), d.robustnessMarginMeters(), td), tv));
  }

  private static double interpolate(double a, double b, double t) {
    return a + (b - a) * t;
  }

  private static Entry find(double distance, double radialVelocity) {
    return ENTRIES.stream()
        .filter(entry -> Math.abs(entry.distance() - distance) < kEpsilon
            && Math.abs(entry.radialVelocity() - radialVelocity) < kEpsilon)
        .findFirst()
        .orElse(null);
  }

  private static double clamp(double value, double min, double max) {
    return Math.max(min, Math.min(max, value));
  }

  private static List<Entry> load() {
    Path path = Filesystem.getDeployDirectory().toPath().resolve("trajectory-map.csv");
    List<Entry> entries = new ArrayList<>();
    try {
      if (!Files.exists(path)) {
        return entries;
      }
      List<String> lines = Files.readAllLines(path);
      for (int i = 1; i < lines.size(); i++) {
        String line = lines.get(i).trim();
        if (line.isEmpty()) {
          continue;
        }
        String[] values = line.split(",");
        if (values.length < 8) {
          continue;
        }
        entries.add(new Entry(
            Double.parseDouble(values[0]),
            Double.parseDouble(values[1]),
            new Solution(
                Double.parseDouble(values[3]),
                Double.parseDouble(values[5]),
                Double.parseDouble(values[6]),
                Double.parseDouble(values[7]))));
      }
    } catch (IOException | NumberFormatException ex) {
      entries.clear();
    }
    return entries;
  }
}
