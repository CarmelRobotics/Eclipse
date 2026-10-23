package frc.robot.auto;

import static org.junit.jupiter.api.Assertions.*;

import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.trajectory.PathPlannerTrajectory;
import com.pathplanner.lib.util.FlippingUtil;
import edu.wpi.first.hal.HAL;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Commands;
import java.nio.file.Files;
import java.nio.file.Path;
import java.util.ArrayList;
import java.util.List;
import java.util.Locale;
import org.junit.jupiter.api.Test;

/** Geometric pickup coverage, NOT a fuel physics or scoring simulator. */
class FuelCoverageTest {
  // Rear intake, 9 inches past a 0.90 m bumper. Usable 0.80 m roller width is
  // an assumption to verify physically, not a measured CAD dimension.
  private static final double WIDTH = 0.80;
  private static final double REAR_REACH = 0.45 + 9 * 0.0254;
  private static final double GRID = 0.04;

  @Test
  void collectStartingCarpetAndSweepNeutralPocketBehindOwnHub() throws Exception {
    HAL.initialize(500, 0);
    NamedCommands.registerCommand("intake run", Commands.none());
    var config = RobotConfig.fromGUISettings();
    List<String> cells = new ArrayList<>();
    cells.add("side,alliance,x,y,cycle,region");
    List<String> report = new ArrayList<>();
    List<org.junit.jupiter.api.function.Executable> checks = new ArrayList<>();
    report.add("side,alliance,doubleFirstCarpetM2,doubleFreshHubPocketM2,singleHubPocketM2,doubleFirstHubPocketM2,doubleSecondHubPocketM2,doubleFreshSecondCarpetM2,totalCarpetM2");
    for (String side : List.of("LEFT", "RIGHT")) {
      List<PathPlannerTrajectory> paths = new ArrayList<>();
      for (String leg : List.of("DOUBLE_START_TO_FUEL", "COLLECT1_TO_SCORE",
          "SCORE_TO_FUEL2", "COLLECT2_TO_SCORE", "START_TO_FUEL", "SINGLE_COLLECT_TO_SCORE")) {
        paths.add(PathPlannerPath.fromPathFile("ECLIPSE_" + side + "_" + leg)
            .getIdealTrajectory(config).orElseThrow());
      }
      for (boolean red : new boolean[] {false, true}) {
        var trajectories = red ? paths.stream().map(t -> t.flip()).toList() : paths;
        var firstEntry = PathPlannerPath.fromPathFile("ECLIPSE_" + side + "_DOUBLE_START_TO_FUEL");
        var firstSamples = samples(trajectories.subList(0, 2),
            CompetitionPathsTest.pickupStartTime(firstEntry, trajectories.get(0)));
        var entry = PathPlannerPath.fromPathFile("ECLIPSE_" + side + "_SCORE_TO_FUEL2");
        var secondSamples = samples(trajectories.subList(2, 4),
            CompetitionPathsTest.pickupStartTime(entry, trajectories.get(2)));
        var singleEntry = PathPlannerPath.fromPathFile("ECLIPSE_" + side + "_START_TO_FUEL");
        var singleSamples = samples(List.of(trajectories.get(4), trajectories.get(5)),
            CompetitionPathsTest.pickupStartTime(singleEntry, trajectories.get(4)));
        int firstCarpet = 0;
        int freshPocket = 0;
        int singlePocket = 0;
        int firstPocket = 0;
        int secondPocket = 0;
        int freshCarpet = 0;
        int totalCarpet = 0;
        // Two explicit regions: the nominal midfield carpet and the team's
        // observed neutral-side accumulation pocket behind its own hub.
        // No fuel density or ball-motion assumptions are attached to either.
        for (int region = 0; region < 2; region++) {
          int nx = region == 0 ? 46 : 40;
          int ny = region == 0 ? 116 : 50;
          double x0 = region == 0 ? 7.36 : 5.55;
          double y0 = region == 0 ? 1.72 : 3.05;
          for (int ix = 0; ix < nx; ix++) {
            for (int iy = 0; iy < ny; iy++) {
              var blue = new Translation2d(x0 + ix * GRID, y0 + iy * GRID);
              var fuel = red ? FlippingUtil.flipFieldPosition(blue) : blue;
              boolean one = covers(firstSamples, fuel);
              boolean two = covers(secondSamples, fuel);
              if (region == 0 && one) firstCarpet++;
              if (region == 0 && two && !one) freshCarpet++;
              if (region == 0 && (one || two)) totalCarpet++;
              if (region == 1 && two && !one) freshPocket++;
              if (region == 1 && one) firstPocket++;
              if (region == 1 && two) secondPocket++;
              if (region == 1 && covers(singleSamples, fuel)) singlePocket++;
              if (one || two) cells.add(String.format(Locale.ROOT, "%s,%s,%.3f,%.3f,%d,%s",
                  side, red ? "Red" : "Blue", fuel.getX(), fuel.getY(), one ? 1 : 2,
                  region == 0 ? "carpet" : "hub-pocket"));
            }
          }
        }
        double firstArea = firstCarpet * GRID * GRID;
        double freshArea = freshPocket * GRID * GRID;
        double singleArea = singlePocket * GRID * GRID;
        double firstPocketArea = firstPocket * GRID * GRID;
        double secondPocketArea = secondPocket * GRID * GRID;
        double freshCarpetArea = freshCarpet * GRID * GRID;
        double totalCarpetArea = totalCarpet * GRID * GRID;
        System.out.printf(Locale.ROOT,
            "%s %s: double first carpet %.3fm2; fresh hub pocket %.3fm2; single pocket %.3fm2; first/second pocket %.3f/%.3fm2%n",
            side, red ? "Red" : "Blue", firstArea, freshArea, singleArea, firstPocketArea, secondPocketArea);
        System.out.printf(Locale.ROOT, "  Fresh second carpet %.3fm2; total carpet %.3fm2%n", freshCarpetArea, totalCarpetArea);
        report.add(String.format(Locale.ROOT, "%s,%s,%.4f,%.4f,%.4f,%.4f,%.4f,%.4f,%.4f",
            side, red ? "Red" : "Blue", firstArea, freshArea, singleArea, firstPocketArea, secondPocketArea,
            freshCarpetArea, totalCarpetArea));
        checks.add(() -> assertTrue(firstArea >= 1.20, side + " compact first pickup misses starting carpet"));
        checks.add(() -> assertTrue(freshCarpetArea >= 1.20, side + " second trip needs fresh carpet, not retracing"));
        checks.add(() -> assertTrue(totalCarpetArea >= 3.00, side + " two trips do not cover complementary carpet"));
        // Track both total and fresh coverage: overlap with trip one is not new fuel.
        checks.add(() -> assertTrue(secondPocketArea >= 1.00, side + " second route misses hub pocket: " + secondPocketArea));
        checks.add(() -> assertTrue(freshArea >= 1.00, side + " second route mostly retraces the first pickup: " + freshArea));
        checks.add(() -> assertTrue(singleArea >= 0.60, side + " single does not sweep the hub pocket: " + singleArea));
      }
    }
    var dir = Path.of("build", "reports", "auto-paths");
    Files.createDirectories(dir);
    Files.write(dir.resolve("pickup-cells.csv"), cells);
    Files.write(dir.resolve("coverage.csv"), report);
    assertAll(checks);
  }

  private record IntakeSample(double x, double y, double cos, double sin) {}

  private static List<IntakeSample> samples(List<PathPlannerTrajectory> trajectories) {
    return samples(trajectories, 0);
  }

  private static List<IntakeSample> samples(List<PathPlannerTrajectory> trajectories, double firstPickupTime) {
    List<IntakeSample> result = new ArrayList<>();
    for (int i = 0; i < trajectories.size(); i++) {
      var trajectory = trajectories.get(i);
      for (double t = i == 0 ? firstPickupTime : 0; t <= trajectory.getTotalTimeSeconds(); t += 0.02) {
        var state = trajectory.sample(t);
        var forward = state.pose.getRotation();
        double intoIntake = -(state.fieldSpeeds.vxMetersPerSecond * forward.getCos()
            + state.fieldSpeeds.vyMetersPerSecond * forward.getSin());
        if (intoIntake > 0.15 && CompetitionPathsTest.rearIntakeAngle(state.pose, state.fieldSpeeds) <= 10)
          result.add(new IntakeSample(
            state.pose.getX(), state.pose.getY(), forward.getCos(), forward.getSin()));
      }
    }
    return result;
  }

  private static boolean covers(List<IntakeSample> samples, Translation2d fuel) {
    for (var sample : samples) {
      double dx = fuel.getX() - sample.x();
      double dy = fuel.getY() - sample.y();
      double localX = dx * sample.cos() + dy * sample.sin();
      double localY = -dx * sample.sin() + dy * sample.cos();
      if (localX >= -REAR_REACH - 0.08 && localX <= -REAR_REACH + 0.08
          && Math.abs(localY) <= WIDTH / 2) return true;
    }
    return false;
  }
}
