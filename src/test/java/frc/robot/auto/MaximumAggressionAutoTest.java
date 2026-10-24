package frc.robot.auto;

import static org.junit.jupiter.api.Assertions.*;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.trajectory.PathPlannerTrajectory;
import com.pathplanner.lib.util.FlippingUtil;
import edu.wpi.first.hal.HAL;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.simulation.SimHooks;
import edu.wpi.first.wpilibj2.command.Commands;
import java.nio.file.Files;
import java.nio.file.Path;
import java.util.ArrayList;
import java.util.List;
import java.util.Locale;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;

/** New opt-in autos; baseline routes and their existing acceptance tolerances stay intact. */
class MaximumAggressionAutoTest {
  private static final List<String> LEGS = List.of("START_TO_FUEL", "COLLECT1_TO_SCORE",
      "SCORE_TO_FUEL2", "COLLECT2_TO_SCORE");
  private RobotConfig config;

  @BeforeEach void initialize() throws Exception {
    assertTrue(HAL.initialize(500, 0));
    CompetitionPathsTest.initialize();
    PathPlannerPath.clearCache();
    NamedCommands.registerCommand("intake run", Commands.none());
    config = RobotConfig.fromGUISettings();
  }

  @Test void trajectoriesClearObstaclesKeepIntakeLeadingAndFitReserve() throws Exception {
    var report = new ArrayList<String>(List.of("side,firstDriveS,secondDriveS,centerlineS,firstActivePickupS,maxPlannedPickupAngleDeg"));
    var samples = new ArrayList<String>(List.of("path,alliance,t,x,y,headingDeg,speedMps"));
    var checks = new ArrayList<org.junit.jupiter.api.function.Executable>();
    for (String side : List.of("LEFT", "RIGHT")) {
      var paths = routes(side, true);
      double first = paths.get(0).getTotalTimeSeconds() + paths.get(1).getTotalTimeSeconds();
      double second = paths.get(2).getTotalTimeSeconds() + paths.get(3).getTotalTimeSeconds();
      double centerline = Double.NaN, activePickup = Double.NaN, maxAngle = 0;
      for (int leg = 0; leg < 4; leg++) {
        var file = PathPlannerPath.fromPathFile("ECLIPSE_MAX_" + side + "_" + LEGS.get(leg));
        assertFalse(file.getGlobalConstraints().unlimited());
        double pickupStart = CompetitionPathsTest.pickupStartTime(file, paths.get(leg));
        for (boolean red : new boolean[] {false, true}) {
          var tr = red ? paths.get(leg).flip() : paths.get(leg);
          for (double t = 0; t <= tr.getTotalTimeSeconds(); t += .01) {
            var state = tr.sample(t);
            String context = side + " max leg " + leg + " red=" + red + " t=" + t;
            CompetitionPathsTest.checkClearance(state.pose, context);
            var blue = red ? FlippingUtil.flipFieldPose(state.pose) : state.pose;
            CompetitionPathsTest.checkCrossingHeading(blue, context);
            assertTrue(blue.getX() - .45 * (Math.abs(blue.getRotation().getCos())
                + Math.abs(blue.getRotation().getSin())) <= 8.27, context + " wholly past centerline");
            if (leg % 2 == 0 && blue.getX() >= 4.02 && blue.getX() <= 5.24)
              assertEquals(side.equals("LEFT") ? (leg == 0 ? 7.5453 : 7.39)
                  : 8.07 - (leg == 0 ? 7.5453 : 7.39), blue.getY(), .08, context + " outbound trench");
            if (leg % 2 == 1 && CompetitionPathsTest.touchesBarrier(blue))
              assertTrue(state.linearVelocity <= AutoTiming.BUMP_SPEED_METERS_PER_SECOND + .02, context);
            if (t >= pickupStart && CompetitionPathsTest.pickupRegion(blue) && state.linearVelocity > .3)
              maxAngle = Math.max(maxAngle, CompetitionPathsTest.rearIntakeAngle(state.pose, state.fieldSpeeds));
            if (leg == 0 && !red) {
              if (!Double.isFinite(centerline) && blue.getX() >= 8.27) centerline = t;
              if (!Double.isFinite(activePickup) && t >= pickupStart && CompetitionPathsTest.intakeTouchesStartingFuel(blue)) activePickup = t;
            }
            samples.add(String.format(Locale.ROOT, "%s,%s,%.4f,%.6f,%.6f,%.3f,%.4f",
                "ECLIPSE_MAX_" + side + "_" + LEGS.get(leg), red ? "Red" : "Blue", t,
                state.pose.getX(), state.pose.getY(), state.pose.getRotation().getDegrees(), state.linearVelocity));
          }
        }
      }
      for (int i : new int[] {0, 2}) {
        var a = paths.get(i).getEndState();
        var b = paths.get(i + 1).getInitialState();
        assertEquals(0, a.pose.getTranslation().getDistance(b.pose.getTranslation()), 1e-8);
        assertEquals(0, a.pose.getRotation().minus(b.pose.getRotation()).getRadians(), 1e-8);
        assertEquals(a.linearVelocity, b.linearVelocity, 1e-8);
        // PathPlanner's synthetic terminal state stores a heading-based velocity,
        // not the incoming Bezier tangent. Validate the actual saved join geometry.
        var source = PathPlannerPath.fromPathFile("ECLIPSE_MAX_" + side + "_" + LEGS.get(i));
        var target = PathPlannerPath.fromPathFile("ECLIPSE_MAX_" + side + "_" + LEGS.get(i + 1));
        var end = source.getWaypoints().get(source.getWaypoints().size() - 1);
        var begin = target.getWaypoints().get(0);
        assertEquals(0, end.anchor().minus(end.prevControl()).getAngle().minus(
            begin.nextControl().minus(begin.anchor()).getAngle()).getRadians(), 1e-9);
      }
      report.add(String.format(Locale.ROOT, "%s,%.4f,%.4f,%.4f,%.4f,%.4f", side, first, second, centerline, activePickup, maxAngle));
      System.out.println("MAX_TIMING " + report.get(report.size() - 1));
      double angle = maxAngle;
      double arrival = centerline, pickup = activePickup;
      checks.add(() -> assertTrue(angle <= 10, "Rear-leading pickup planned error " + angle));
      checks.add(() -> assertTrue(arrival <= 1.30 && pickup <= 1.60, "Rush arrival/pickup " + arrival + "/" + pickup));
      checks.add(() -> assertTrue(second + 1.25 + .50 + .25 <= AutoTiming.MAX_AGGRESSION_SECOND_CYCLE_RESERVE_SECONDS,
          "New departure reserve does not cover drive, spinup, useful feed and scheduling margin"));
    }
    var out = Path.of("build/reports/auto-paths"); Files.createDirectories(out);
    Files.write(out.resolve("max-timing.csv"), report); Files.write(out.resolve("max-samples.csv"), samples);
    assertAll(checks);
  }

  @Test void largerFirstHarvestAndComplementarySecondTripBeatBaselineArea() throws Exception {
    var report = new ArrayList<String>(List.of("side,baselineFirstM2,maxFirstM2,baselineTotalM2,maxTotalM2,maxFreshSecondM2,maxPocketM2"));
    var checks = new ArrayList<org.junit.jupiter.api.function.Executable>();
    for (String side : List.of("LEFT", "RIGHT")) {
      var normal = coverage(side, false);
      var maximum = coverage(side, true);
      report.add(String.format(Locale.ROOT, "%s,%.4f,%.4f,%.4f,%.4f,%.4f,%.4f", side,
          normal[0], maximum[0], normal[1], maximum[1], maximum[2], maximum[3]));
      System.out.println("MAX_COVERAGE " + report.get(report.size() - 1));
      // Keep the 20-second/two-volley budget: the continuous-arc design trades
      // the slower S-turn draft's bigger first harvest for more fresh trip-two area.
      checks.add(() -> assertTrue(maximum[0] >= normal[0] * 1.10, "First harvest must improve without delaying the first unload"));
      checks.add(() -> assertTrue(maximum[1] >= normal[1] * 1.15, "New auto must increase total carpet, not just rename a speed change"));
      checks.add(() -> assertTrue(maximum[2] >= 1.20, "Second harvest must cover fresh carpet"));
      checks.add(() -> assertTrue(maximum[3] >= 1.20, "Must keep behind-hub collection"));
    }
    var out = Path.of("build/reports/auto-paths"); Files.createDirectories(out);
    Files.write(out.resolve("max-coverage.csv"), report);
    assertAll(checks);
  }

  @Test void trackAllNewLegsOnBothAlliancesWithSixtyAndOneHundredTwentyMsResponse() throws Exception {
    var checks = new ArrayList<org.junit.jupiter.api.function.Executable>();
    for (String side : List.of("LEFT", "RIGHT")) for (String leg : LEGS) {
      String name = "ECLIPSE_MAX_" + side + "_" + leg;
      var trajectory = PathPlannerPath.fromPathFile(name).getIdealTrajectory(config).orElseThrow();
      for (double lag : new double[] {.06, .12}) for (boolean red : new boolean[] {false, true})
        checks.add(() -> CompetitionPathsTest.followWithLag(name + (red ? " Red" : ""), red ? trajectory.flip() : trajectory, lag));
    }
    assertAll(checks);
  }

  @Test void executeBothCyclesAndAbortOrStayScoringWhenReadinessIsLate() throws Exception {
    SimHooks.pauseTiming();
    try {
      for (String side : List.of("left", "right")) for (boolean red : new boolean[] {false, true}) {
        String name = "eclipse_max_" + side + "_double";
        for (double[] readiness : new double[][] {{.55,.55,.35,.35},{.55,.55,.35,0},
            {.55,1.25,.35,.35},{1.25,.55,.35,.35},{.55,.55,1.05,.35},
            {.55,.55,1.80,.35},{.55,.55,.35,.60}})
          CompetitionAutoExecutionTest.execute(name, red, readiness[0], readiness[1], readiness[2], readiness[3]);
      }
    } finally { AutoBuilder.resetForTesting(); SimHooks.resumeTiming(); }
  }

  @Test void newBudgetRejectsInvalidTimeAndCannotChangeBaselineAdmission() {
    assertEquals(9.0, AutoTiming.SECOND_CYCLE_RESERVE_SECONDS);
    double cutoff = AutoTiming.DURATION_SECONDS - AutoTiming.MAX_AGGRESSION_SECOND_CYCLE_RESERVE_SECONDS;
    assertTrue(AutoTiming.canStartSecondCycle(cutoff, AutoTiming.MAX_AGGRESSION_SECOND_CYCLE_RESERVE_SECONDS));
    assertFalse(AutoTiming.canStartSecondCycle(cutoff + .001, AutoTiming.MAX_AGGRESSION_SECOND_CYCLE_RESERVE_SECONDS));
    for (double invalid : new double[] {Double.NaN, Double.POSITIVE_INFINITY, -1, 0, 20})
      assertFalse(AutoTiming.canStartSecondCycle(0, invalid));
    assertEquals(AutoTiming.canStartSecondCycle(cutoff, 9.0), AutoTiming.canStartSecondCycle(cutoff));
  }

  @Test void launchesTolerateAllModeledPlacementErrorSignsAndMirrorExactly() throws Exception {
    var checks = new ArrayList<org.junit.jupiter.api.function.Executable>();
    for (String side : List.of("LEFT", "RIGHT")) {
      String name = "ECLIPSE_MAX_" + side + "_START_TO_FUEL";
      var route = PathPlannerPath.fromPathFile(name).getIdealTrajectory(config).orElseThrow();
      for (boolean red : new boolean[] {false, true}) for (double dx : new double[] {-.05,.05})
        for (double dy : new double[] {-.05,.05}) for (double yaw : new double[] {-2,2})
          checks.add(() -> CompetitionPathsTest.followWithLag(name + (red ? " Red" : ""),
              red ? route.flip() : route, .12, dx, dy, yaw));
    }
    for (String leg : LEGS) {
      var a = PathPlannerPath.fromPathFile("ECLIPSE_MAX_LEFT_" + leg);
      var b = PathPlannerPath.fromPathFile("ECLIPSE_MAX_RIGHT_" + leg);
      assertEquals(a.getGlobalConstraints(), b.getGlobalConstraints());
      assertEquals(a.getConstraintZones(), b.getConstraintZones());
      assertEquals(a.getWaypoints().size(), b.getWaypoints().size());
      for (int i = 0; i < a.getWaypoints().size(); i++) {
        var aw = a.getWaypoints().get(i); var bw = b.getWaypoints().get(i);
        mirror(aw.anchor(), bw.anchor()); mirror(aw.prevControl(), bw.prevControl()); mirror(aw.nextControl(), bw.nextControl());
      }
      assertEquals(a.getRotationTargets().size(), b.getRotationTargets().size());
      for (int i = 0; i < a.getRotationTargets().size(); i++) {
        assertEquals(a.getRotationTargets().get(i).position(), b.getRotationTargets().get(i).position(), 1e-9);
        assertEquals(0, a.getRotationTargets().get(i).rotation().plus(b.getRotationTargets().get(i).rotation()).getRadians(), 1e-9);
      }
      assertEquals(a.getPointTowardsZones().size(), b.getPointTowardsZones().size());
      for (int i = 0; i < a.getPointTowardsZones().size(); i++) {
        var az = a.getPointTowardsZones().get(i); var bz = b.getPointTowardsZones().get(i);
        mirror(az.targetPosition(), bz.targetPosition());
        assertEquals(az.minPosition(), bz.minPosition()); assertEquals(az.maxPosition(), bz.maxPosition());
        assertEquals(0, az.rotationOffset().plus(bz.rotationOffset()).getRadians(), 1e-9);
      }
      assertEquals(0, a.getIdealStartingState().rotation().plus(b.getIdealStartingState().rotation()).getRadians(), 1e-9);
    }
    assertAll(checks);
  }

  private static void mirror(Translation2d a, Translation2d b) {
    if (a == null) { assertNull(b); return; }
    assertNotNull(b); assertEquals(a.getX(), b.getX(), 1e-9); assertEquals(8.07 - a.getY(), b.getY(), 1e-9);
  }

  private List<PathPlannerTrajectory> routes(String side, boolean maximum) throws Exception {
    var result = new ArrayList<PathPlannerTrajectory>();
    for (String leg : LEGS) result.add(PathPlannerPath.fromPathFile("ECLIPSE_" + (maximum ? "MAX_" : "")
        + side + "_" + (leg.equals("START_TO_FUEL") && !maximum ? "DOUBLE_START_TO_FUEL" : leg))
        .getIdealTrajectory(config).orElseThrow());
    return result;
  }

  private double[] coverage(String side, boolean maximum) throws Exception {
    var paths = routes(side, maximum);
    String prefix = "ECLIPSE_" + (maximum ? "MAX_" : "") + side + "_";
    var one = FuelCoverageTest.samples(paths.subList(0, 2), CompetitionPathsTest.pickupStartTime(
        PathPlannerPath.fromPathFile(prefix + (maximum ? "START_TO_FUEL" : "DOUBLE_START_TO_FUEL")), paths.get(0)));
    var two = FuelCoverageTest.samples(paths.subList(2, 4), CompetitionPathsTest.pickupStartTime(
        PathPlannerPath.fromPathFile(prefix + "SCORE_TO_FUEL2"), paths.get(2)));
    double[] area = new double[4];
    for (int region = 0; region < 2; region++) {
      int nx = region == 0 ? 46 : 40, ny = region == 0 ? 116 : 50;
      for (int ix = 0; ix < nx; ix++) for (int iy = 0; iy < ny; iy++) {
        var blue = new Translation2d((region == 0 ? 7.36 : 5.55) + ix * .04,
            (region == 0 ? 1.72 : 3.05) + iy * .04);
        var fuel = side.equals("RIGHT") ? new Translation2d(blue.getX(), 8.07 - blue.getY()) : blue;
        boolean a = FuelCoverageTest.covers(one, fuel), b = FuelCoverageTest.covers(two, fuel);
        if (region == 0) { if (a) area[0] += .0016; if (a || b) area[1] += .0016; if (b && !a) area[2] += .0016; }
        else if (a || b) area[3] += .0016;
      }
    }
    return area;
  }
}
