package frc.robot.auto;

import static org.junit.jupiter.api.Assertions.*;

import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.commands.PathPlannerAuto;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.trajectory.PathPlannerTrajectory;
import com.pathplanner.lib.util.FlippingUtil;
import edu.wpi.first.hal.HAL;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.robot.subsystems.shooter.ShooterConstants;
import frc.robot.subsystems.drive.TunerConstants;
import frc.robot.subsystems.power.DriveCurrentPolicy;
import edu.wpi.first.math.system.plant.DCMotor;
import java.nio.file.Files;
import java.nio.file.Path;
import java.util.ArrayList;
import java.util.List;
import java.util.Locale;
import org.junit.jupiter.api.BeforeAll;
import org.junit.jupiter.api.Test;

/** Uses the deployed files and installed PathPlanner generator; no desktop GUI or robot required. */
class CompetitionPathsTest {
  private static final double FIELD_X = 16.54;
  private static final double FIELD_Y = 8.07;
  // Intake is on -X/rear, not both ends of the bumper.
  private static final double HALF_LENGTH = RobotFootprint.REAR_REACH;
  private static final String[] LEGS = {
    "START_TO_FUEL", "COLLECT1_TO_SCORE", "SCORE_TO_FUEL2", "COLLECT2_TO_SCORE"
  };
  private static RobotConfig config;

  @BeforeAll
  static void initialize() throws Exception {
    assertTrue(HAL.initialize(500, 0));
    config = RobotConfig.fromGUISettings();
    NamedCommands.registerCommand("intake run", Commands.none());
    assertEquals(FIELD_X, FlippingUtil.fieldSizeX, 0.001);
    assertEquals(FIELD_Y, FlippingUtil.fieldSizeY, 0.001);
    assertEquals(FlippingUtil.FieldSymmetry.kRotational, FlippingUtil.symmetryType);
    assertEquals(DriveCurrentPolicy.AUTO_SUPPLY_AMPS, config.moduleConfig.driveCurrentLimit, 1e-9);
    assertEquals(TunerConstants.kDriveGearRatio,
        config.moduleConfig.driveMotor.KtNMPerAmp / DCMotor.getKrakenX60(1).KtNMPerAmp, 1e-9,
        "Planner and hardware gearing must agree");
  }

  @Test
  void reportRouteSpeedAndSegmentTiming() throws Exception {
    List<String> rows = new ArrayList<>();
    rows.add("path,seconds,lengthM,peakMps,movingTimeBelow1Mps");
    List<String> segments = new ArrayList<>();
    segments.add("path,segment,beginSeconds,endSeconds");
    for (String side : List.of("LEFT", "RIGHT")) {
      for (String leg : List.of("START_TO_FUEL", "DOUBLE_START_TO_FUEL", "COLLECT1_TO_SCORE",
          "SCORE_TO_FUEL2", "COLLECT2_TO_SCORE", "SINGLE_COLLECT_TO_SCORE")) {
        String name = "ECLIPSE_" + side + "_" + leg;
        var path = PathPlannerPath.fromPathFile(name);
        var trajectory = path.getIdealTrajectory(config).orElseThrow();
        double length = 0, peak = 0, slow = 0;
        for (int i = 1; i < path.numPoints(); i++)
          length += path.getPoint(i).position.getDistance(path.getPoint(i - 1).position);
        for (double t = 0; t < trajectory.getTotalTimeSeconds(); t += .01) {
          double speed = trajectory.sample(t).linearVelocity;
          peak = Math.max(peak, speed);
          if (speed > .15 && speed < 1) slow += .01;
        }
        rows.add(String.format(Locale.ROOT, "%s,%.3f,%.3f,%.3f,%.3f", name,
            trajectory.getTotalTimeSeconds(), length, peak, slow));
        // Performance regressions check both file roles AND the complete-cycle
        // budgets below: moving pickup work to another file is not a speedup.
        if (leg.equals("SCORE_TO_FUEL2")) assertTrue(trajectory.getTotalTimeSeconds() < 2.95, name);
        if (leg.equals("COLLECT1_TO_SCORE")) assertTrue(trajectory.getTotalTimeSeconds() < 1.6, name);
        if (leg.equals("SINGLE_COLLECT_TO_SCORE")) assertTrue(trajectory.getTotalTimeSeconds() < 3.15, name);
        int segment = 0;
        double begin = 0;
        for (int i = 0; i < trajectory.getStates().size(); i++) {
          var state = trajectory.getState(i);
          if (path.getPoint(i).waypointRelativePos >= segment + 1) {
            segments.add(String.format(Locale.ROOT, "%s,%d,%.3f,%.3f", name, segment, begin, state.timeSeconds));
            begin = state.timeSeconds;
            segment++;
          }
        }
        System.out.println(rows.get(rows.size() - 1));
      }
    }
    var dir = Path.of("build", "reports", "auto-paths");
    Files.createDirectories(dir);
    Files.write(dir.resolve("route-speed.csv"), rows);
    Files.write(dir.resolve("segment-timing.csv"), segments);
  }

  @Test
  void outboundUsesTrenchAndRearIntakeLeadsPickup() throws Exception {
    List<String> report = new ArrayList<>();
    report.add("path,alliance,maxPlannedPickupAngleDegrees");
    List<String> points = new ArrayList<>();
    points.add("path,position,x,y,headingDeg");
    for (String side : List.of("LEFT", "RIGHT")) {
      for (String leg : List.of("START_TO_FUEL", "DOUBLE_START_TO_FUEL", "COLLECT1_TO_SCORE",
          "SCORE_TO_FUEL2", "COLLECT2_TO_SCORE", "SINGLE_COLLECT_TO_SCORE")) {
        String name = "ECLIPSE_" + side + "_" + leg;
        var path = PathPlannerPath.fromPathFile(name);
        for (int i = 0; i < path.numPoints() - 1; i++) {
          var a = path.getPoint(i);
          var b = path.getPoint(i + 1);
          points.add(String.format(Locale.ROOT, "%s,%.12f,%.12f,%.12f,%.12f", name,
              a.waypointRelativePos, a.position.getX(), a.position.getY(),
              b.position.minus(a.position).getAngle().getDegrees()));
        }
        var original = path.getIdealTrajectory(config).orElseThrow();
        double pickupStart = pickupStartTime(path, original);
        for (boolean red : new boolean[] {false, true}) {
          var trajectory = red ? original.flip() : original;
          double maxAngle = 0;
          boolean crossed = false;
          for (double t = 0; t < trajectory.getTotalTimeSeconds(); t += .01) {
            var state = trajectory.sample(t);
            var blue = red ? FlippingUtil.flipFieldPose(state.pose) : state.pose;
            if (blue.getX() >= 4.02 && blue.getX() <= 5.24) {
              crossed = true;
              if (leg.endsWith("START_TO_FUEL") || leg.equals("SCORE_TO_FUEL2")) {
                double trenchY = leg.endsWith("START_TO_FUEL") ? 7.5453 : 7.39;
                assertEquals(side.equals("LEFT") ? trenchY : FIELD_Y - trenchY, blue.getY(), .08,
                    name + " outbound must use trench, not bump");
              } else {
                assertEquals(side.equals("LEFT") ? 5.55 : 2.52, blue.getY(), .08,
                    name + " loaded return must use bump");
              }
            }
            if (t >= pickupStart && pickupRegion(blue) && state.linearVelocity > .3) {
              maxAngle = Math.max(maxAngle, rearIntakeAngle(state.pose, state.fieldSpeeds));
            }
          }
          assertTrue(crossed, name + " never crossed the barrier");
          assertTrue(maxAngle <= 10, name + " rear intake is not aligned with planned pickup motion: " + maxAngle);
          System.out.printf(Locale.ROOT, "%s %s planned pickup angle %.2fdeg%n", name, red ? "Red" : "Blue", maxAngle);
          report.add(String.format(Locale.ROOT, "%s,%s,%.3f", name, red ? "Red" : "Blue", maxAngle));
        }
      }
    }
    var dir = Path.of("build", "reports", "auto-paths");
    Files.createDirectories(dir);
    Files.write(dir.resolve("pickup-alignment.csv"), report);
    Files.write(dir.resolve("path-points.csv"), points);
  }

  @Test
  void emptyLaunchUsesRequestedAccelerationWithoutRemovingPhysicalLimits() throws Exception {
    List<String> report = new ArrayList<>();
    report.add("path,requestedLaunchAccelerationMps2,generatedStraightPeakAccelerationMps2,generatedStraightPeakSpeedMps");
    for (String side : List.of("LEFT", "RIGHT")) {
      for (String leg : List.of("START_TO_FUEL", "DOUBLE_START_TO_FUEL")) {
        String name = "ECLIPSE_" + side + "_" + leg;
        var path = PathPlannerPath.fromPathFile(name);
        double launchAcceleration = leg.equals("DOUBLE_START_TO_FUEL") ? 10.2 : 9.6;
        assertEquals(launchAcceleration, path.getGlobalConstraints().maxAccelerationMPSSq(), 1e-9, name);
        assertFalse(path.getGlobalConstraints().unlimited(), name);
        assertEquals(launchAcceleration, path.getPoint(0).constraints.maxAccelerationMPSSq(), 1e-9, name);
        for (var zone : path.getConstraintZones()) {
          assertTrue(zone.constraints().maxAccelerationMPSSq() <= launchAcceleration, name);
          assertFalse(zone.constraints().unlimited(), name);
        }
        // Measure what the installed generator actually requests, not merely the
        // acceleration ceiling in JSON. Do not raise simulated motor power to match it.
        var trajectory = path.getIdealTrajectory(config).orElseThrow();
        var previous = trajectory.sample(0);
        double peakAcceleration = 0, peakSpeed = 0;
        for (double t = 0.005; t < trajectory.getTotalTimeSeconds(); t += 0.005) {
          var state = trajectory.sample(t);
          if (state.pose.getX() > 6.15) break;
          peakAcceleration = Math.max(peakAcceleration, Math.hypot(
              state.fieldSpeeds.vxMetersPerSecond - previous.fieldSpeeds.vxMetersPerSecond,
              state.fieldSpeeds.vyMetersPerSecond - previous.fieldSpeeds.vyMetersPerSecond) / 0.005);
          peakSpeed = Math.max(peakSpeed, state.linearVelocity);
          previous = state;
        }
        assertTrue(Double.isFinite(peakAcceleration) && peakAcceleration > 0, name);
        assertTrue(peakAcceleration < launchAcceleration * 1.15,
            name + " has an acceleration spike, not a smooth launch: " + peakAcceleration);
        report.add(String.format(Locale.ROOT, "%s,%.3f,%.3f,%.3f",
            name, launchAcceleration, peakAcceleration, peakSpeed));
        System.out.printf(Locale.ROOT, "%s launch: %.1f m/s2 ceiling, %.3f m/s2 generated straight peak%n",
            name, launchAcceleration, peakAcceleration);
      }
    }
    var dir = Path.of("build", "reports", "auto-paths");
    Files.createDirectories(dir);
    Files.write(dir.resolve("launch-envelope.csv"), report);
  }

  @Test
  void openingRushCommitsToOneTurnWithoutHeadingBacktrackingOrCreeping() throws Exception {
    List<String> report = new ArrayList<>();
    report.add("path,totalYawDegrees,reverseYawDegrees,reverseTravelTurnDegrees,minRushSpeedMps");
    for (String side : List.of("LEFT", "RIGHT")) {
      double turnSign = side.equals("LEFT") ? -1 : 1;
      for (String leg : List.of("START_TO_FUEL", "DOUBLE_START_TO_FUEL")) {
        String name = "ECLIPSE_" + side + "_" + leg;
        var trajectory = PathPlannerPath.fromPathFile(name).getIdealTrajectory(config).orElseThrow();
        var previous = trajectory.sample(0);
        double totalYaw = 0, reverseYaw = 0, reverseTravel = 0;
        double minRushSpeed = Double.POSITIVE_INFINITY;
        Rotation2d previousTravel = Rotation2d.kZero;
        boolean crossedCenterline = false;
        for (double t = .01; t <= trajectory.getTotalTimeSeconds(); t += .01) {
          var state = trajectory.sample(t);
          double yawDelta = state.pose.getRotation().minus(previous.pose.getRotation()).getRadians();
          totalYaw += Math.abs(yawDelta);
          reverseYaw += Math.max(0, -turnSign * yawDelta);
          if (state.linearVelocity > .3) {
            var travel = new Rotation2d(state.fieldSpeeds.vxMetersPerSecond, state.fieldSpeeds.vyMetersPerSecond);
            reverseTravel += Math.max(0, -turnSign * travel.minus(previousTravel).getRadians());
            previousTravel = travel;
          }
          if (!crossedCenterline && state.pose.getX() >= 6.1)
            minRushSpeed = Math.min(minRushSpeed, state.linearVelocity);
          if (state.pose.getX() >= FIELD_X / 2) crossedCenterline = true;
          previous = state;
        }
        report.add(String.format(Locale.ROOT, "%s,%.3f,%.3f,%.3f,%.3f", name,
            Math.toDegrees(totalYaw), Math.toDegrees(reverseYaw), Math.toDegrees(reverseTravel), minRushSpeed));
        assertTrue(crossedCenterline, name + " must actually reach centerline");
        assertTrue(totalYaw <= Math.toRadians(95), name + " wastes rotation before the pickup turn");
        assertTrue(reverseYaw <= Math.toRadians(2), name + " turns the chassis back the other way");
        assertTrue(reverseTravel <= Math.toRadians(2), name + " has a counter-steering wiggle");
        assertTrue(minRushSpeed >= 2.0, name + " slows to a crawl before centerline: " + minRushSpeed);
      }
    }
    var reports = Path.of("build", "reports", "auto-paths");
    Files.createDirectories(reports);
    Files.write(reports.resolve("rush-profile.csv"), report);
  }

  @Test
  void generateAndValidateActualDeployedTrajectories() throws Exception {
    List<String> samples = new ArrayList<>();
    samples.add("path,alliance,t,x,y,headingDeg,speed");
    List<String> timing = new ArrayList<>();
    List<String> arrivals = new ArrayList<>();
    arrivals.add("path,firstFuelContactDriveSeconds,rearIntakeCenterlineDriveSeconds,robotCenterlineDriveSeconds,firstActivePickupDriveSeconds");
    timing.add("side,singleDriveSeconds,doubleFirstDriveSeconds,secondCycleDriveSeconds,totalDoubleDriveSeconds,secondCycleWorstCaseSeconds");
    for (String side : List.of("LEFT", "RIGHT")) {
      double[] times = new double[LEGS.length];
      for (int leg = 0; leg < LEGS.length; leg++) {
        String name = "ECLIPSE_" + side + "_" + LEGS[leg];
        var path = PathPlannerPath.fromPathFile(name);
        var trajectory = path.getIdealTrajectory(config).orElseThrow();
        times[leg] = trajectory.getTotalTimeSeconds();
        assertTrue(Double.isFinite(times[leg]) && times[leg] > 0.1, name);
        assertEquals(leg == 0 ? 1.8 : leg == 2 ? 2.8 : 0,
            path.getGoalEndState().velocityMPS(), 1e-9, name);
        assertEquals(leg == 1 ? 2.5 : leg == 3 ? 2.8 : 0,
            path.getIdealStartingState().velocityMPS(), 1e-9, name);
        validateSamples(name, "Blue", trajectory, samples);
        validateSamples(name, "Red", trajectory.flip(), samples);
        if (leg == 0) arrivals.add(centerlineArrival(name, path, trajectory));
      }
      var singleReturnPath = PathPlannerPath.fromPathFile("ECLIPSE_" + side + "_SINGLE_COLLECT_TO_SCORE");
      var singleReturn = singleReturnPath.getIdealTrajectory(config).orElseThrow();
      validateSamples("ECLIPSE_" + side + "_SINGLE_COLLECT_TO_SCORE", "Blue", singleReturn, samples);
      validateSamples("ECLIPSE_" + side + "_SINGLE_COLLECT_TO_SCORE", "Red", singleReturn.flip(), samples);
      double first = times[0] + singleReturn.getTotalTimeSeconds();
      double second = times[2] + times[3];
      double secondWorstCase = second
          + ShooterConstants.kShotSpinupTimeoutSeconds + AutoTiming.MIN_FINAL_FEED_SECONDS + 0.25;
      var doubleFirstPath = PathPlannerPath.fromPathFile("ECLIPSE_" + side + "_DOUBLE_START_TO_FUEL");
      var doubleFirst = doubleFirstPath.getIdealTrajectory(config).orElseThrow();
      double doubleFirstTime = doubleFirst.getTotalTimeSeconds() + times[1];
      assertTrue(first < 6.95, "Complete single collection/return regressed: " + first);
      assertTrue(doubleFirstTime < 3.75, "First complete collection/return regressed: " + doubleFirstTime);
      assertTrue(second < 6.7, "Complete second collection/return regressed: " + second);
      assertEquals(2.5, doubleFirstPath.getGoalEndState().velocityMPS(), 1e-9);
      var returnPath = PathPlannerPath.fromPathFile("ECLIPSE_" + side + "_COLLECT1_TO_SCORE");
      assertEquals(0, doubleFirst.getEndState().pose.getTranslation().getDistance(
          returnPath.getIdealTrajectory(config).orElseThrow().getInitialState().pose.getTranslation()), 1e-9);
      var pocketReturn = PathPlannerPath.fromPathFile("ECLIPSE_" + side + "_COLLECT2_TO_SCORE")
          .getIdealTrajectory(config).orElseThrow();
      for (String entry : List.of("START_TO_FUEL", "SCORE_TO_FUEL2")) {
        var matchingReturn = entry.equals("START_TO_FUEL") ? singleReturn : pocketReturn;
        var entryPath = PathPlannerPath.fromPathFile("ECLIPSE_" + side + "_" + entry);
        var end = entryPath.getIdealTrajectory(config).orElseThrow().getEndState();
        var waypoints = entryPath.getWaypoints();
        assertEquals(0, waypoints.get(waypoints.size() - 1).anchor().getDistance(
            matchingReturn.getInitialState().pose.getTranslation()), 1e-9, entry + " exact saved join");
        // Exact saved anchors and rotations must join; discretized samples must
        // also stay within 2 cm of that rolling handoff.
        assertEquals(0, end.pose.getTranslation().getDistance(
            matchingReturn.getInitialState().pose.getTranslation()), 0.02, entry + " sampled pocket join");
        assertEquals(0, end.pose.getRotation().minus(
            matchingReturn.getInitialState().pose.getRotation()).getDegrees(), 1e-9, entry + " heading join");
      }
      validateSamples("ECLIPSE_" + side + "_DOUBLE_START_TO_FUEL", "Blue", doubleFirst, samples);
      validateSamples("ECLIPSE_" + side + "_DOUBLE_START_TO_FUEL", "Red", doubleFirst.flip(), samples);
      arrivals.add(centerlineArrival("ECLIPSE_" + side + "_DOUBLE_START_TO_FUEL", doubleFirstPath, doubleFirst));
      System.out.printf(Locale.ROOT, "%s: first drive %.3fs; second drive %.3fs; required reserve %.3fs%n",
          side, first, second, secondWorstCase);
      System.out.printf(Locale.ROOT, "%s: double first drive %.3fs%n", side, doubleFirstTime);
      assertTrue(secondWorstCase <= AutoTiming.SECOND_CYCLE_RESERVE_SECONDS,
          "Post-preparation reserve must include drive, shot spin-up, feed, and scheduler margin: " + secondWorstCase);
      timing.add(String.format(Locale.ROOT, "%s,%.3f,%.3f,%.3f,%.3f,%.3f",
          side, first, doubleFirstTime, second, doubleFirstTime + second, secondWorstCase));
    }
    Path reports = Path.of("build", "reports", "auto-paths");
    Files.createDirectories(reports);
    Files.write(reports.resolve("samples.csv"), samples);
    Files.write(reports.resolve("timing.csv"), timing);
    Files.write(reports.resolve("centerline.csv"), arrivals);
  }

  private static String centerlineArrival(String name, PathPlannerPath path, PathPlannerTrajectory trajectory) {
    double intake = Double.NaN, center = Double.NaN, firstFuel = Double.NaN;
    double activePickup = Double.NaN;
    double pickupStart = pickupStartTime(path, trajectory);
    for (double t = 0; t <= trajectory.getTotalTimeSeconds(); t += 0.005) {
      var pose = trajectory.sample(t).pose;
      if (!Double.isFinite(firstFuel) && intakeTouchesStartingFuel(pose)) firstFuel = t;
      if (t >= pickupStart && !Double.isFinite(activePickup) && intakeTouchesStartingFuel(pose)) activePickup = t;
      if (!Double.isFinite(intake) && pose.getX() - HALF_LENGTH * pose.getRotation().getCos() >= FIELD_X / 2)
        intake = t;
      if (!Double.isFinite(center) && pose.getX() >= FIELD_X / 2) center = t;
    }
    assertTrue(Double.isFinite(intake) && Double.isFinite(center), name + " does not reach centerline");
    assertTrue(Double.isFinite(firstFuel), name + " misses starting fuel");
    System.out.printf(Locale.ROOT, "%s first fuel %.3fs; intake centerline %.3fs; robot centerline %.3fs%n", name, firstFuel, intake, center);
    // Direct centerline rush replaces the old edge-fuel-first S bend. Check BOTH
    // first pickup and robot-center arrival: a straight corridor alone is not a pickup.
    // Sideways starts trade the old rear-tip X lead for earlier ROBOT arrival.
    // Keep geometric contact and roller-enabled pickup separate in the report.
    assertTrue(firstFuel <= 1.40, name + " delayed first fuel contact: " + firstFuel);
    assertTrue(center <= 1.45, name + " no longer rushes the robot center to midfield: " + center);
    assertTrue(Double.isFinite(activePickup), name + " never enables pickup on starting carpet");
    System.out.printf(Locale.ROOT, "  first active pickup %.3fs (roller marker %.3fs)%n", activePickup, pickupStart);
    return String.format(Locale.ROOT, "%s,%.3f,%.3f,%.3f,%.3f", name, firstFuel, intake, center, activePickup);
  }

  static boolean intakeTouchesStartingFuel(Pose2d pose) {
    double cos = pose.getRotation().getCos(), sin = pose.getRotation().getSin();
    for (double lateral = -0.4; lateral <= 0.4001; lateral += 0.05) {
      double x = pose.getX() - HALF_LENGTH * cos - lateral * sin;
      double y = pose.getY() - HALF_LENGTH * sin + lateral * cos;
      if (x >= 7.36 && x <= 9.20 && y >= 1.72 && y <= 6.35) return true;
    }
    return false;
  }

  @Test
  void parseCompetitionAutosWithRegisteredCommandsAndValidateStartingPoses() throws Exception {
    AutoBuilder.resetForTesting();
    Subsystem drive = new Subsystem() {};
    AutoBuilder.configure(() -> Pose2d.kZero, pose -> {}, ChassisSpeeds::new,
        (speeds, ff) -> {}, new PPHolonomicDriveController(
            new PIDConstants(10, 0, 0), new PIDConstants(7, 0, 0)), config, () -> false, drive);
    for (String name : List.of("auto start", "auto start left", "auto start right", "auto travel", "auto cycle 2 left", "auto cycle 2 right", "intake deploy",
        "intake retract", "intake run", "intake stop", "stopshoot", "zerodrive", "auto collect settle")) {
      NamedCommands.registerCommand(name, Commands.none());
    }
    NamedCommands.registerCommand("shoot", Commands.waitSeconds(
        ShooterConstants.kShotSpinupTimeoutSeconds + AutoTiming.DUMP_FEED_SECONDS));
    NamedCommands.registerCommand("dump", Commands.waitSeconds(
        ShooterConstants.kShotSpinupTimeoutSeconds + AutoTiming.DUMP_FEED_SECONDS));
    NamedCommands.registerCommand("auto finish shoot", Commands.waitSeconds(AutoTiming.DURATION_SECONDS));
    for (String side : List.of("left", "right")) {
      for (String suffix : List.of("", "_double")) {
        String name = "2910_trench_" + side + suffix;
        var auto = new PathPlannerAuto(name);
        assertNotNull(auto.getStartingPose(), name);
        assertEquals(4.4, auto.getStartingPose().getX(), 1e-6, name);
        assertEquals(side.equals("left") ? 7.5453 : 8.07 - 7.5453, auto.getStartingPose().getY(), 1e-6, name);
        assertEquals(side.equals("left") ? 90 : -90, auto.getStartingPose().getRotation().getDegrees(), 1e-6, name);
        assertTrue(auto.getRequirements().contains(drive), name);
        var paths = PathPlannerAuto.getPathGroupFromAutoFile(name);
        assertEquals(suffix.isEmpty() ? 2 : 4, paths.size(), name);
      }
    }
    double cutoff = AutoTiming.DURATION_SECONDS - AutoTiming.SECOND_CYCLE_RESERVE_SECONDS;
    assertTrue(AutoTiming.canStartSecondCycle(cutoff - 0.01));
    assertFalse(AutoTiming.canStartSecondCycle(cutoff + 0.01));
    assertFalse(AutoTiming.canStartSecondCycle(Double.NaN));
    AutoBuilder.resetForTesting();
  }

  @Test
  void rightPathsMirrorLeftAndMovingJoinsAreContinuous() throws Exception {
    for (String leg : List.of("START_TO_FUEL", "DOUBLE_START_TO_FUEL", "COLLECT1_TO_SCORE",
        "SCORE_TO_FUEL2", "COLLECT2_TO_SCORE", "SINGLE_COLLECT_TO_SCORE")) {
      var left = PathPlannerPath.fromPathFile("ECLIPSE_LEFT_" + leg);
      var right = PathPlannerPath.fromPathFile("ECLIPSE_RIGHT_" + leg);
      assertEquals(left.getGlobalConstraints(), right.getGlobalConstraints(), leg);
      assertEquals(left.getConstraintZones(), right.getConstraintZones(), leg);
      assertEquals(left.getWaypoints().size(), right.getWaypoints().size(), leg);
      for (int i = 0; i < left.getWaypoints().size(); i++) {
        var a = left.getWaypoints().get(i);
        var b = right.getWaypoints().get(i);
        assertYReflection(a.anchor(), b.anchor(), leg);
        assertYReflection(a.prevControl(), b.prevControl(), leg);
        assertYReflection(a.nextControl(), b.nextControl(), leg);
      }
      assertEquals(left.getRotationTargets().size(), right.getRotationTargets().size(), leg);
      assertEquals(left.getPointTowardsZones().size(), right.getPointTowardsZones().size(), leg);
      for (int i = 0; i < left.getPointTowardsZones().size(); i++) {
        var a = left.getPointTowardsZones().get(i);
        var b = right.getPointTowardsZones().get(i);
        assertYReflection(a.targetPosition(), b.targetPosition(), leg + " pickup arc center");
        assertEquals(a.minPosition(), b.minPosition(), 1e-9);
        assertEquals(a.maxPosition(), b.maxPosition(), 1e-9);
        assertEquals(0, a.rotationOffset().plus(b.rotationOffset()).getRadians(), 1e-9);
      }
      for (int i = 0; i < left.getRotationTargets().size(); i++) {
        var a = left.getRotationTargets().get(i);
        var b = right.getRotationTargets().get(i);
        assertEquals(a.position(), b.position(), 1e-9);
        assertEquals(0, a.rotation().plus(b.rotation()).getRadians(), 1e-9);
      }
      assertEquals(left.getIdealStartingState().velocityMPS(),
          right.getIdealStartingState().velocityMPS(), 1e-9);
      assertEquals(0, left.getIdealStartingState().rotation().plus(
          right.getIdealStartingState().rotation()).getRadians(), 1e-9);
    }
    for (String side : List.of("LEFT", "RIGHT")) {
      for (String[] pair : List.of(
          new String[] {"START_TO_FUEL", "SINGLE_COLLECT_TO_SCORE"},
          new String[] {"DOUBLE_START_TO_FUEL", "COLLECT1_TO_SCORE"},
          new String[] {"SCORE_TO_FUEL2", "COLLECT2_TO_SCORE"})) {
        var a = PathPlannerPath.fromPathFile("ECLIPSE_" + side + "_" + pair[0]);
        var b = PathPlannerPath.fromPathFile("ECLIPSE_" + side + "_" + pair[1]);
        var end = a.getWaypoints().get(a.getWaypoints().size() - 1);
        var start = b.getWaypoints().get(0);
        assertEquals(0, end.anchor().getDistance(start.anchor()), 1e-9, pair[0]);
        assertEquals(a.getGoalEndState().velocityMPS(), b.getIdealStartingState().velocityMPS(), 1e-9);
        assertEquals(0, a.getGoalEndState().rotation().minus(
            b.getIdealStartingState().rotation()).getRadians(), 1e-9);
        assertEquals(0, end.anchor().minus(end.prevControl()).getAngle().minus(
            start.nextControl().minus(start.anchor()).getAngle()).getRadians(), 1e-9,
            "A moving join cannot abruptly change travel direction");
      }
    }
  }

  private static void assertYReflection(edu.wpi.first.math.geometry.Translation2d left,
      edu.wpi.first.math.geometry.Translation2d right, String context) {
    if (left == null) { assertNull(right, context); return; }
    assertNotNull(right, context);
    assertEquals(left.getX(), right.getX(), 1e-9, context);
    assertEquals(FIELD_Y - left.getY(), right.getY(), 1e-9, context);
  }

  @Test
  void followRoutesWithoutVisionWithPlacementErrorAndDriveLag() throws Exception {
    List<org.junit.jupiter.api.function.Executable> cases = new ArrayList<>();
    for (String side : List.of("LEFT", "RIGHT")) {
      for (String leg : List.of("START_TO_FUEL", "DOUBLE_START_TO_FUEL", "COLLECT1_TO_SCORE",
          "SCORE_TO_FUEL2", "COLLECT2_TO_SCORE", "SINGLE_COLLECT_TO_SCORE")) {
        String name = "ECLIPSE_" + side + "_" + leg;
        var trajectory = PathPlannerPath.fromPathFile(name).getIdealTrajectory(config).orElseThrow();
        for (double lag : new double[] {0.06, 0.12}) {
          cases.add(() -> followWithLag(name, trajectory, lag));
          cases.add(() -> followWithLag(name + " Red", trajectory.flip(), lag));
        }
      }
    }
    assertAll("All deployed routes with both alliances and response delays", cases);
  }

  private static void validateSamples(String name, String alliance,
      PathPlannerTrajectory trajectory, List<String> samples) {
    double duration = trajectory.getTotalTimeSeconds();
    for (double t = 0; t <= duration + 0.02; t += 0.02) {
      var state = trajectory.sample(Math.min(t, duration));
      String context = name + " " + alliance + " t=" + t;
      assertTrue(Double.isFinite(state.pose.getX()) && Double.isFinite(state.pose.getY())
          && Double.isFinite(state.fieldSpeeds.omegaRadiansPerSecond), context);
      checkClearance(state.pose, context);
      Pose2d bluePose = alliance.equals("Red") ? FlippingUtil.flipFieldPose(state.pose) : state.pose;
      double bumperExtentX = 0.45 * (Math.abs(bluePose.getRotation().getCos())
          + Math.abs(bluePose.getRotation().getSin()));
      assertTrue(bluePose.getX() - bumperExtentX <= FIELD_X / 2,
          context + " bumpers completely across the center line");
      if (touchesBarrier(bluePose)) {
        double angle = crossingAxisError(bluePose);
        assertTrue(angle < Math.toRadians(4), context + " must cross straight");
        if (name.contains("COLLECT")) {
          assertTrue(state.linearVelocity <= AutoTiming.BUMP_SPEED_METERS_PER_SECOND + 0.02,
              context + " loaded bump crossing too fast: " + state.linearVelocity);
        }
      }
      samples.add(String.format(Locale.ROOT, "%s,%s,%.3f,%.5f,%.5f,%.3f,%.4f",
          name, alliance, t, state.pose.getX(), state.pose.getY(),
          state.pose.getRotation().getDegrees(), state.linearVelocity));
    }
  }

  static void followWithLag(String name, PathPlannerTrajectory trajectory, double lag) throws Exception {
    followWithLag(name, trajectory, lag, .04, -.03, 2);
  }

  static void followWithLag(String name, PathPlannerTrajectory trajectory, double lag,
      double placementX, double placementY, double placementYaw) throws Exception {
    double pickupStart = pickupStartTime(PathPlannerPath.fromPathFile(name.replace(" Red", "")), trajectory);
    var controller = new PPHolonomicDriveController(new PIDConstants(10, 0, 0), new PIDConstants(7, 0, 0));
    Pose2d pose = trajectory.getInitialState().pose;
    // Small placement error is intentional. No vision measurements are supplied.
    pose = new Pose2d(pose.getX() + placementX, pose.getY() + placementY,
        pose.getRotation().plus(Rotation2d.fromDegrees(placementYaw)));
    ChassisSpeeds actual = ChassisSpeeds.fromFieldRelativeSpeeds(
        trajectory.getInitialState().fieldSpeeds, pose.getRotation());
    var initial = trajectory.getInitialState().fieldSpeeds;
    ChassisSpeeds fieldActual = new ChassisSpeeds(initial.vxMetersPerSecond,
        initial.vyMetersPerSecond, initial.omegaRadiansPerSecond);
    controller.reset(pose, actual);
    double duration = trajectory.getTotalTimeSeconds();
    double maxError = 0;
    double maxPickupAngle = 0;
    double maxBumpSpeed = 0;
    String angleContext = "";
    double worstTime = 0;
    for (double t = 0; t <= duration; t += 0.02) {
      var target = trajectory.sample(t);
      var demand = controller.calculateRobotRelativeSpeeds(pose, target);
      double scale = Math.min(1, 5.0 / Math.max(0.001,
          Math.hypot(demand.vxMetersPerSecond, demand.vyMetersPerSecond)));
      demand.vxMetersPerSecond *= scale;
      demand.vyMetersPerSecond *= scale;
      // Translational inertia is field-relative: turning the chassis does not instantly
      // rotate its existing momentum. Convert demands and feedback at every step.
      var fieldDemand = ChassisSpeeds.fromRobotRelativeSpeeds(demand, pose.getRotation());
      double dx = (fieldDemand.vxMetersPerSecond - fieldActual.vxMetersPerSecond) * 0.02 / lag;
      double dy = (fieldDemand.vyMetersPerSecond - fieldActual.vyMetersPerSecond) * 0.02 / lag;
      boolean braking = dx * fieldActual.vxMetersPerSecond + dy * fieldActual.vyMetersPerSecond < 0;
      double accelLimit = DrivePhysicsModel.accelerationLimit(config, actual, braking, 10.5);
      double accelScale = Math.min(1, accelLimit * 0.02 / Math.max(0.001, Math.hypot(dx, dy)));
      fieldActual.vxMetersPerSecond += dx * accelScale;
      fieldActual.vyMetersPerSecond += dy * accelScale;
      double omega = MathUtil.clamp(demand.omegaRadiansPerSecond, -Math.toRadians(360), Math.toRadians(360));
      fieldActual.omegaRadiansPerSecond += MathUtil.clamp(
          (omega - fieldActual.omegaRadiansPerSecond) * 0.02 / lag,
          -Math.toRadians(720) * 0.02, Math.toRadians(720) * 0.02);
      pose = new Pose2d(pose.getX() + fieldActual.vxMetersPerSecond * 0.02,
          pose.getY() + fieldActual.vyMetersPerSecond * 0.02,
          pose.getRotation().plus(Rotation2d.fromRadians(fieldActual.omegaRadiansPerSecond * 0.02)));
      actual = ChassisSpeeds.fromFieldRelativeSpeeds(fieldActual, pose.getRotation());
      if (t >= pickupStart && pickupRegion(name.endsWith(" Red") ? FlippingUtil.flipFieldPose(pose) : pose)
          && Math.hypot(fieldActual.vxMetersPerSecond, fieldActual.vyMetersPerSecond) > 0.3) {
        double angle = rearIntakeAngle(pose, fieldActual);
        if (angle > maxPickupAngle) {
          maxPickupAngle = angle;
          angleContext = String.format(Locale.ROOT, "t=%.2f actual=%.2f target=%.2f heading=%.1f targetHeading=%.1f motion=%.1f",
              t, Math.hypot(fieldActual.vxMetersPerSecond, fieldActual.vyMetersPerSecond),
              target.linearVelocity, pose.getRotation().getDegrees(), target.pose.getRotation().getDegrees(),
              Math.toDegrees(Math.atan2(fieldActual.vyMetersPerSecond, fieldActual.vxMetersPerSecond)));
        }
      }
      double error = pose.getTranslation().getDistance(target.pose.getTranslation());
      if (error > maxError) {
        maxError = error;
        worstTime = t;
      }
      checkClearance(pose, name + " lag=" + lag + " t=" + t);
      Pose2d bluePose = name.endsWith(" Red") ? FlippingUtil.flipFieldPose(pose) : pose;
      if (name.contains("COLLECT") && touchesBarrier(bluePose)) {
        maxBumpSpeed = Math.max(maxBumpSpeed,
            Math.hypot(fieldActual.vxMetersPerSecond, fieldActual.vyMetersPerSecond));
      }
      if (touchesBarrier(bluePose) && crossingAxisError(bluePose) > Math.toRadians(4)) {
        System.out.printf(Locale.ROOT, "CROSSING %s t=%.2f pose=%s targetHeading=%.2f targetOmega=%.2f actualOmega=%.2f%n",
            name, t, pose, target.pose.getRotation().getDegrees(), target.fieldSpeeds.omegaRadiansPerSecond,
            fieldActual.omegaRadiansPerSecond);
      }
      checkCrossingHeading(bluePose,
          name + " modeled crossing t=" + t);
    }
    System.out.printf(Locale.ROOT, "%s lag %.2f: max error %.3fm at %.2fs; end error %.3fm; max pickup angle %.2fdeg%n",
        name, lag, maxError, worstTime,
        pose.getTranslation().getDistance(trajectory.getEndState().pose.getTranslation()), maxPickupAngle);
    System.out.println("  pickup worst: " + angleContext);
    System.out.printf(Locale.ROOT, "  modeled loaded bump peak: %.3fm/s%n", maxBumpSpeed);
    // Check the delayed plant, not just the path's speed cap. 5 cm/s is a
    // numerical/control tolerance, not permission to plan a faster bump crossing.
    assertTrue(maxBumpSpeed <= AutoTiming.BUMP_SPEED_METERS_PER_SECOND + .05,
        name + " braking started too late for loaded bump: " + maxBumpSpeed);
    assertTrue(maxError < 0.35, name + " tracking error " + maxError + " at " + worstTime);
    // Deliberate 5 cm placement corrections can cause brief lateral translation,
    // so this is wider than the <=10-degree generated-trajectory check above.
    assertTrue(maxPickupAngle <= 25, name + " rear intake lagged pickup motion: " + maxPickupAngle);
    double endTolerance = name.contains("COLLECT") ? 0.20 : 0.30;
    assertTrue(pose.getTranslation().getDistance(trajectory.getEndState().pose.getTranslation()) < endTolerance,
        name + " did not reach its goal with lag " + lag);
  }

  static boolean touchesBarrier(Pose2d pose) {
    return RobotFootprint.maxX(pose) >= 4.02 && RobotFootprint.minX(pose) <= 5.24;
  }

  private static double crossingAxisError(Pose2d pose) {
    double heading = pose.getRotation().getRadians();
    return Math.abs(MathUtil.angleModulus(heading - Math.rint(heading / (Math.PI / 2)) * Math.PI / 2));
  }

  static void checkCrossingHeading(Pose2d bluePose, String context) {
    if (!touchesBarrier(bluePose)) return;
    assertTrue(crossingAxisError(bluePose) < Math.toRadians(4),
        context + " chassis rotating at the barrier");
  }

  static void checkClearance(Pose2d pose, String context) {
    assertTrue(RobotFootprint.insideField(pose), context + " field boundary at " + pose);
    for (double barrierX : new double[] {4.02, FIELD_X - 5.24}) {
      // Hub and trench supports are solid obstacles. Bumps are deliberately traversable.
      for (double[] ys : new double[][] {{3.43, 4.64}, {1.278, 1.67}, {6.40, FIELD_Y - 1.278}}) {
        assertFalse(RobotFootprint.overlaps(pose, barrierX, barrierX + 1.22, ys[0], ys[1]),
            context + " overlaps hub/trench support at " + pose);
      }
    }
  }

  static boolean pickupRegion(Pose2d blue) {
    double c = blue.getRotation().getCos(), s = blue.getRotation().getSin();
    for (double lateral = -.4; lateral <= .4001; lateral += .05) {
      double x = blue.getX() - HALF_LENGTH * c - lateral * s;
      double y = blue.getY() - HALF_LENGTH * s + lateral * c;
      if (x >= 5.55 && x <= 9.2 && y >= 1.72 && y <= 6.5) return true;
    }
    return false;
  }

  @Test
  void sidewaysStartNeedsItsOwnTrenchLaneAndSurvivesPlacementExtremes() throws Exception {
    var oldLane = new Pose2d(4.6, 7.39, Rotation2d.fromDegrees(90));
    assertTrue(RobotFootprint.overlaps(oldLane, 4.02, 5.24, 6.40, 6.792),
        "A sideways deployed intake CANNOT reuse the old 180-degree lane");
    var nominal = new Pose2d(4.6, 7.5453, Rotation2d.fromDegrees(90));
    double margin = Double.POSITIVE_INFINITY;
    for (var corner : RobotFootprint.corners(nominal))
      margin = Math.min(margin, Math.min(8.07 - corner.getY(), corner.getY() - 6.792));
    assertEquals(.0747, margin, 1e-6);
    double perturbedMargin = Double.POSITIVE_INFINITY;
    for (double dy : new double[] {-.05, .05}) {
      for (double yaw : new double[] {-2, 2}) {
        var pose = new Pose2d(4.6, 7.5453 + dy, Rotation2d.fromDegrees(90 + yaw));
        checkClearance(pose, "90-degree placement tolerance");
        for (var corner : RobotFootprint.corners(pose))
          perturbedMargin = Math.min(perturbedMargin,
              Math.min(8.07 - corner.getY(), corner.getY() - 6.792));
      }
    }
    System.out.printf(Locale.ROOT,
        "90-degree trench nominal clearance %.4fm, +/-5cm and +/-2deg envelope %.4fm%n",
        margin, perturbedMargin);
    List<org.junit.jupiter.api.function.Executable> cases = new ArrayList<>();
    for (String side : List.of("LEFT", "RIGHT")) {
      for (String leg : List.of("START_TO_FUEL", "DOUBLE_START_TO_FUEL")) {
        String name = "ECLIPSE_" + side + "_" + leg;
        var trajectory = PathPlannerPath.fromPathFile(name).getIdealTrajectory(config).orElseThrow();
        for (boolean red : new boolean[] {false, true}) {
          var route = red ? trajectory.flip() : trajectory;
          for (double dx : new double[] {-.05, .05}) {
            for (double dy : new double[] {-.05, .05}) {
              for (double yaw : new double[] {-2, 2})
                cases.add(() -> followWithLag(name + (red ? " Red" : ""), route, .12, dx, dy, yaw));
            }
          }
        }
      }
    }
    assertAll("90-degree launches with all placement error signs", cases);
  }

  static double pickupStartTime(PathPlannerPath path, PathPlannerTrajectory trajectory) {
    if (path.getEventMarkers().isEmpty()) return 0;
    double markerPos = path.getEventMarkers().get(0).position();
    int nearest = 0;
    for (int i = 1; i < path.numPoints(); i++) {
      if (Math.abs(path.getPoint(i).waypointRelativePos - markerPos)
          < Math.abs(path.getPoint(nearest).waypointRelativePos - markerPos)) nearest = i;
    }
    return trajectory.getState(nearest).timeSeconds;
  }

  static double rearIntakeAngle(Pose2d pose, ChassisSpeeds fieldSpeed) {
    return Math.abs(Math.toDegrees(MathUtil.angleModulus(Math.atan2(
        fieldSpeed.vyMetersPerSecond, fieldSpeed.vxMetersPerSecond)
        - pose.getRotation().getRadians() - Math.PI)));
  }
}
