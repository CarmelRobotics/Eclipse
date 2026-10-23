package frc.robot.auto;

import static org.junit.jupiter.api.Assertions.*;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.commands.PathPlannerAuto;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.util.FlippingUtil;
import edu.wpi.first.hal.HAL;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import frc.robot.subsystems.shooter.ShooterConstants;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.simulation.SimHooks;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.Subsystem;
import java.nio.file.Files;
import java.nio.file.Path;
import java.util.ArrayList;
import java.util.List;
import java.util.Locale;
import org.junit.jupiter.api.Test;

/** Real PathPlanner auto execution with a delayed drive plant and explicitly mocked mechanisms. */
class CompetitionAutoExecutionTest {
  @Test
  void executeAllCompetitionAutosOnBothAlliancesAndHandleLateFirstShot() throws Exception {
    assertTrue(HAL.initialize(500, 0));
    SimHooks.pauseTiming();
    try {
      for (boolean red : new boolean[] {false, true}) {
        for (String side : List.of("left", "right")) {
          execute("2910_trench_" + side, red, 0.55, 0.55, 0.35);
          execute("2910_trench_" + side + "_double", red, 0.55, 0.55, 0.35);
          execute("2910_trench_" + side + "_double", red, 1.25, 0.55, 0.35);
          execute("2910_trench_" + side + "_double", red, 0.55, 1.25, 0.35);
          execute("2910_trench_" + side + "_double", red, 0.55, 0.55, 1.80);
          execute("2910_trench_" + side, red, 0.55, 0.55, 0.35, 0.0);
          execute("2910_trench_" + side + "_double", red, 0.55, 0.55, 0.35, 0.0);
          execute("2910_trench_" + side + "_double", red, 0.55, 1.25, 0.35, 0.0);
          execute("2910_trench_" + side + "_double", red, 0.55, 0.55, 1.35, 0.0);
          execute("2910_trench_" + side + "_double", red, 0.55, 0.55, 1.55, 0.35);
          execute("2910_trench_" + side, red, 0.55, 0.55, 0.35, 0.60);
          execute("2910_trench_" + side + "_double", red, 0.55, 0.55, 0.35, 0.60);
        }
      }
    } finally {
      AutoBuilder.resetForTesting();
      SimHooks.resumeTiming();
    }
  }

  private static void execute(String name, boolean red, double firstSpinup,
      double secondSpinup, double pivotStow) throws Exception {
    execute(name, red, firstSpinup, secondSpinup, pivotStow, 0.35);
  }

  private static void execute(String name, boolean red, double firstSpinup,
      double secondSpinup, double pivotStow, double initialStow) throws Exception {
    execute(name, red, firstSpinup, secondSpinup, pivotStow, initialStow, false);
  }

  @Test
  void isolatedSecondTripUsesTrenchAndRestartsIntakeAfterClearance() throws Exception {
    assertTrue(HAL.initialize(500, 0));
    SimHooks.pauseTiming();
    try {
      for (boolean red : new boolean[] {false, true}) {
        for (String side : List.of("left", "right")) {
          // Isolated route/fault tests after an ASSUMED completed unload; these
          // do not claim that two full collections + five-second dump fit a match.
          execute("2910_trench_" + side + "_double", red, .55, .55, .35, 0, true);
          execute("2910_trench_" + side + "_double", red, 1.25, 1.25, .35, 0, true);
          execute("2910_trench_" + side + "_double", red, .55, .55, 1.8, 0, true);
        }
      }
    } finally {
      AutoBuilder.resetForTesting();
      SimHooks.resumeTiming();
    }
  }

  private static void execute(String name, boolean red, double firstSpinup,
      double secondSpinup, double pivotStow, double initialStow, boolean secondOnly) throws Exception {
    AutoBuilder.resetForTesting();
    // Event markers capture NamedCommands. Never reuse one case's roller/plant
    // closures in another case via PathPlanner's global path cache.
    PathPlannerPath.clearCache();
    var plant = new Plant(RobotConfig.fromGUISettings());
    // Export one prompt-readiness case for route animation; no robot-code changes.
    boolean exportTrace = name.equals("2910_trench_left_double") && !red && !secondOnly
        && firstSpinup == .55 && secondSpinup == .55 && pivotStow == .35 && initialStow == .35;
    List<String> trace = new ArrayList<>();
    if (exportTrace) trace.add("timeSeconds,x,y,headingDeg,speedMps,roller,intakeStarts,shots,"
        + "feed1Seconds,feed2Seconds,secondPreparation,secondAdmitted,stopped");
    Subsystem drive = new Subsystem() {};
    var clock = new Timer();
    final boolean[] safe = {true};
    final boolean[] stopped = {false};
    final boolean[] roller = {false};
    final int[] shots = {0};
    final double[] feedSeconds = {0, 0, 0};
    final boolean[] admitted = {false};
    final boolean[] preparationStarted = {false};
    final int[] intakeStarts = {0};
    var blueTravelHeading = Rotation2d.fromDegrees(name.contains("left") ? -90 : 90);
    var travelHeading = red ? FlippingUtil.flipFieldRotation(blueTravelHeading) : blueTravelHeading;
    Timer travelClock = new Timer();
    Command travel = Commands.sequence(
        Commands.deadline(
            Commands.waitUntil(() -> travelClock.hasElapsed(pivotStow)
                && plant.headingError(travelHeading) < Math.toRadians(4)
                && Math.abs(plant.speed.omegaRadiansPerSecond)
                    <= AutoTiming.MAX_TRAVEL_ANGULAR_SPEED_RAD_PER_SEC)
                .withTimeout(AutoTiming.TRAVEL_TIMEOUT_SECONDS),
            Commands.run(() -> plant.holdHeading(travelHeading))),
        Commands.runOnce(() -> safe[0] = travelClock.hasElapsed(pivotStow)
            && plant.headingError(travelHeading) < Math.toRadians(4)
            && Math.abs(plant.speed.omegaRadiansPerSecond)
                <= AutoTiming.MAX_TRAVEL_ANGULAR_SPEED_RAD_PER_SEC))
        .beforeStarting(travelClock::restart);
    AutoBuilder.configure(() -> plant.pose, pose -> {
      plant.pose = pose;
      plant.speed = new ChassisSpeeds();
      plant.fieldSpeed = new ChassisSpeeds();
      plant.resets++;
    }, () -> plant.speed, (speed, ff) -> plant.demand = speed,
        new PPHolonomicDriveController(new PIDConstants(10, 0, 0), new PIDConstants(7, 0, 0)),
        RobotConfig.fromGUISettings(), () -> red, drive);
    Timer startClock = new Timer();
    var blueStartHeading = Rotation2d.fromDegrees(name.contains("left") ? 90 : -90);
    var startHeading = red ? FlippingUtil.flipFieldRotation(blueStartHeading) : blueStartHeading;
    Command start = Commands.sequence(
        Commands.deadline(
            Commands.waitUntil(() -> startClock.hasElapsed(initialStow)
                && plant.headingError(startHeading) <= Math.toRadians(4)
                && Math.abs(plant.speed.omegaRadiansPerSecond)
                    <= AutoTiming.MAX_TRAVEL_ANGULAR_SPEED_RAD_PER_SEC)
                .withTimeout(AutoTiming.INITIAL_STOW_TIMEOUT_SECONDS),
            Commands.run(() -> plant.holdHeading(startHeading))),
        Commands.runOnce(() -> safe[0] = startClock.hasElapsed(initialStow)
            && plant.headingError(startHeading) <= Math.toRadians(4)
            && Math.abs(plant.speed.omegaRadiansPerSecond)
                <= AutoTiming.MAX_TRAVEL_ANGULAR_SPEED_RAD_PER_SEC))
        .beforeStarting(startClock::restart);
    NamedCommands.registerCommand("auto start", Commands.runOnce(
        () -> fail("Eclipse sideways auto used the legacy 180-degree startup")));
    NamedCommands.registerCommand(name.contains("left") ? "auto start left" : "auto start right", start);
    NamedCommands.registerCommand("intake run", Commands.runOnce(() -> {
      var blue = red ? FlippingUtil.flipFieldPose(plant.pose) : plant.pose;
      assertTrue(RobotFootprint.minX(blue) > 5.24, "Collection started before clearing trench");
      assertTrue(CompetitionPathsTest.rearIntakeAngle(plant.pose, plant.fieldSpeed) <= 25,
          "Collection started before rear intake aligned with motion");
      intakeStarts[0]++;
      roller[0] = true;
    }));
    NamedCommands.registerCommand("intake stop", Commands.runOnce(() -> roller[0] = false));
    NamedCommands.registerCommand("auto collect settle", Commands.sequence(
        Commands.runOnce(() -> plant.demand = new ChassisSpeeds()),
        Commands.waitSeconds(AutoTiming.COLLECT_SETTLE_SECONDS)));
    NamedCommands.registerCommand("stopshoot", Commands.none());
    for (String commandName : List.of("shoot", "dump", "auto finish shoot")) {
      double feed = switch (commandName) {
        case "shoot", "dump" -> AutoTiming.DUMP_FEED_SECONDS;
        default -> AutoTiming.DURATION_SECONDS;
      };
      Timer prepareClock = new Timer();
      NamedCommands.registerCommand(commandName, Commands.sequence(
          Commands.deadline(
              Commands.sequence(
                  Commands.waitUntil(() -> prepareClock.hasElapsed(shots[0] == 0 ? firstSpinup : secondSpinup)
                      && plant.headingError(plant.hubHeading(red)) < Math.toRadians(5))
                      .withTimeout(ShooterConstants.kShotSpinupTimeoutSeconds),
                  Commands.runOnce(() -> shots[0]++),
                  Commands.run(() -> feedSeconds[shots[0] - 1] += 0.02).withTimeout(feed)),
              // Real angular drive response; only pivot/flywheel readiness and feed
              // durations are supplied as assumptions. Never teleport the heading.
              Commands.run(() -> plant.holdHeading(plant.hubHeading(red))))
              .beforeStarting(() -> { prepareClock.restart(); roller[0] = false; })));
    }
    NamedCommands.registerCommand(name.contains("left") ? "auto cycle 2 left" : "auto cycle 2 right", Commands.either(
        Commands.sequence(travel, Commands.either(Commands.none(),
            NamedCommands.getCommand("auto finish shoot"), () -> {
              admitted[0] = safe[0] && AutoTiming.canStartSecondCycle(clock.get());
              return admitted[0];
            }).onlyIf(() -> safe[0])),
        NamedCommands.getCommand("auto finish shoot"),
        () -> {
          preparationStarted[0] = AutoTiming.canStartSecondCycle(clock.get());
          return preparationStarted[0];
        }));
    Command auto;
    if (secondOnly) {
      String side = name.contains("left") ? "LEFT" : "RIGHT";
      auto = Commands.sequence(
          Commands.runOnce(() -> {
            var score = new Pose2d(3, sideY(name), Rotation2d.kZero);
            plant.pose = red ? FlippingUtil.flipFieldPose(score) : score;
            plant.pose = new Pose2d(plant.pose.getTranslation(), plant.hubHeading(red));
            plant.resets++;
          }),
          NamedCommands.getCommand(name.contains("left") ? "auto cycle 2 left" : "auto cycle 2 right"),
          AutoBuilder.followPath(PathPlannerPath.fromPathFile("ECLIPSE_" + side + "_SCORE_TO_FUEL2")),
          AutoBuilder.followPath(PathPlannerPath.fromPathFile("ECLIPSE_" + side + "_COLLECT2_TO_SCORE")),
          NamedCommands.getCommand("auto finish shoot"));
    } else {
      auto = new PathPlannerAuto(name);
    }
    var command = AutoTiming.guard(auto, () -> safe[0], clock::restart, () -> {
      stopped[0] = true;
      roller[0] = false;
      plant.demand = new ChassisSpeeds();
    });
    command.initialize();
    double intakeAtCenterline = Double.NaN;
    double robotAtCenterline = Double.NaN;
    double firstFuelContact = Double.NaN;
    double firstActivePickup = Double.NaN;
    double maxPickupAngle = 0;
    double maxBumpSpeed = 0;
    for (int tick = 0; tick < 1100 && !command.isFinished(); tick++) {
      command.execute();
      plant.update();
      if (exportTrace) trace.add(traceRow(clock.get(), plant, roller[0], intakeStarts[0], shots[0],
          feedSeconds, preparationStarted[0], admitted[0], false));
      CompetitionPathsTest.checkClearance(plant.pose, name + " full auto t=" + clock.get());
      CompetitionPathsTest.checkCrossingHeading(red ? FlippingUtil.flipFieldPose(plant.pose) : plant.pose,
          name + " full auto t=" + clock.get());
      var blue = red ? FlippingUtil.flipFieldPose(plant.pose) : plant.pose;
      if (CompetitionPathsTest.touchesBarrier(blue)
          && (red ? -plant.fieldSpeed.vxMetersPerSecond : plant.fieldSpeed.vxMetersPerSecond) < -.2) {
        maxBumpSpeed = Math.max(maxBumpSpeed,
            Math.hypot(plant.fieldSpeed.vxMetersPerSecond, plant.fieldSpeed.vyMetersPerSecond));
      }
      if (secondOnly && blue.getX() >= 4.02 && blue.getX() <= 5.24
          && (red ? -plant.fieldSpeed.vxMetersPerSecond : plant.fieldSpeed.vxMetersPerSecond) > .2) {
        assertEquals(name.contains("left") ? 7.39 : .68, blue.getY(), .08,
            "Second outbound did not use trench");
        assertFalse(roller[0], "Must not collect more fuel while transiting trench after unload");
      }
      if (roller[0] && CompetitionPathsTest.pickupRegion(blue)
          && Math.hypot(plant.fieldSpeed.vxMetersPerSecond, plant.fieldSpeed.vyMetersPerSecond) > .3) {
        maxPickupAngle = Math.max(maxPickupAngle,
            CompetitionPathsTest.rearIntakeAngle(plant.pose, plant.fieldSpeed));
      }
      if (!Double.isFinite(firstFuelContact) && CompetitionPathsTest.intakeTouchesStartingFuel(blue))
        firstFuelContact = clock.get();
      if (roller[0] && !Double.isFinite(firstActivePickup)
          && CompetitionPathsTest.intakeTouchesStartingFuel(blue))
        firstActivePickup = clock.get();
      double rearX = blue.getX() - 0.6786 * blue.getRotation().getCos();
      if (!Double.isFinite(intakeAtCenterline) && rearX >= 8.27) intakeAtCenterline = clock.get();
      if (!Double.isFinite(robotAtCenterline) && blue.getX() >= 8.27) robotAtCenterline = clock.get();
      double extent = 0.45 * (Math.abs(blue.getRotation().getCos())
          + Math.abs(blue.getRotation().getSin()));
      assertTrue(blue.getX() - extent <= 8.27,
          name + " modeled bumpers wholly across centerline at " + blue + " t=" + clock.get());
      SimHooks.stepTiming(0.02);
    }
    assertTrue(command.isFinished(), name + " never finished");
    command.end(false);
    if (exportTrace) {
      trace.add(traceRow(clock.get(), plant, roller[0], intakeStarts[0], shots[0],
          feedSeconds, preparationStarted[0], admitted[0], stopped[0]));
      var reports = Path.of("build", "reports", "auto-paths");
      Files.createDirectories(reports);
      Files.write(reports.resolve("left-double-command-trace.csv"), trace);
    }
    assertTrue(stopped[0], name + " skipped cleanup");
    assertFalse(roller[0], name + " left roller on");
    assertEquals(1, plant.resets, name + " must seed pose once at auto start");
    assertTrue(clock.get() <= 19.85, name + " exceeded auto budget: " + clock.get());
    assertTrue(maxPickupAngle <= 25, name + " rear intake not leading: " + maxPickupAngle);
    Pose2d bluePose = red ? FlippingUtil.flipFieldPose(plant.pose) : plant.pose;
    System.out.printf(Locale.ROOT, "%s%s %s readiness %.2f/%.2fs stow %.2fs: %.2fs, %d feed windows (%.2f/%.2fs feed), goal (%.3f, %.3f), pickup angle %.2fdeg%n",
        name, secondOnly ? " ISOLATED SECOND TRIP" : "", red ? "Red" : "Blue", firstSpinup, secondSpinup, pivotStow,
        clock.get(), shots[0], feedSeconds[0], feedSeconds[1], bluePose.getX(), bluePose.getY(), maxPickupAngle);
    System.out.printf(Locale.ROOT,
        "  From auto enable: first fuel %.2fs, rear intake at centerline %.2fs, robot center %.2fs (initial stow %.2fs); second trip admitted=%s%n",
        firstFuelContact, intakeAtCenterline, robotAtCenterline, initialStow, admitted[0]);
    System.out.printf(Locale.ROOT, "  First active pickup from enable: %.2fs%n", firstActivePickup);
    System.out.printf(Locale.ROOT, "  modeled loaded bump peak: %.3fm/s%n", maxBumpSpeed);
    assertTrue(maxBumpSpeed <= AutoTiming.BUMP_SPEED_METERS_PER_SECOND + .05,
        name + " modeled loaded bump overspeed: " + maxBumpSpeed);
    if (!secondOnly && initialStow > AutoTiming.INITIAL_STOW_TIMEOUT_SECONDS) {
      assertFalse(safe[0], "Failed initial stow must abort the sideways launch");
      assertEquals(0, intakeStarts[0]);
      assertEquals(0, shots[0]);
      assertTrue(clock.get() < .55, "Initial timeout must stop without executing a path");
      assertEquals(4.4, bluePose.getX(), 1e-9);
      assertEquals(name.contains("left") ? 7.5453 : 8.07 - 7.5453, bluePose.getY(), 1e-9);
      return;
    }
    if (!secondOnly) assertTrue(Double.isFinite(intakeAtCenterline) && Double.isFinite(robotAtCenterline), name);
    assertEquals(3.0, bluePose.getX(), 0.20, name + " did not return to score");
    assertEquals(sideY(name), bluePose.getY(), 0.20, name + " did not return to score");
    if (secondOnly) {
      assertTrue(preparationStarted[0], "Isolated second-trip preparation must actually execute");
      if (pivotStow < AutoTiming.TRAVEL_TIMEOUT_SECONDS) {
        assertTrue(admitted[0]);
        assertTrue(safe[0]);
        assertEquals(1, intakeStarts[0], "Trench exit marker must restart intake exactly once");
        assertEquals(1, shots[0], "Isolated trip did not return and feed");
        assertTrue(feedSeconds[0] >= AutoTiming.MIN_FINAL_FEED_SECONDS - .05);
      } else {
        assertFalse(safe[0], "Failed stow must abort at score");
        assertEquals(0, intakeStarts[0]);
        assertEquals(0, shots[0]);
      }
    } else if (!name.endsWith("double")) {
      assertEquals(1, shots[0], name);
      assertTrue(feedSeconds[0] >= AutoTiming.DUMP_FEED_SECONDS - 0.05, name);
    } else if (!safe[0]) {
      assertEquals(1, shots[0], "Failed stow must abort after first dump");
      assertEquals(1, intakeStarts[0], "Failed stow must not restart collection");
    } else if (admitted[0]) {
      assertEquals(2, shots[0], name + " should reach both scores when admitted");
      assertTrue(feedSeconds[1] >= AutoTiming.MIN_FINAL_FEED_SECONDS - 0.05,
          name + " admitted a second trip without a useful final shot");
    } else {
      assertEquals(2, shots[0], name + " should finish unloading without a second trip");
      assertTrue(feedSeconds[1] >= 1, name + " wasted the remaining auto instead of unloading");
      assertEquals(1, intakeStarts[0], "Late-trip fallback must not restart collection");
    }
    if (name.endsWith("double") && !secondOnly) {
      assertTrue(feedSeconds[0] >= AutoTiming.DUMP_FEED_SECONDS - 0.05,
          name + " departed before completing the five-second first dump");
      if (initialStow <= .35 && firstSpinup < 1 && pivotStow < 1) {
        assertTrue(admitted[0], name + " must run two actual collection cycles with prompt readiness");
        assertEquals(2, intakeStarts[0], name + " must restart intake after trench exit on cycle two");
      }
      if (pivotStow == 1.35) {
        assertTrue(safe[0]);
        assertTrue(admitted[0], "Faster return now leaves time for a 1.35-second preparation");
      }
      if (pivotStow == 1.55) {
        assertTrue(preparationStarted[0], "Late-preparation case must reach the second time check");
        assertTrue(safe[0], "This case is late, not a stow fault");
        assertFalse(admitted[0], "Time must be rechecked after successful but slow preparation");
      }
    }
  }

  private static String traceRow(double time, Plant plant, boolean roller, int intakeStarts,
      int shots, double[] feed, boolean preparing, boolean admitted, boolean stopped) {
    return String.format(Locale.ROOT, "%.4f,%.6f,%.6f,%.4f,%.4f,%s,%d,%d,%.4f,%.4f,%s,%s,%s",
        time, plant.pose.getX(), plant.pose.getY(), plant.pose.getRotation().getDegrees(),
        Math.hypot(plant.fieldSpeed.vxMetersPerSecond, plant.fieldSpeed.vyMetersPerSecond),
        roller, intakeStarts, shots, feed[0], feed[1], preparing, admitted, stopped);
  }

  private static double sideY(String name) {
    return name.contains("left") ? 5.55 : 8.07 - 5.55;
  }

  @Test
  void timeLimitAndTravelFaultAlwaysRunCleanup() {
    HAL.initialize(500, 0);
    SimHooks.pauseTiming();
    try {
      for (boolean fault : new boolean[] {false, true}) {
        boolean[] safe = {true};
        boolean[] stopped = {false};
        Timer clock = new Timer();
        Command guarded = AutoTiming.guard(Commands.run(() -> {}), () -> safe[0],
            clock::restart, () -> stopped[0] = true);
        guarded.initialize();
        for (int tick = 0; tick < 1100 && !guarded.isFinished(); tick++) {
          guarded.execute();
          if (fault && clock.get() > 1.0) safe[0] = false;
          SimHooks.stepTiming(0.02);
        }
        assertTrue(guarded.isFinished());
        guarded.end(false);
        assertTrue(stopped[0]);
        assertTrue(clock.get() <= (fault ? 1.10 : 19.85));
      }
    } finally {
      SimHooks.resumeTiming();
    }
  }

  private static final class Plant {
    final RobotConfig config;
    Plant(RobotConfig config) { this.config = config; }
    Pose2d pose = Pose2d.kZero;
    ChassisSpeeds demand = new ChassisSpeeds();
    ChassisSpeeds speed = new ChassisSpeeds();
    ChassisSpeeds fieldSpeed = new ChassisSpeeds();
    int resets;

    double headingError(Rotation2d heading) {
      return Math.abs(MathUtil.angleModulus(heading.minus(pose.getRotation()).getRadians()));
    }

    Rotation2d hubHeading(boolean red) {
      var hub = red ? ShooterConstants.kRedHubPosition : ShooterConstants.kBlueHubPosition;
      return hub.minus(pose.getTranslation()).getAngle().plus(Rotation2d.fromDegrees(6.7));
    }

    void holdHeading(Rotation2d heading) {
      demand = new ChassisSpeeds(0, 0, MathUtil.clamp(8.0 * MathUtil.angleModulus(
          heading.minus(pose.getRotation()).getRadians()) - 0.2 * speed.omegaRadiansPerSecond,
          -Math.toRadians(360), Math.toRadians(360)));
    }

    void update() {
      var fieldDemand = ChassisSpeeds.fromRobotRelativeSpeeds(demand, pose.getRotation());
      double dx = (fieldDemand.vxMetersPerSecond - fieldSpeed.vxMetersPerSecond) * 0.02 / 0.12;
      double dy = (fieldDemand.vyMetersPerSecond - fieldSpeed.vyMetersPerSecond) * 0.02 / 0.12;
      boolean braking = dx * fieldSpeed.vxMetersPerSecond + dy * fieldSpeed.vyMetersPerSecond < 0;
      double accelLimit = DrivePhysicsModel.accelerationLimit(config, speed, braking, 10.5);
      double scale = Math.min(1.0, accelLimit * 0.02 / Math.max(0.001, Math.hypot(dx, dy)));
      fieldSpeed.vxMetersPerSecond += dx * scale;
      fieldSpeed.vyMetersPerSecond += dy * scale;
      fieldSpeed.omegaRadiansPerSecond += MathUtil.clamp(
          (MathUtil.clamp(demand.omegaRadiansPerSecond, -Math.toRadians(360), Math.toRadians(360))
              - fieldSpeed.omegaRadiansPerSecond) * 0.02 / 0.12,
          -Math.toRadians(720) * 0.02, Math.toRadians(720) * 0.02);
      pose = new Pose2d(pose.getX() + fieldSpeed.vxMetersPerSecond * 0.02,
          pose.getY() + fieldSpeed.vyMetersPerSecond * 0.02,
          pose.getRotation().plus(Rotation2d.fromRadians(fieldSpeed.omegaRadiansPerSecond * 0.02)));
      speed = ChassisSpeeds.fromFieldRelativeSpeeds(fieldSpeed, pose.getRotation());
    }
  }
}
