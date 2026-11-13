package frc.robot.subsystems.shooter;

import static org.junit.jupiter.api.Assertions.*;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Twist2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import frc.robot.subsystems.localisation.VisionObservation;
import java.nio.file.Files;
import java.nio.file.Path;
import java.util.ArrayDeque;
import java.util.ArrayList;
import java.util.List;
import java.util.Locale;
import java.util.Random;
import org.junit.jupiter.api.Test;
import org.junit.jupiter.params.ParameterizedTest;
import org.junit.jupiter.params.provider.EnumSource;

/**
 * Synthetic 160-second controller/relative-motion runs, NOT a calibrated robot physics model.
 * Uses the production packet decoder/tracker. The small yaw plant and rate-limited P control
 * below are explicit model assumptions; native request/feed checks live in integration tests.
 * No camera rendering, fuel physics, CAN load, or autonomous path following is modeled.
 */
class OdometryDriftSimulationTest {
  private static final double DT = .02, DURATION = 160, LATENCY = .06, TRIM = 6.7;
  private static final int SEEDS = 6;
  private static final AprilTagFieldLayout FIELD = AprilTagFieldLayout.loadField(AprilTagFields.k2026RebuiltWelded);
  private static final Path REPORTS = Path.of("build", "reports", "vision-drift");
  enum Fault { CLEAN, WHEEL_SCALE_10_PERCENT, GYRO_BIAS_HALF_DEG_PER_SECOND,
    GLOBAL_POSE_JUMPS, CAMERA_LOSS_100_MS, CAMERA_LOSS_1_SECOND, CAMERA_LOSS_3_SECONDS, COMBINED }
  private record Pending(double due, long publishUs, double[] packet) {}
  private record Result(boolean red, int seed, double maxOdomError, double maxRangeError,
      double maxBearingErrorDeg, double validPercent, int expiredChecks, double acquireSeconds) {
    String csv(Fault fault) {
      return String.format(Locale.ROOT, "%s,%s,%d,%.6f,%.6f,%.6f,%.3f,%d,%.3f",
          fault, red ? "Red" : "Blue", seed, maxOdomError, maxRangeError,
          maxBearingErrorDeg, validPercent, expiredChecks, acquireSeconds);
    }
  }

  @ParameterizedTest @EnumSource(Fault.class)
  void modeledMatchesKeepRelativeAimIndependentOfGlobalDrift(Fault fault) throws Exception {
    List<String> rows = new ArrayList<>();
    rows.add("fault,alliance,seed,max_global_xy_error_m,max_relative_range_error_m,max_relative_bearing_error_deg,valid_percent_after_warmup,expired_target_checks,first_acquire_s");
    double worstOdom = 0, worstRange = 0, worstBearing = 0, minimumValid = 100;
    int expired = 0;
    for (boolean red : new boolean[] {false, true}) for (int seed = 0; seed < SEEDS; seed++) {
      Result result = run(fault, red, seed);
      rows.add(result.csv(fault));
      worstOdom = Math.max(worstOdom, result.maxOdomError());
      worstRange = Math.max(worstRange, result.maxRangeError());
      worstBearing = Math.max(worstBearing, result.maxBearingErrorDeg());
      minimumValid = Math.min(minimumValid, result.validPercent());
      expired += result.expiredChecks();
      assertTrue(result.maxRangeError() < .20, result.csv(fault));
      assertTrue(result.maxBearingErrorDeg() < 4, result.csv(fault));
      assertTrue(result.validPercent() > 78, result.csv(fault));
      assertTrue(result.acquireSeconds() >= 0 && result.acquireSeconds() <= .20, result.csv(fault));
    }
    if (fault == Fault.CAMERA_LOSS_1_SECOND || fault == Fault.CAMERA_LOSS_3_SECONDS || fault == Fault.COMBINED)
      assertTrue(expired > 0, "Long losses must exercise target expiry, not magically preserve vision");
    Files.createDirectories(REPORTS);
    Files.write(REPORTS.resolve(fault.name().toLowerCase(Locale.ROOT) + ".csv"), rows);
    System.out.printf(Locale.ROOT,
        "DRIFT_MODEL %s: %d x %.0fs; max odom %.3fm, range error %.3fm, bearing error %.3fdeg; min valid %.2f%%; %d expired checks%n",
        fault, SEEDS * 2, DURATION, worstOdom, worstRange, worstBearing, minimumValid, expired);
  }

  private static Result run(Fault fault, boolean red, int seed) {
    var random = new Random(9317L + seed * 53L + (red ? 1000 : 0));
    var tracker = new VisionAimTracker();
    var queue = new ArrayDeque<Pending>();
    var hub = red ? ShooterConstants.kRedHubPosition : ShooterConstants.kBlueHubPosition;
    Translation2d position = position(0, red);
    double yaw = hub.minus(position).getAngle().getRadians() + Math.toRadians(TRIM + 35);
    Pose2d odometry = new Pose2d(position, Rotation2d.fromRadians(yaw));
    double omega = 0, command = 0, firstAcquire = -1;
    double maxOdom = 0, maxRange = 0, maxBearing = 0;
    int valid = 0, eligible = 0, expired = 0;
    VisionObservation latest = null;
    for (int step = 0; step < Math.round(DURATION / DT); step++) {
      double t = step * DT, now = 1 + t;
      var velocityField = position(t + DT, red).minus(position).div(DT);
      var bodyVelocity = velocityField.rotateBy(Rotation2d.fromRadians(-yaw));
      boolean wheelFault = fault == Fault.WHEEL_SCALE_10_PERCENT || fault == Fault.COMBINED;
      boolean gyroFault = fault == Fault.GYRO_BIAS_HALF_DEG_PER_SECOND || fault == Fault.COMBINED;
      var measured = new ChassisSpeeds(bodyVelocity.getX() * (wheelFault ? 1.10 : 1),
          bodyVelocity.getY() * (wheelFault ? 1.10 : 1), omega + (gyroFault ? Math.toRadians(.5) : 0));
      var truth = new Pose2d(position, Rotation2d.fromRadians(yaw));
      if (step % 2 == 0 && !occluded(fault, t)) {
        var noisyPose = new Pose2d(position.plus(new Translation2d(random.nextGaussian() * .025,
            random.nextGaussian() * .025)), Rotation2d.fromRadians(yaw + Math.toRadians(random.nextGaussian() * .45)));
        queue.addLast(new Pending(now + LATENCY, Math.round((now + LATENCY) * 1e6), packet(noisyPose, truth, red, random)));
      }
      while (!queue.isEmpty() && queue.peekFirst().due() <= now + 1e-8) {
        var image = queue.removeFirst();
        var decoded = VisionObservation.decode("synthetic-camera", image.publishUs(), image.packet(), now);
        assertTrue(decoded.valid(), decoded.reason());
        latest = decoded.observation();
      }
      tracker.update(now, measured, latest);
      if (tracker.valid() && firstAcquire < 0) firstAcquire = t;
      double desired = tracker.inRange() ? MathUtil.clamp(ShooterConstants.kVisionAimKp * tracker.bearingErrorRadians(TRIM),
          -ShooterConstants.kVisionAimMaxOmega, ShooterConstants.kVisionAimMaxOmega) : 0;
      command = tracker.inRange() ? command + MathUtil.clamp(desired - command, -6 * DT, 6 * DT) : 0;
      if (latest != null && now - latest.captureTime() > VisionAimTracker.HOLDOVER_SECONDS + 1e-8) {
        assertFalse(tracker.valid(), "Stale camera data must not sustain aiming during drift");
        assertEquals(0, command, 0);
        expired++;
      }
      if (t >= 1) {
        eligible++;
        if (tracker.valid()) {
          valid++;
          var trueVector = hub.minus(position).rotateBy(Rotation2d.fromRadians(-yaw));
          maxRange = Math.max(maxRange, Math.abs(tracker.distanceMeters() - trueVector.getNorm()));
          maxBearing = Math.max(maxBearing, Math.abs(tracker.robotRelativeVector().getAngle()
              .minus(trueVector.getAngle()).getDegrees()));
        }
      }
      // This separate global estimate accumulates measured encoder/gyro errors.
      // It is deliberately NEVER passed to the relative-aim tracker.
      odometry = odometry.exp(new Twist2d(measured.vxMetersPerSecond * DT,
          measured.vyMetersPerSecond * DT, measured.omegaRadiansPerSecond * DT));
      if (fault == Fault.GLOBAL_POSE_JUMPS || fault == Fault.COMBINED) {
        if (step == 500) odometry = new Pose2d(odometry.getX() + 2, odometry.getY() - 1,
            odometry.getRotation().plus(Rotation2d.fromDegrees(45)));
        if (step == 3000) odometry = new Pose2d(ShooterConstants.kFieldLengthMeters - odometry.getX(),
            ShooterConstants.kFieldWidthMeters - odometry.getY(), odometry.getRotation().plus(Rotation2d.k180deg));
        if (step == 5000) odometry = new Pose2d(red ? 1 : 15, 7, odometry.getRotation());
      }
      maxOdom = Math.max(maxOdom, odometry.getTranslation().getDistance(position));
      // Assumed yaw actuator: 100 ms response, 6 rad/s^2 acceleration limit.
      omega += MathUtil.clamp((command - omega) * DT / .10, -6 * DT, 6 * DT);
      yaw += omega * DT;
      position = position(t + DT, red);
      assertTrue(Double.isFinite(command));
      assertTrue(Math.abs(command) <= ShooterConstants.kVisionAimMaxOmega + 1e-8);
    }
    return new Result(red, seed, maxOdom, maxRange, maxBearing, 100.0 * valid / eligible, expired, firstAcquire);
  }

  private static Translation2d position(double time, boolean red) {
    double x = ShooterConstants.kBlueHubPosition.getX() - 3 + .55 * Math.sin(time * .26);
    double y = ShooterConstants.kBlueHubPosition.getY() + .45 * Math.sin(time * .19);
    return red ? new Translation2d(ShooterConstants.kFieldLengthMeters - x,
        ShooterConstants.kFieldWidthMeters - y) : new Translation2d(x, y);
  }

  private static boolean occluded(Fault fault, double t) {
    return switch (fault) {
      case CAMERA_LOSS_100_MS -> t % 6 >= 4.8 && t % 6 < 4.9;
      case CAMERA_LOSS_1_SECOND -> t % 12 >= 7 && t % 12 < 8;
      case CAMERA_LOSS_3_SECONDS -> t % 30 >= 12 && t % 30 < 15;
      case COMBINED -> (t % 6 >= 4.8 && t % 6 < 4.9) || (t % 30 >= 12 && t % 30 < 15)
          || (t % 20 >= 17 && t % 20 < 18);
      default -> false;
    };
  }

  /** Synthetic quality metadata; does not claim these tags are inside a real camera's FOV. */
  private static double[] packet(Pose2d reported, Pose2d truth, boolean red, Random random) {
    double[] data = {reported.getX(), reported.getY(), 0, 0, 0, reported.getRotation().getDegrees(),
        LATENCY * 1000, 2, .5, 2, .5, red ? 15 : 31, 0, 0, .5, 2, 2, .05,
        red ? 16 : 32, 0, 0, .5, 2, 2, .05};
    double sum = 0;
    for (int base : new int[] {11, 18}) {
      var tag = FIELD.getTagPose((int) data[base]).orElseThrow();
      double planar = tag.toPose2d().getTranslation().getDistance(truth.getTranslation());
      double range = Math.sqrt(planar * planar + tag.getZ() * tag.getZ()) + random.nextGaussian() * .005;
      data[base + 4] = range;
      data[base + 5] = range;
      sum += range;
    }
    data[9] = sum / 2;
    return data;
  }

  @Test void stableCameraYawCalibrationErrorRemainsARealLimitation() {
    var tracker = new VisionAimTracker();
    var truth = new Pose2d(position(0, false), Rotation2d.kZero);
    for (int i = 0; i < 10; i++) {
      double now = 1 + i * DT;
      var reported = new Pose2d(truth.getTranslation(), Rotation2d.fromDegrees(20));
      var decoded = VisionObservation.decode("camera", Math.round(now * 1e6), packet(reported, truth, false, new Random(i)), now);
      assertTrue(decoded.valid());
      tracker.update(now, new ChassisSpeeds(), decoded.observation());
    }
    assertTrue(tracker.valid());
    assertEquals(-20, Math.toDegrees(tracker.bearingErrorRadians(0)), .01,
        "Consistent camera yaw errors cannot be cured by ignoring global odometry");
    System.out.println("DRIFT_LIMIT: stable +20 deg camera-yaw miscalibration remains -20 deg aiming error");
  }

  @Test void positionOnlyCameraMirrorWithOriginalTagIdsFailsRangeValidation() {
    var truth = new Pose2d(position(0, false), Rotation2d.kZero);
    var mirrored = new Pose2d(ShooterConstants.kFieldLengthMeters - truth.getX(),
        ShooterConstants.kFieldWidthMeters - truth.getY(), truth.getRotation());
    var decoded = VisionObservation.decode("camera", 1_000_000, packet(mirrored, truth, false, new Random(5)), 1);
    assertFalse(decoded.valid());
    assertTrue(decoded.reason().contains("inconsistent with tag ranges"), decoded.reason());
  }
}
