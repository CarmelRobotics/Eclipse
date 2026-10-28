package frc.robot.subsystems.shooter;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Twist2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import frc.robot.subsystems.localisation.VisionObservation;

/** Robot-relative aiming with bounded measured-chassis-motion holdover. Never resets global odometry. */
public final class VisionAimTracker {
  public static final double HOLDOVER_SECONDS = .30;
  public static final double FRESH_SECONDS = .12;
  private Translation2d target;
  private Translation2d pending;
  private int confirmations;
  private double lastUpdate = Double.NaN, lastCapture = Double.NEGATIVE_INFINITY;
  private double pendingCapture = Double.NEGATIVE_INFINITY;
  private long lastFrame;
  private String lastCamera = "";
  private String status = "Waiting for validated camera frames";
  private double now;
  private int inconsistentFrames;
  private boolean faultLatched;

  public void update(double time, ChassisSpeeds speeds, VisionObservation observation) {
    now = time;
    double dt = Double.isFinite(lastUpdate) ? time - lastUpdate : 0;
    lastUpdate = time;
    if (!finite(speeds) || !Double.isFinite(time) || dt < 0 || dt > .10) {
      reset("Aim clock/motion discontinuity");
      return;
    }
    if (faultLatched) return;
    if (target != null) target = advance(target, speeds, dt);
    if (pending != null) pending = advance(pending, speeds, dt);
    if (observation != null && (observation.frameId() != lastFrame || !observation.camera().equals(lastCamera))) {
      lastFrame = observation.frameId();
      lastCamera = observation.camera();
      double age = time - observation.captureTime();
      if (age < -.03 || age > VisionObservation.MAX_AGE_SECONDS) {
        status = "Rejected stale/future aim observation";
      } else if (observation.captureTime() <= pendingCapture) {
        status = "Ignored duplicate/out-of-order camera frame";
      } else {
        Translation2d candidate = nearestHubVector(observation);
        candidate = advance(candidate, speeds, Math.max(0, age));
        if (target != null && time - lastCapture <= HOLDOVER_SECONDS
            && (target.getDistance(candidate) > .75
                || Math.abs(target.getAngle().minus(candidate.getAngle()).getDegrees()) > 20)) {
          inconsistentFrames++;
          status = "Rejected inconsistent vision target";
          if (inconsistentFrames >= 3) {
            reset("Vision target inconsistent with robot motion - release and re-aim");
            faultLatched = true;
          }
          return;
        }
        inconsistentFrames = 0;
        // Compare in robot-relative space, so a consistent 180-degree field-frame
        // mirror does not change the target or produce an unintended 180-degree spin.
        boolean coherent = pending != null && candidate.getDistance(pending) <= .25
            && Math.abs(candidate.getAngle().minus(pending.getAngle()).getDegrees()) <= 8
            && observation.captureTime() > pendingCapture
            && observation.captureTime() - pendingCapture <= .15;
        confirmations = coherent ? confirmations + 1 : 1;
        pending = candidate;
        pendingCapture = observation.captureTime();
        if (confirmations >= 3) {
          boolean smooth = target != null && time - lastCapture <= HOLDOVER_SECONDS
              && target.getDistance(candidate) <= .5;
          target = smooth ? target.interpolate(candidate, .45) : candidate;
          lastCapture = observation.captureTime();
          status = "Tracking validated vision";
        } else {
          status = "Confirming target (3 distinct frames required)";
        }
      }
    }
    if (target != null && time - lastCapture > HOLDOVER_SECONDS) {
      target = null;
      confirmations = 0;
      pending = null;
      status = "Target expired - manual tower fallback available";
    } else if (target != null && time - lastCapture > FRESH_SECONDS) {
      status = "Brief tag-loss holdover (relative chassis motion only)";
    }
  }

  static Translation2d nearestHubVector(VisionObservation observation) {
    var robot = observation.pose().getTranslation();
    var blue = ShooterConstants.kBlueHubPosition;
    var red = ShooterConstants.kRedHubPosition;
    var hub = robot.getDistance(blue) <= robot.getDistance(red) ? blue : red;
    return hub.minus(robot).rotateBy(observation.pose().getRotation().unaryMinus());
  }

  private static Translation2d advance(Translation2d vector, ChassisSpeeds speeds, double dt) {
    var motion = Pose2d.kZero.exp(new Twist2d(speeds.vxMetersPerSecond * dt,
        speeds.vyMetersPerSecond * dt, speeds.omegaRadiansPerSecond * dt));
    return vector.minus(motion.getTranslation()).rotateBy(motion.getRotation().unaryMinus());
  }

  private static boolean finite(ChassisSpeeds speeds) {
    return speeds != null && Double.isFinite(speeds.vxMetersPerSecond)
        && Double.isFinite(speeds.vyMetersPerSecond) && Double.isFinite(speeds.omegaRadiansPerSecond);
  }

  public boolean valid() { return target != null && now - lastCapture <= HOLDOVER_SECONDS; }
  public boolean inRange() { return valid() && distanceMeters() >= 1.5
      && distanceMeters() <= ShooterConstants.kMaxShotDistanceMeters; }
  public double distanceMeters() { return target == null ? 0 : target.getNorm(); }
  public double bearingErrorRadians(double trimDegrees) { return target == null ? 0
      : MathUtil.angleModulus(target.getAngle().getRadians()
          + Math.toRadians(Double.isFinite(trimDegrees) ? MathUtil.clamp(trimDegrees, -20, 20) : 0)); }
  public double ageSeconds() { return valid() ? Math.max(0, now - lastCapture) : -1; }
  public String status() { return valid() && !inRange() ? "Target outside calibrated 1.5-5 m range" : status; }
  public Translation2d robotRelativeVector() { return target; }
  public void reset(String reason) {
    target = null;
    pending = null;
    confirmations = 0;
    lastCapture = Double.NEGATIVE_INFINITY;
    pendingCapture = Double.NEGATIVE_INFINITY;
    status = reason;
    inconsistentFrames = 0;
  }
  public boolean faultLatched() { return faultLatched; }
  public void rearm() {
    if (!faultLatched) return;
    faultLatched = false;
    lastFrame = 0;
    lastCamera = "";
    reset("Reacquiring after driver re-arm");
  }
}
