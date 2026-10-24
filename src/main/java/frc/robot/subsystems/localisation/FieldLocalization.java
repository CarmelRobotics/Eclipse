package frc.robot.subsystems.localisation;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import frc.robot.subsystems.shooter.ShooterConstants;
import java.util.Optional;

/** Blue-origin localization validation. Alliance selects targets, never a pose reflection. */
public final class FieldLocalization {
  public enum Update { REJECT, FUSE, SEED_TRANSLATION }

  private Optional<Alliance> alliance = Optional.empty();
  private Optional<Alliance> startingAlliance = Optional.empty();
  private Optional<Alliance> lastFmsAlliance = Optional.empty();
  private boolean positionKnown;
  private boolean headingKnown;
  private double lastTimestamp = Double.NEGATIVE_INFINITY;
  private Pose2d pendingPose;
  private double pendingSince;
  private int pendingFrames;
  private String status = "Waiting for a field position and heading";

  public void observeAlliance(Optional<Alliance> reported, boolean fms, boolean disabled,
      Pose2d currentPose) {
    // Keep the last real assignment through a missing DS packet; never turn Unknown into Blue.
    reported.ifPresent(value -> alliance = Optional.of(value));
    if (fms && disabled && reported.isPresent() && !reported.equals(lastFmsAlliance)) {
      lastFmsAlliance = reported;
      startingAlliance = reported;
      // At staging, robots start on their own half. This is NOT a constraint during play.
      if (!onStartingHalf(currentPose)) positionKnown = false;
      clearPending();
    }
  }

  /** Called at match start, before PathPlanner has an opportunity to seed the full pose. */
  public void prepareForMatch(Pose2d currentPose, boolean fms) {
    startingAlliance = fms ? alliance : Optional.empty();
    if (!onStartingHalf(currentPose)) positionKnown = false;
    clearPending();
  }

  public Optional<Alliance> alliance() { return alliance; }
  public boolean isRed() { return alliance.orElse(null) == Alliance.Red; }
  public boolean positionKnown() { return positionKnown; }
  public boolean headingKnown() { return headingKnown; }
  public boolean readyToAim() { return alliance.isPresent() && positionKnown && headingKnown; }
  public String status() { return status; }

  public static Rotation2d downfieldHeading(Alliance alliance) {
    return alliance == Alliance.Red ? Rotation2d.k180deg : Rotation2d.kZero;
  }

  /** A deliberate full-pose reset (e.g. PathPlanner), not a vision XY correction. */
  public void seedPose(Pose2d pose) {
    positionKnown = inField(pose);
    headingKnown = positionKnown;
    startingAlliance = Optional.empty();
    clearPending();
    status = positionKnown ? "Full field pose initialized" : "Invalid explicit pose";
  }

  public void seedHeading() {
    headingKnown = alliance.isPresent();
    status = headingKnown ? "Field heading initialized" : "Cannot seed heading: alliance unknown";
  }

  public static boolean fresh(LimelightHelpers.PoseEstimate estimate, double now) {
    return estimate != null && Double.isFinite(estimate.timestampSeconds)
        && estimate.timestampSeconds <= now + 0.05
        && now - estimate.timestampSeconds <= 0.35;
  }

  public Update evaluate(LimelightHelpers.PoseEstimate estimate, double now, Pose2d currentPose,
      ChassisSpeeds speeds, boolean disabled) {
    if (estimate == null || estimate.tagCount <= 0) return reject("No tags; retaining odometry", true);
    if (!fresh(estimate, now)) return reject("Stale or future vision frame", true);
    if (estimate.timestampSeconds <= lastTimestamp) return reject("No new vision frame", false);
    lastTimestamp = estimate.timestampSeconds;
    if (!inField(estimate.pose) || !Double.isFinite(estimate.avgTagDist)
        || estimate.avgTagDist <= 0 || estimate.avgTagDist > 6.0
        || !Double.isFinite(estimate.latency) || estimate.latency < 0) {
      return reject("Invalid or out-of-field vision pose", true);
    }
    // Vendor guidance rejects distant/ambiguous single-tag solutions. Do not assume
    // a single tag is reliable when its ambiguity metadata is absent.
    if (estimate.tagCount == 1) {
      var raw = estimate.rawFiducials;
      if (raw == null || raw.length != 1 || raw[0] == null
          || !Double.isFinite(raw[0].ambiguity) || raw[0].ambiguity < 0
          || raw[0].ambiguity > 0.2 || !Double.isFinite(raw[0].distToCamera)
          || raw[0].distToCamera <= 0 || raw[0].distToCamera > 3.0) {
        return reject("Single-tag vision too ambiguous or distant", true);
      }
    }
    if (!positionKnown && !onStartingHalf(estimate.pose)) {
      return reject("Wrong starting half: check Limelight field map/tag IDs", true);
    }
    double delta = estimate.pose.getTranslation().getDistance(currentPose.getTranslation());
    if (positionKnown && delta <= ShooterConstants.kMaxVisionCorrectionMeters) {
      clearPending();
      status = "Fusing blue-origin vision; gyro owns heading";
      return Update.FUSE;
    }
    // Never teleport a trusted pose across the field during play. A missing tag
    // leaves odometry running; a bad tag must not replace that odometry wholesale.
    if (positionKnown && !disabled) return reject("Large enabled vision jump rejected", true);
    if (Math.hypot(speeds.vxMetersPerSecond, speeds.vyMetersPerSecond) > 0.25
        || Math.abs(speeds.omegaRadiansPerSecond) > Math.toRadians(15)) {
      return reject("Stop moving to initialize/relocalize position", true);
    }
    // A reset needs multiple distinct, consistent frames, not repeated reads of one frame.
    if (pendingPose == null
        || pendingPose.getTranslation().getDistance(estimate.pose.getTranslation()) > 0.15) {
      pendingPose = estimate.pose;
      pendingSince = estimate.timestampSeconds;
      pendingFrames = 1;
    } else {
      pendingFrames++;
    }
    if (pendingFrames < 4 || estimate.timestampSeconds - pendingSince < 0.10) {
      return reject("Confirming stationary vision position", false);
    }
    positionKnown = true;
    startingAlliance = Optional.empty();
    clearPending();
    status = "Position initialized; gyro heading retained";
    return Update.SEED_TRANSLATION;
  }

  private Update reject(String reason, boolean clear) {
    status = reason;
    if (clear) clearPending();
    return Update.REJECT;
  }

  private void clearPending() { pendingPose = null; pendingFrames = 0; }

  private boolean onStartingHalf(Pose2d pose) {
    if (startingAlliance.isEmpty()) return true;
    double middle = ShooterConstants.kFieldLengthMeters / 2.0;
    return startingAlliance.get() == Alliance.Red ? pose.getX() >= middle : pose.getX() <= middle;
  }

  private static boolean inField(Pose2d pose) {
    return pose != null && Double.isFinite(pose.getX()) && Double.isFinite(pose.getY())
        && Double.isFinite(pose.getRotation().getRadians())
        && pose.getX() >= -0.15 && pose.getX() <= ShooterConstants.kFieldLengthMeters + 0.15
        && pose.getY() >= -0.15 && pose.getY() <= ShooterConstants.kFieldWidthMeters + 0.15;
  }
}
