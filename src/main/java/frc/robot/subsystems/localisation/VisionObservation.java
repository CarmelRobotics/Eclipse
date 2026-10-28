package frc.robot.subsystems.localisation;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import frc.robot.subsystems.shooter.ShooterConstants;

/** One atomic MegaTag1 blue-origin packet. Never assembled from unrelated NT topics. */
public record VisionObservation(String camera, long frameId, Pose2d pose, double captureTime,
    double latencyMs, int tagCount, double tagSpan, double averageDistance, double averageArea,
    LimelightHelpers.RawFiducial[] tags) {
  public static final double MAX_AGE_SECONDS = .25;
  private static final AprilTagFieldLayout FIELD = AprilTagFieldLayout.loadField(AprilTagFields.k2026RebuiltWelded);
  public record Decode(VisionObservation observation, String reason) {
    public boolean valid() { return observation != null; }
  }

  public static Decode decode(String camera, long timestampUs, double[] data, double now) {
    if (data == null || data.length < 11 || data.length > 11 + 32 * 7)
      return bad("Missing/malformed botpose_wpiblue packet");
    for (double value : data) if (!Double.isFinite(value)) return bad("Non-finite camera packet");
    if (!Double.isFinite(now) || timestampUs <= 0) return bad("Invalid camera timestamp");
    if (data[6] < 0 || data[6] > 200) return bad("Invalid/excessive camera latency");
    double capture = timestampUs / 1e6 - data[6] / 1000;
    if (now - capture < -.03 || now - capture > MAX_AGE_SECONDS) return bad("Stale/future camera pose");
    if (data[7] != Math.rint(data[7]) || data[7] < 1 || data[7] > 32) return bad("No valid AprilTags in pose packet");
    int count = (int) data[7];
    if (data[0] < -.15 || data[0] > ShooterConstants.kFieldLengthMeters + .15
        || data[1] < -.15 || data[1] > ShooterConstants.kFieldWidthMeters + .15
        || Math.abs(data[2]) > .35 || Math.abs(data[3]) > 25 || Math.abs(data[4]) > 25)
      return bad("Camera pose outside field/ground plane");
    if (data[9] <= 0 || data[9] > 6 || data[10] <= 0 || data[8] < 0)
      return bad("Invalid tag distance/area/span");
    boolean metadata = data.length == 11 + count * 7;
    if (!metadata && data.length != 11) return bad("Incomplete embedded tag metadata");
    if (count == 1 && !metadata) return bad("Single tag needs ambiguity metadata");
    // A well-spread multi-tag solve may use the standard 11-value packet on older
    // camera firmware. A single tag must always include its quality information.
    if (count > 1 && data[8] < .15) return bad("Insufficient multi-tag span");
    var tags = new LimelightHelpers.RawFiducial[metadata ? count : 0];
    long seen = 0;
    for (int i = 0; i < tags.length; i++) {
      int base = 11 + i * 7;
      double tagId = data[base], distance = data[base + 4], ambiguity = data[base + 6];
      if (tagId != Math.rint(tagId) || tagId < 1 || tagId > 32 || (seen & (1L << (int) tagId)) != 0)
        return bad("Invalid/duplicate AprilTag ID");
      seen |= 1L << (int) tagId;
      if (distance <= 0 || distance > 6 || data[base + 5] <= 0 || ambiguity < 0 || ambiguity > .7)
        return bad("Unreliable embedded tag quality");
      if (count == 1 && (ambiguity > .2 || distance > 4.5)) return bad("Ambiguous/distant single tag");
      // Reject an incorrect field origin/map rather than letting a plausible-looking
      // X/Y coordinate command a turn. Measured robot-to-tag range is in this same packet.
      var tagPose = FIELD.getTagPose((int) tagId).orElseThrow();
      double dx = tagPose.getX() - data[0], dy = tagPose.getY() - data[1], dz = tagPose.getZ() - data[2];
      double predictedRange = Math.sqrt(dx * dx + dy * dy + dz * dz);
      if (Math.abs(predictedRange - data[base + 5]) > .35 + .03 * data[base + 5])
        return bad("Pose inconsistent with tag ranges - check field map/calibration");
      tags[i] = new LimelightHelpers.RawFiducial((int) tagId, data[base + 1], data[base + 2],
          data[base + 3], distance, data[base + 5], ambiguity);
    }
    var pose = new Pose2d(data[0], data[1], Rotation2d.fromDegrees(data[5]));
    return new Decode(new VisionObservation(camera, timestampUs, pose, capture, data[6], count,
        data[8], data[9], data[10], tags), "Valid atomic camera frame");
  }

  private static Decode bad(String reason) { return new Decode(null, reason); }

  public LimelightHelpers.PoseEstimate poseEstimate() {
    return new LimelightHelpers.PoseEstimate(pose, captureTime, latencyMs, tagCount,
        tagSpan, averageDistance, averageArea, tags, false);
  }
}
