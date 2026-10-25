package frc.robot.subsystems.shooter;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.networktables.DoubleArrayEntry;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import frc.robot.subsystems.localisation.LocalisationConstants;

/** Hub range in tag space: no field robot pose, alliance selection, gyro, or odometry. */
public final class VisionHubRange {
  public static final double MAX_AGE_SECONDS = .25;
  private static final Translation2d[] HUB_IN_TAG_XZ = hubOffsets();
  private final Camera[] cameras;
  private double distance = Double.NaN;
  private double acceptedTimestamp = Double.NEGATIVE_INFINITY;
  private int hub;
  private boolean valid;

  public VisionHubRange() {
    cameras = new Camera[LocalisationConstants.kLimelights.length];
    for (int i = 0; i < cameras.length; i++) {
      cameras[i] = new Camera(NetworkTableInstance.getDefault()
          .getTable(LocalisationConstants.kLimelights[i].name()));
    }
  }

  // LL target space: X right, Y down, Z out of the tag. WPILib tag space:
  // X out of tag, Y right as viewed facing tag, Z up. Only X/Z are horizontal.
  // https://docs.limelightvision.io/docs/docs-limelight/pipeline-apriltag/apriltag-coordinate-systems
  private static Translation2d[] hubOffsets() {
    var layout = AprilTagFieldLayout.loadField(AprilTagFields.k2026RebuiltWelded);
    var offsets = new Translation2d[33];
    for (int id : new int[] {2, 3, 4, 5, 8, 9, 10, 11, 18, 19, 20, 21, 24, 25, 26, 27}) {
      var tag = layout.getTagPose(id).orElseThrow();
      // Center-face tag coordinates locate each hub center; shifted face tags
      // retain their lateral offset instead of treating every tag as centered.
      double x = layout.getTagPose(id < 17 ? 2 : 18).orElseThrow().getX();
      double y = layout.getTagPose(id < 17 ? 10 : 26).orElseThrow().getY();
      var center = new Translation3d(x, y, tag.getZ()).minus(tag.getTranslation())
          .rotateBy(tag.getRotation().unaryMinus());
      offsets[id] = new Translation2d(center.getY(), center.getX());
    }
    return offsets;
  }

  static double rangeFromRobotInTagSpace(int id, double[] pose) {
    if (id < 0 || id >= HUB_IN_TAG_XZ.length || HUB_IN_TAG_XZ[id] == null || pose.length < 6)
      return Double.NaN;
    for (double value : pose) if (!Double.isFinite(value)) return Double.NaN;
    var center = HUB_IN_TAG_XZ[id];
    return Math.hypot(pose[0] - center.getX(), pose[2] - center.getY());
  }

  public void update(double now) {
    valid = false;
    double nearest = Double.POSITIVE_INFINITY;
    double timestamp = 0;
    int selectedHub = 0;
    for (Camera camera : cameras) {
      double reportedId = camera.id.getDouble(-1);
      if (!Double.isFinite(reportedId) || reportedId != Math.rint(reportedId)) continue;
      int id = (int) reportedId;
      if (camera.visible.getDouble(0) != 1) continue;
      var frame = camera.pose.getAtomic();
      var raw = camera.raw.getAtomic();
      double latency = (camera.pipelineLatency.getDouble(Double.NaN)
          + camera.captureLatency.getDouble(Double.NaN)) / 1000;
      double capture = frame.timestamp / 1e6 - latency;
      if (!Double.isFinite(latency) || latency < 0 || latency > MAX_AGE_SECONDS
          || now - capture < -.05 || now - capture > MAX_AGE_SECONDS
          || Math.abs(raw.timestamp - frame.timestamp) > 50_000
          || !goodTag(id, raw.value)) continue;
      double range = rangeFromRobotInTagSpace(id, frame.value);
      if (range >= 1.5 && range <= ShooterConstants.kMaxShotDistanceMeters && range < nearest) {
        nearest = range;
        timestamp = capture;
        selectedHub = id < 17 ? 1 : 2;
      }
    }
    if (!Double.isFinite(nearest)) return;
    if (timestamp < acceptedTimestamp) return;
    if (timestamp != acceptedTimestamp || hub != selectedHub) {
      boolean continuous = Double.isFinite(distance) && hub == selectedHub
          && now - acceptedTimestamp <= MAX_AGE_SECONDS;
      // Reject single-frame jumps; after a genuine interruption reacquire afresh.
      if (continuous && Math.abs(nearest - distance) > .75) return;
      distance = continuous ? distance + .25 * (nearest - distance) : nearest;
      acceptedTimestamp = timestamp;
      hub = selectedHub;
    }
    valid = true;
  }

  static boolean goodTag(int id, double[] raw) {
    if (id < 0 || id >= HUB_IN_TAG_XZ.length || HUB_IN_TAG_XZ[id] == null || raw.length % 7 != 0)
      return false;
    for (int i = 0; i + 6 < raw.length; i += 7) {
      if (raw[i] != id) continue;
      double range = raw[i + 4], ambiguity = raw[i + 6];
      return Double.isFinite(range) && range > 0 && range <= 6
          && Double.isFinite(ambiguity) && ambiguity >= 0 && ambiguity <= .2;
    }
    return false;
  }

  public boolean isValid() { return valid; }
  public double distanceMeters() { return distance; }

  private static final class Camera {
    final NetworkTableEntry visible, id, pipelineLatency, captureLatency;
    final DoubleArrayEntry pose, raw;
    Camera(NetworkTable table) {
      visible = table.getEntry("tv");
      id = table.getEntry("tid");
      pipelineLatency = table.getEntry("tl");
      captureLatency = table.getEntry("cl");
      pose = table.getDoubleArrayTopic("botpose_targetspace").getEntry(new double[0]);
      raw = table.getDoubleArrayTopic("rawfiducials").getEntry(new double[0]);
    }
  }
}
