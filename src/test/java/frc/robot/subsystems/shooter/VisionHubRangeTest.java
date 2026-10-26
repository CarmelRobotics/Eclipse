package frc.robot.subsystems.shooter;

import static org.junit.jupiter.api.Assertions.*;
import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.networktables.NetworkTableInstance;
import frc.robot.subsystems.localisation.LocalisationConstants;
import org.junit.jupiter.api.Test;

class VisionHubRangeTest {
  @Test void liveHeartbeatAllowsUnchangedPoseAndIndependentQualityTimestamps() {
    try (var instance = NetworkTableInstance.create()) {
      var table = instance.getTable(LocalisationConstants.kLimelights[0].name());
      table.getEntry("tv").setDouble(1);
      table.getEntry("tid").setDouble(26);
      table.getEntry("tl").setDouble(0);
      table.getEntry("cl").setDouble(0);
      var pose = table.getDoubleArrayTopic("botpose_targetspace").getEntry(new double[0]);
      var raw = table.getDoubleArrayTopic("rawfiducials").getEntry(new double[0]);
      var heartbeat = table.getDoubleTopic("hb").getEntry(0);
      var range = new VisionHubRange(instance);
      pose.set(new double[] {0, 1.12395, 2.396242, 0, 0, 0}, 10_000_000);
      raw.set(new double[] {26, 0, 0, 2, 3, 3, .05}, 9_800_000);
      heartbeat.set(1, 10_000_000);
      range.update(10);
      assertTrue(range.isValid(), range.status());
      assertEquals(3, range.distanceMeters(), 1e-8);
      // Neither pose nor quality needs a value change while stationary. The camera
      // continues capturing; heartbeat advances while the arrays retain their values.
      heartbeat.set(2, 11_000_000);
      range.update(11);
      assertTrue(range.isValid(), range.status());
      assertEquals(3, range.distanceMeters(), 1e-8);
      // A new unrelated array update must not make a dead camera heartbeat fresh.
      pose.set(new double[] {0, 1.12395, 2.4, 0, 0, 0}, 11_300_000);
      range.update(11.31);
      assertFalse(range.isValid());
      assertEquals("Stale camera heartbeat", range.status());
    }
  }

  @Test void heartbeatCannotReuseOldPoseWhenPrimaryTagChanges() {
    try (var instance = NetworkTableInstance.create()) {
      var table = instance.getTable(LocalisationConstants.kLimelights[0].name());
      table.getEntry("tv").setDouble(1);
      table.getEntry("tid").setDouble(26);
      table.getEntry("tl").setDouble(0);
      table.getEntry("cl").setDouble(0);
      var pose = table.getDoubleArrayTopic("botpose_targetspace").getEntry(new double[0]);
      var raw = table.getDoubleArrayTopic("rawfiducials").getEntry(new double[0]);
      var heartbeat = table.getDoubleTopic("hb").getEntry(0);
      var range = new VisionHubRange(instance);
      pose.set(new double[] {0, 1.12395, 2.396242, 0, 0, 0}, 10_000_000);
      raw.set(new double[] {26, 0, 0, 2, 3, 3, .05}, 10_000_000);
      heartbeat.set(1, 10_000_000);
      range.update(10);
      assertTrue(range.isValid());
      table.getEntry("tid").setDouble(25);
      heartbeat.set(2, 10_500_000);
      range.update(10.5);
      assertFalse(range.isValid());
      assertEquals("Waiting for new tag pose", range.status());
      table.getEntry("tid").setDouble(31);
      range.update(10.5);
      assertFalse(range.isValid());
      assertEquals("Visible tag is not a hub tag", range.status());
      assertEquals(31, range.observedTagId());
    }
  }

  @Test void camerasWithoutHeartbeatStillRequireFreshPose() {
    try (var instance = NetworkTableInstance.create()) {
      var table = instance.getTable(LocalisationConstants.kLimelights[0].name());
      table.getEntry("tv").setDouble(1);
      table.getEntry("tid").setDouble(26);
      table.getEntry("tl").setDouble(0);
      table.getEntry("cl").setDouble(0);
      var pose = table.getDoubleArrayTopic("botpose_targetspace").getEntry(new double[0]);
      var raw = table.getDoubleArrayTopic("rawfiducials").getEntry(new double[0]);
      var range = new VisionHubRange(instance);
      pose.set(new double[] {0, 1.12395, 2.396242, 0, 0, 0}, 10_000_000);
      raw.set(new double[] {26, 0, 0, 2, 3, 3, .05}, 9_000_000);
      range.update(10);
      assertTrue(range.isValid(), range.status());
      range.update(10.3);
      assertFalse(range.isValid());
      assertEquals("Stale tag pose", range.status());
    }
  }

  @Test void allHubFacesAndOffsetTagsRecoverHorizontalCenterRange() {
    var layout = AprilTagFieldLayout.loadField(AprilTagFields.k2026RebuiltWelded);
    for (int id : new int[] {2, 3, 4, 5, 8, 9, 10, 11, 18, 19, 20, 21, 24, 25, 26, 27}) {
      var tag = layout.getTagPose(id).orElseThrow();
      double cx = layout.getTagPose(id < 17 ? 2 : 18).orElseThrow().getX();
      double cy = layout.getTagPose(id < 17 ? 10 : 26).orElseThrow().getY();
      for (double[] offset : new double[][] {{-3, 1}, {3, -.5}, {0, 3}, {0, -3}}) {
        var robot = new Translation3d(cx + offset[0], cy + offset[1], 0);
        var local = robot.minus(tag.getTranslation()).rotateBy(tag.getRotation().unaryMinus());
        double[] ll = {local.getY(), -local.getZ(), local.getX(), 0, 0, 0};
        assertEquals(Math.hypot(offset[0], offset[1]),
            VisionHubRange.rangeFromRobotInTagSpace(id, ll), 1e-8, "tag " + id);
      }
    }
    assertEquals(3, VisionHubRange.rangeFromRobotInTagSpace(26,
        new double[] {0, 1.12395, 2.396242, 0, 0, 0}), 1e-8);
  }

  @Test void rejectsNonHubTagsMalformedFramesAndBadQuality() {
    double[] good = {26, 0, 0, 2, 3, 3, .05};
    assertTrue(VisionHubRange.goodTag(26, good));
    assertFalse(VisionHubRange.goodTag(31, new double[] {31, 0, 0, 2, 3, 3, .05}));
    assertFalse(VisionHubRange.goodTag(26, new double[] {26, 0, 0, 2, 3, 3, .8}));
    assertFalse(VisionHubRange.goodTag(26, new double[] {26, 0, 0, 2, 3, 3, Double.NaN}));
    assertFalse(VisionHubRange.goodTag(26, new double[] {26, 0, 0, 2, -1, 3, .05}));
    assertFalse(VisionHubRange.goodTag(26, new double[0]));
    assertFalse(VisionHubRange.goodTag(26, new double[6]));
    assertTrue(Double.isNaN(VisionHubRange.rangeFromRobotInTagSpace(31, new double[6])));
    assertTrue(Double.isNaN(VisionHubRange.rangeFromRobotInTagSpace(26, new double[0])));
    assertTrue(Double.isNaN(VisionHubRange.rangeFromRobotInTagSpace(26,
        new double[] {0, 1, Double.NaN, 0, 0, 0})));
  }
}
