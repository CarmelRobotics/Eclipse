package frc.robot.subsystems.shooter;

import static org.junit.jupiter.api.Assertions.*;
import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Translation3d;
import org.junit.jupiter.api.Test;

class VisionHubRangeTest {
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
