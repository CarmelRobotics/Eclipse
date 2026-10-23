package frc.robot.auto;

import static org.junit.jupiter.api.Assertions.*;

import com.pathplanner.lib.util.FlippingUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import org.junit.jupiter.api.Test;

class RobotFootprintTest {
  @Test
  void extensionExistsOnlyOnRearAndRotatesWithRobot() {
    var forward = new Pose2d(2, 2, Rotation2d.kZero);
    assertEquals(2 - .6786, RobotFootprint.minX(forward), 1e-9);
    assertEquals(2 + .45, RobotFootprint.maxX(forward), 1e-9);
    var reverse = new Pose2d(2, 2, Rotation2d.k180deg);
    assertEquals(2 - .45, RobotFootprint.minX(reverse), 1e-9);
    assertEquals(2 + .6786, RobotFootprint.maxX(reverse), 1e-9);
    var sideways = new Pose2d(4.6, 7.5453, Rotation2d.fromDegrees(90));
    assertTrue(RobotFootprint.insideField(sideways));
    assertEquals(4.15, RobotFootprint.minX(sideways), 1e-9);
    assertEquals(5.05, RobotFootprint.maxX(sideways), 1e-9);
    assertFalse(RobotFootprint.insideField(new Pose2d(4.6, 7.65, Rotation2d.fromDegrees(90))));
  }

  @Test
  void separatingAxisDetectsRealOverlapButNotEmptyAabbCorners() {
    var pose = new Pose2d(2, 2, Rotation2d.fromDegrees(45));
    assertTrue(RobotFootprint.overlaps(pose, 2.1, 2.2, 2.1, 2.2));
    assertTrue(RobotFootprint.overlaps(pose, 0, 4, 0, 4));
    assertFalse(RobotFootprint.overlaps(pose, 2.55, 2.62, 2.55, 2.62));
    assertFalse(RobotFootprint.overlaps(pose, 4, 5, 4, 5));
  }

  @Test
  void rotationalAllianceFlipPreservesEnvelopeAndCollision() {
    var blue = new Pose2d(4.6, 7.49, Rotation2d.fromDegrees(92));
    var red = FlippingUtil.flipFieldPose(blue);
    assertEquals(RobotFootprint.insideField(blue), RobotFootprint.insideField(red));
    assertEquals(16.54 - RobotFootprint.maxX(blue), RobotFootprint.minX(red), 1e-9);
    assertEquals(RobotFootprint.overlaps(blue, 4.02, 5.24, 6.4, 6.792),
        RobotFootprint.overlaps(red, 16.54 - 5.24, 16.54 - 4.02, 8.07 - 6.792, 8.07 - 6.4));
  }
}
