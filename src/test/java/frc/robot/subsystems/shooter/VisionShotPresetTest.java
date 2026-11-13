package frc.robot.subsystems.shooter;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertTrue;

import org.junit.jupiter.api.Test;

class VisionShotPresetTest {
  @Test
  void visionShotLowersPivotAndPreservesBallisticHeightAcrossRange() {
    for (double distance : new double[] {1.5, 2.5, 3.5, 4.5, 5.0}) {
      var original = ManualShotPreset.create(distance, 0, 0);
      var vision = ManualShotPreset.vision(distance, 0, 0);
      assertEquals(original.pivotRotations() - 0.008, vision.pivotRotations(), 1e-9);
      double originalAngle = Math.toRadians(70 - 360 * original.pivotRotations());
      double visionAngle = Math.toRadians(70 - 360 * vision.pivotRotations());
      double oldHeightFactor = Math.pow(original.wheelRps() * Math.cos(originalAngle), 2)
          * (distance * Math.tan(originalAngle) - 1.524);
      double newHeightFactor = Math.pow(vision.wheelRps() * Math.cos(visionAngle), 2)
          * (distance * Math.tan(visionAngle) - 1.524);
      assertEquals(oldHeightFactor, newHeightFactor, 1e-6);
      assertTrue(Double.isFinite(vision.wheelRps()));
    }
  }

  @Test
  void trimsAndTowerRemainIndependent() {
    var base = ManualShotPreset.vision(3.5, 0, 0);
    var trimmed = ManualShotPreset.vision(3.5, 0, 2);
    assertEquals(base.wheelRps() + 2, trimmed.wheelRps(), 1e-9);
    assertEquals(ShooterConstants.getScorePivotPosition(ShooterConstants.kTowerShotDistanceMeters),
        ManualShotPreset.tower(0, 0).pivotRotations(), 1e-9);
  }
}
