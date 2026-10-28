package frc.robot.subsystems.shooter;

import static org.junit.jupiter.api.Assertions.*;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import frc.robot.subsystems.localisation.LimelightHelpers;
import frc.robot.subsystems.localisation.VisionObservation;
import org.junit.jupiter.api.Test;
import org.junit.jupiter.params.ParameterizedTest;
import org.junit.jupiter.params.provider.ValueSource;

class VisionAimTrackerTest {
  @Test void nonfiniteAndExtremeTrimsCannotProduceNonfiniteTurnCommands() {
    var tracker = new VisionAimTracker();
    acquire(tracker, pose(3, 0));
    for (double trim : new double[] {Double.NaN, Double.NEGATIVE_INFINITY, Double.POSITIVE_INFINITY, -1000, 1000}) {
      double bearing = tracker.bearingErrorRadians(trim);
      assertTrue(Double.isFinite(bearing));
      assertTrue(Math.abs(bearing) <= Math.toRadians(20) + 1e-9);
    }
  }
  private static final ChassisSpeeds STOPPED = new ChassisSpeeds();
  private static Pose2d pose(double distance, double yaw) {
    return new Pose2d(ShooterConstants.kBlueHubPosition.getX() - distance,
        ShooterConstants.kBlueHubPosition.getY(), Rotation2d.fromDegrees(yaw));
  }
  private static VisionObservation frame(Pose2d pose, double time) {
    return new VisionObservation("camera", Math.round(time * 1e6), pose, time, 0,
        2, 1, 2, .5, new LimelightHelpers.RawFiducial[0]);
  }
  private static void acquire(VisionAimTracker tracker, Pose2d pose) {
    for (int i = 0; i < 3; i++) tracker.update(1 + i * .02, STOPPED, frame(pose, 1 + i * .02));
    assertTrue(tracker.valid(), tracker.status());
  }

  @ParameterizedTest
  @ValueSource(doubles = {0, 15, -15, 90, -90, 179, -179})
  void robotRelativeBearingAndConsistentAllianceMirrorHaveSameSign(double yaw) {
    var blue = pose(3, yaw);
    var red = new Pose2d(ShooterConstants.kBlueHubPosition.getX() + ShooterConstants.kRedHubPosition.getX()
        - blue.getX(), 2 * ShooterConstants.kBlueHubPosition.getY() - blue.getY(),
        blue.getRotation().plus(Rotation2d.k180deg));
    var blueVector = VisionAimTracker.nearestHubVector(frame(blue, 1));
    var redVector = VisionAimTracker.nearestHubVector(frame(red, 1));
    assertEquals(3, blueVector.getNorm(), 1e-8);
    assertEquals(0, blueVector.getDistance(redVector), 1e-8);
    assertEquals(0, blueVector.getAngle().minus(Rotation2d.fromDegrees(-yaw)).getRadians(), 1e-8);
  }

  @Test void repeatedReadsOfOneImageDoNotSatisfyAcquisition() {
    var tracker = new VisionAimTracker();
    var image = frame(pose(3, 0), 1);
    for (int i = 0; i < 10; i++) tracker.update(1 + i * .02, STOPPED, image);
    assertFalse(tracker.valid());
    tracker.update(1.20, STOPPED, frame(pose(3, 0), 1.20));
    tracker.update(1.22, STOPPED, frame(pose(3, 0), 1.22));
    assertFalse(tracker.valid(), "A long gap restarts distinct-frame confirmation");
    tracker.update(1.24, STOPPED, frame(pose(3, 0), 1.24));
    assertTrue(tracker.valid());
  }

  @Test void shortLossAdvancesWithRobotMotionAndExpiresWithoutCamera() {
    var tracker = new VisionAimTracker();
    acquire(tracker, pose(3, 0));
    for (int i = 1; i <= 5; i++) tracker.update(1.04 + i * .02, new ChassisSpeeds(1, 0, 0), null);
    assertEquals(2.9, tracker.distanceMeters(), 1e-8);
    assertTrue(tracker.valid());
    for (int i = 6; i <= 17; i++) tracker.update(1.04 + i * .02, STOPPED, null);
    assertFalse(tracker.valid());
  }

  @Test void measuredChassisRotationUpdatesBearingWithoutGlobalHeadingOrNewTags() {
    var tracker = new VisionAimTracker();
    acquire(tracker, pose(3, 0));
    tracker.update(1.06, new ChassisSpeeds(0, 0, 1), null);
    assertEquals(-.02, tracker.bearingErrorRadians(0), 1e-8);
    assertEquals(3, tracker.distanceMeters(), 1e-8);
  }

  @Test void inconsistentHeadingCannotCauseUncommandedHalfTurnAndRequiresRearm() {
    var tracker = new VisionAimTracker();
    acquire(tracker, pose(3, 0));
    for (int i = 1; i <= 3; i++) {
      double t = 1.04 + i * .02;
      tracker.update(t, STOPPED, frame(pose(3, 180), t));
    }
    assertTrue(tracker.faultLatched());
    assertFalse(tracker.valid());
    for (int i = 4; i < 20; i++) {
      double t = 1.04 + i * .02;
      tracker.update(t, STOPPED, frame(pose(3, 180), t));
    }
    assertFalse(tracker.valid(), "A held trigger must not silently re-arm a contradictory target");
    tracker.rearm();
    for (int i = 20; i < 23; i++) {
      double t = 1.04 + i * .02;
      tracker.update(t, STOPPED, frame(pose(3, 0), t));
    }
    assertTrue(tracker.valid(), tracker.status());
  }

  @Test void staleNonfiniteMotionAndLoopStallsDoNotProduceAimCommands() {
    var tracker = new VisionAimTracker();
    acquire(tracker, pose(3, 0));
    tracker.update(1.06, new ChassisSpeeds(Double.NaN, 0, 0), null);
    assertFalse(tracker.valid());
    var another = new VisionAimTracker();
    acquire(another, pose(3, 0));
    another.update(2, STOPPED, frame(pose(3, 0), 1.04));
    assertFalse(another.valid());
  }

  @Test void latencyAndTranslationAreAppliedExactlyOnce() {
    var tracker = new VisionAimTracker();
    for (int i = 0; i < 3; i++) {
      double capture = 1 + i * .02;
      tracker.update(capture + .05, new ChassisSpeeds(1, 0, 0), frame(pose(3 - i * .02, 0), capture));
    }
    assertTrue(tracker.valid());
    assertEquals(2.91, tracker.distanceMeters(), .025);
  }

  @Test void longMatchFaultInjectionKeepsOneTargetAndRecoversAfterShortOcclusions() {
    var tracker = new VisionAimTracker();
    int valid = 0;
    for (int i = 0; i < 15_000; i++) {
      double t = 1 + i * .02;
      boolean occluded = i % 200 >= 195;
      tracker.update(t, STOPPED, occluded ? null : frame(pose(3 + .01 * Math.sin(i), 0), t));
      if (tracker.valid()) {
        valid++;
        assertTrue(tracker.distanceMeters() > 2.95 && tracker.distanceMeters() < 3.05);
        assertTrue(Double.isFinite(tracker.bearingErrorRadians(0)));
      }
    }
    assertTrue(valid > 14_900);
    assertNotNull(tracker.robotRelativeVector());
  }
}
