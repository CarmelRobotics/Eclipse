package frc.robot.subsystems.localisation;

import static org.junit.jupiter.api.Assertions.*;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import java.util.Optional;
import org.junit.jupiter.api.Test;

class FieldLocalizationTest {
  private static final Pose2d RED = new Pose2d(14, 2, Rotation2d.k180deg);
  private static final Pose2d BLUE = new Pose2d(2.541, 6.069, Rotation2d.kZero);
  private static final ChassisSpeeds STOPPED = new ChassisSpeeds();

  @Test void redWithoutAutoRejectsMirroredStartingPosition() {
    var guard = staged(Alliance.Red);
    for (int i = 0; i < 12; i++) {
      double time = 1 + .04 * i;
      assertEquals(FieldLocalization.Update.REJECT,
          guard.evaluate(frame(BLUE, time), time, Pose2d.kZero, STOPPED, true));
    }
    assertFalse(guard.positionKnown());
    assertFalse(guard.readyToAim());
    assertTrue(guard.status().contains("Wrong starting half"));
  }

  @Test void blueWithoutAutoRejectsRedSideMirrorToo() {
    var guard = staged(Alliance.Blue);
    assertEquals(FieldLocalization.Update.REJECT,
        guard.evaluate(frame(RED, 1), 1, Pose2d.kZero, STOPPED, true));
    assertFalse(guard.readyToAim());
  }

  @Test void correctRedVisionAcquiresPositionWithoutAutoButDoesNotInventHeading() {
    var guard = staged(Alliance.Red);
    assertEquals(FieldLocalization.Update.SEED_TRANSLATION,
        guard.evaluate(frame(RED, 1), 1, Pose2d.kZero, STOPPED, true));
    assertTrue(guard.positionKnown());
    assertFalse(guard.headingKnown());
    assertFalse(guard.readyToAim());
    guard.seedHeading();
    assertTrue(guard.readyToAim());
    assertEquals(FieldLocalization.Update.FUSE,
        guard.evaluate(frame(RED, 2), 2, RED, STOPPED, false));
  }

  @Test void fullAutoPoseInitializesBothHeadingAndPositionOnBothAlliances() {
    for (var alliance : Alliance.values()) {
      var guard = staged(alliance);
      guard.seedPose(alliance == Alliance.Red ? RED : BLUE);
      assertTrue(guard.readyToAim());
    }
  }

  @Test void missingAllianceNeverMeansBlueAndLastKnownRedSurvivesMissingPackets() {
    var guard = new FieldLocalization();
    guard.seedPose(RED);
    assertFalse(guard.readyToAim());
    assertTrue(guard.alliance().isEmpty());
    guard.observeAlliance(Optional.of(Alliance.Red), true, true, RED);
    guard.observeAlliance(Optional.empty(), false, false, RED);
    assertTrue(guard.isRed());
    assertTrue(guard.readyToAim());
  }

  @Test void learningRedAfterPitInitializationInvalidatesBlueSidePose() {
    var guard = new FieldLocalization();
    guard.seedPose(BLUE);
    guard.observeAlliance(Optional.of(Alliance.Red), true, true, BLUE);
    assertFalse(guard.positionKnown());
    assertFalse(guard.readyToAim());
  }

  @Test void enabledMirroredVisionCannotTeleportTrustedOdometry() {
    var guard = staged(Alliance.Red);
    guard.seedPose(RED);
    for (int i = 0; i < 20; i++) {
      double time = 1 + .04 * i;
      assertEquals(FieldLocalization.Update.REJECT,
          guard.evaluate(frame(BLUE, time), time, RED, STOPPED, false));
    }
    assertTrue(guard.readyToAim(), "Reject bad vision, not the already-known odometry");
    assertTrue(guard.status().contains("gyro heading"));
  }

  @Test void closeConsistentMultiTagVisionRecoversFromOutOfFieldOdometryWhileMoving() {
    var guard = staged(Alliance.Red);
    guard.seedPose(RED);
    // Robot-relative -X is field +X when the gyro points 180 degrees.
    var motion = new ChassisSpeeds(-2, 0, 0);
    for (int i = 0; i < 4; i++) {
      double now = 1 + .04 * i;
      double capturedAt = now - .05;
      var vision = frame(new Pose2d(14 + 2 * (capturedAt - 1), 2, Rotation2d.k180deg), capturedAt);
      var badOdometry = new Pose2d(15.5 + 2 * (now - 1), 8.3, Rotation2d.k180deg);
      assertEquals(FieldLocalization.Update.FUSE,
          guard.evaluate(vision, now, badOdometry, motion, false));
    }
    assertTrue(guard.readyToAim());
    assertTrue(guard.status().contains("immediately"));
  }

  @Test void closeConsistentSingleTagVisionAlsoRecoversLargeDrift() {
    var guard = staged(Alliance.Red);
    guard.seedPose(RED);
    var wrongOdometry = new Pose2d(15, 7, Rotation2d.k180deg);
    for (int i = 0; i < 5; i++) {
      double time = 1 + .04 * i;
      assertEquals(FieldLocalization.Update.FUSE,
          guard.evaluate(singleFrame(RED, time, .08), time, wrongOdometry, STOPPED, false));
    }
    assertTrue(guard.status().contains("immediately"));
  }

  @Test void largeVisionCorrectionAcceptsValidatedSingleAndMultiTagFramesImmediately() {
    var guard = staged(Alliance.Red);
    guard.seedPose(RED);
    var wrongOdometry = new Pose2d(15, 7, Rotation2d.k180deg);
    assertEquals(FieldLocalization.Update.FUSE,
        guard.evaluate(singleFrame(RED, 1, .19), 1, wrongOdometry, STOPPED, false));
    for (int i = 0; i < 3; i++) {
      double time = 1.1 + .04 * i;
      assertEquals(FieldLocalization.Update.FUSE,
          guard.evaluate(frame(RED, time), time, wrongOdometry, STOPPED, false));
    }
    assertEquals(FieldLocalization.Update.FUSE,
        guard.evaluate(frame(new Pose2d(13.2, 2, Rotation2d.k180deg), 1.22), 1.22,
            wrongOdometry, STOPPED, false));
    assertTrue(guard.status().contains("immediately"));
  }

  @Test void crossingMidfieldDuringPlayDoesNotMirrorOrRejectTheOtherHalf() {
    var guard = staged(Alliance.Red);
    guard.seedPose(RED);
    var otherHalf = new Pose2d(7, 3, Rotation2d.kZero);
    assertEquals(FieldLocalization.Update.FUSE,
        guard.evaluate(frame(otherHalf, 1), 1, otherHalf, STOPPED, false));
    assertTrue(guard.readyToAim());
  }

  @Test void tagLossPreservesOdometryAndReadiness() {
    var guard = staged(Alliance.Red);
    guard.seedPose(RED);
    assertEquals(FieldLocalization.Update.REJECT,
        guard.evaluate(null, 20, RED, STOPPED, false));
    assertTrue(guard.readyToAim());
  }

  @Test void firstValidVisionFrameSeedsPositionWithoutInventingHeading() {
    var guard = staged(Alliance.Red);
    var sample = frame(RED, 1);
    assertEquals(FieldLocalization.Update.SEED_TRANSLATION,
        guard.evaluate(sample, 1, Pose2d.kZero, new ChassisSpeeds(2, 0, 0), false));
    assertTrue(guard.positionKnown());
    assertFalse(guard.headingKnown());
    assertEquals(FieldLocalization.Update.REJECT,
        guard.evaluate(sample, 1.02, Pose2d.kZero, STOPPED, false));
  }

  @Test void rejectsStaleFutureNonfiniteOutOfBoundsAndUncertainSingleTags() {
    var guard = staged(Alliance.Red);
    var single = frame(RED, 2);
    single.tagCount = 1;
    single.rawFiducials = new LimelightHelpers.RawFiducial[] {
        new LimelightHelpers.RawFiducial(1, 0, 0, .5, 2, 2, .8)};
    assertEquals(FieldLocalization.Update.REJECT,
        guard.evaluate(single, 2, Pose2d.kZero, STOPPED, true));
    assertEquals(FieldLocalization.Update.REJECT,
        guard.evaluate(frame(RED, 1), 3, Pose2d.kZero, STOPPED, true));
    assertEquals(FieldLocalization.Update.REJECT,
        guard.evaluate(frame(RED, 4), 3, Pose2d.kZero, STOPPED, true));
    assertEquals(FieldLocalization.Update.REJECT,
        guard.evaluate(frame(new Pose2d(Double.NaN, 2, Rotation2d.kZero), 3.1),
            3.1, Pose2d.kZero, STOPPED, true));
    assertEquals(FieldLocalization.Update.REJECT,
        guard.evaluate(frame(new Pose2d(20, 2, Rotation2d.kZero), 3.2),
            3.2, Pose2d.kZero, STOPPED, true));
    assertFalse(guard.positionKnown());
  }

  @Test void movingRobotCanAcquirePositionFromValidVision() {
    var guard = staged(Alliance.Red);
    assertEquals(FieldLocalization.Update.SEED_TRANSLATION,
        guard.evaluate(frame(RED, 1), 1, Pose2d.kZero, new ChassisSpeeds(2, 0, 0), false));
    assertTrue(guard.positionKnown());
  }

  @Test void fieldHeadingResetIsIndependentOfReversedDriverPerspective() {
    assertEquals(0, FieldLocalization.downfieldHeading(Alliance.Blue).getDegrees(), 1e-9);
    assertEquals(180, FieldLocalization.downfieldHeading(Alliance.Red).getDegrees(), 1e-9);
  }

  private static FieldLocalization staged(Alliance alliance) {
    var guard = new FieldLocalization();
    guard.observeAlliance(Optional.of(alliance), true, true, Pose2d.kZero);
    guard.prepareForMatch(Pose2d.kZero, true);
    return guard;
  }

  static LimelightHelpers.PoseEstimate frame(Pose2d pose, double time) {
    return new LimelightHelpers.PoseEstimate(pose, time, 20, 2, 1, 2, .5,
        new LimelightHelpers.RawFiducial[0], false);
  }

  private static LimelightHelpers.PoseEstimate singleFrame(Pose2d pose, double time,
      double ambiguity) {
    var estimate = frame(pose, time);
    estimate.tagCount = 1;
    estimate.rawFiducials = new LimelightHelpers.RawFiducial[] {
        new LimelightHelpers.RawFiducial(1, 0, 0, .5, 2, 2, ambiguity)};
    return estimate;
  }
}
