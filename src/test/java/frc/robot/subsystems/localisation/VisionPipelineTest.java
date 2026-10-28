package frc.robot.subsystems.localisation;

import static org.junit.jupiter.api.Assertions.*;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.NetworkTablesJNI;
import edu.wpi.first.networktables.PubSubOption;
import java.net.InetAddress;
import java.net.ServerSocket;
import java.util.Arrays;
import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import org.junit.jupiter.api.Test;
import org.junit.jupiter.params.ParameterizedTest;
import org.junit.jupiter.params.provider.ValueSource;

class VisionPipelineTest {
  @Test void realNt4LoopbackTransportPreservesAtomicFramesAndCaptureClock() throws Exception {
    int port;
    try (var probe = new ServerSocket(0, 1, InetAddress.getLoopbackAddress())) { port = probe.getLocalPort(); }
    try (var server = NetworkTableInstance.create(); var client = NetworkTableInstance.create();
        var system = new VisionSystem(server)) {
      server.startServer("", "127.0.0.1", 0, port);
      client.startClient4("competition-vision-test");
      client.setServer("127.0.0.1", port);
      long deadline = System.nanoTime() + 2_000_000_000L;
      while (!client.isConnected() && System.nanoTime() < deadline) Thread.sleep(5);
      assertTrue(client.isConnected(), "Local NT4 client must connect without DS or robot network");
      var topic = LocalisationConstants.kLimelights[0].name();
      try (var publisher = client.getTable(topic).getDoubleArrayTopic("botpose_wpiblue")
              .publish(PubSubOption.keepDuplicates(true), PubSubOption.periodic(.02));
          var received = server.getTable(topic).getDoubleArrayTopic("botpose_wpiblue")
              .subscribe(new double[0], PubSubOption.keepDuplicates(true))) {
        long previous = 0;
        for (int i = 0; i < 4; i++) {
          publisher.set(single(31));
          client.flush();
          deadline = System.nanoTime() + 2_000_000_000L;
          while (received.getAtomic().timestamp <= previous && System.nanoTime() < deadline) Thread.sleep(5);
          var packet = received.getAtomic();
          assertTrue(packet.timestamp > previous);
          assertArrayEquals(single(31), packet.value, 0);
          system.update(NetworkTablesJNI.now() / 1e6);
          assertNotNull(system.observation(0), system.status(0));
          assertEquals(packet.timestamp / 1e6 - .02, system.observation(0).captureTime(), 1e-6);
          previous = packet.timestamp;
        }
        assertEquals(4, system.acceptedFrames());
      }
    }
  }

  private static double[] single(int id) {
    return new double[] {2, 4, 0, 0, 0, 10, 20, 1, 0, 2, .5,
        id, 0, 0, .5, 2, 2, .05};
  }

  @ParameterizedTest
  @ValueSource(ints = {1,2,3,4,5,6,7,8,9,10,11,12,13,14,15,16,
      17,18,19,20,21,22,23,24,25,26,27,28,29,30,31,32})
  void allFieldTagsCanSupplyOneAtomicPoseNotOnlyHubTags(int id) {
    var data = single(id);
    var tag = AprilTagFieldLayout.loadField(AprilTagFields.k2026RebuiltWelded).getTagPose(id).orElseThrow();
    // A physically consistent robot location 2 m in 3D from this tag, on its visible side.
    double horizontalRange = Math.sqrt(4 - tag.getZ() * tag.getZ());
    data[0] = tag.getX() + Math.cos(tag.getRotation().getZ()) * horizontalRange;
    data[1] = tag.getY() + Math.sin(tag.getRotation().getZ()) * horizontalRange;
    data[5] = Math.toDegrees(tag.getRotation().getZ()) + 180;
    var decoded = VisionObservation.decode("camera", 1_000_000, data, 1);
    assertTrue(decoded.valid(), decoded.reason());
    assertEquals(.98, decoded.observation().captureTime(), 1e-9);
    assertEquals(id, decoded.observation().tags()[0].id);
    assertEquals(data[0], decoded.observation().pose().getX(), 1e-9);
  }

  @Test void rejectsMalformedNonfiniteImpossibleAndUnreliableFrames() {
    for (double[] malformed : new double[][] {new double[0], new double[10], new double[1000]})
      assertFalse(VisionObservation.decode("camera", 1_000_000, malformed, 1).valid());
    var good = single(31);
    for (int i = 0; i < good.length; i++) {
      var bad = good.clone();
      bad[i] = Double.NaN;
      assertFalse(VisionObservation.decode("camera", 1_000_000, bad, 1).valid(), "NaN index " + i);
    }
    int[] indices = {0, 1, 2, 3, 4, 6, 7, 8, 9, 10, 11, 15, 17};
    double[] values = {20, -2, .5, 40, -40, 250, 1.5, -1, 8, 0, 33, 0, .8};
    for (int i = 0; i < indices.length; i++) {
      var bad = good.clone();
      bad[indices[i]] = values[i];
      assertFalse(VisionObservation.decode("camera", 1_000_000, bad, 1).valid(), "index " + indices[i]);
    }
    assertFalse(VisionObservation.decode("camera", 1_000_000, good, 1.5).valid());
    assertFalse(VisionObservation.decode("camera", 2_000_000, good, 1).valid());
    assertFalse(VisionObservation.decode("camera", 0, good, 1).valid());
    assertFalse(VisionObservation.decode("camera", 1_000_000, Arrays.copyOf(good, 11), 1).valid());
    var wrongOrigin = good.clone();
    wrongOrigin[0] = 14; // In-field but physically inconsistent with the observed blue tower tag.
    assertFalse(VisionObservation.decode("camera", 1_000_000, wrongOrigin, 1).valid());
  }

  @Test void acceptsStandardMultiTagHeaderButRejectsIncompleteOrDuplicateMetadata() {
    var header = new double[] {2, 4, 0, 0, 0, 0, 20, 2, 1, 2, .5};
    assertTrue(VisionObservation.decode("camera", 1_000_000, header, 1).valid());
    var incomplete = Arrays.copyOf(header, 24);
    assertFalse(VisionObservation.decode("camera", 1_000_000, incomplete, 1).valid());
    var duplicate = Arrays.copyOf(header, 25);
    for (int i : new int[] {11, 18}) {
      duplicate[i] = 31;
      duplicate[i + 3] = .5;
      duplicate[i + 4] = 2;
      duplicate[i + 5] = 2;
      duplicate[i + 6] = .05;
    }
    assertFalse(VisionObservation.decode("camera", 1_000_000, duplicate, 1).valid());
  }

  @Test void identicalPoseFramesAreFreshButHeartbeatCannotReviveFrozenGeometry() {
    try (var instance = NetworkTableInstance.create(); var system = new VisionSystem(instance)) {
      var table = instance.getTable(LocalisationConstants.kLimelights[0].name());
      try (var publisher = table.getDoubleArrayTopic("botpose_wpiblue")
          .publish(PubSubOption.keepDuplicates(true))) {
        publisher.set(single(31), 1_000_000);
        system.update(1);
        assertEquals(1, system.acceptedFrames());
        assertNotNull(system.observation(0));
        publisher.set(single(31), 1_040_000); // Bit-identical camera values, distinct timestamp.
        system.update(1.04);
        assertEquals(2, system.acceptedFrames());
        assertEquals(1.02, system.observation(0).captureTime(), 1e-9);
        table.getEntry("hb").setDouble(123);
        system.update(2);
        assertEquals(2, system.acceptedFrames());
        assertEquals("No fresh pose frames", system.status(0));
        assertEquals(1.02, system.observation(0).captureTime(), 1e-9);
        publisher.set(new double[11], 2_010_000);
        system.update(2.01);
        assertNull(system.observation(0));
        assertEquals(1, system.rejectedFrames());
      }
    }
  }

  @Test void onlyBlueOriginTopicIsSubscribedAndStorageRemainsBoundedForLongRuns() {
    try (var instance = NetworkTableInstance.create(); var system = new VisionSystem(instance)) {
      var table = instance.getTable(LocalisationConstants.kLimelights[0].name());
      table.getEntry("botpose_wpired").setDoubleArray(single(16));
      system.update(1);
      assertNull(system.observation(0));
      try (var publisher = table.getDoubleArrayTopic("botpose_wpiblue")
          .publish(PubSubOption.keepDuplicates(true))) {
        for (int i = 1; i <= 20_000; i++) {
          long timeUs = 1_000_000L + i * 20_000L;
          publisher.set(single(31), timeUs);
          system.update(timeUs / 1e6);
        }
        assertEquals(20_000, system.acceptedFrames());
        assertEquals(LocalisationConstants.kLimelights.length, system.cameraCount());
        assertEquals(401_000_000L, system.observation(0).frameId());
      }
    }
  }
}
