package frc.robot.subsystems.localisation;

import edu.wpi.first.networktables.DoubleArraySubscriber;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.PubSubOption;

/** Bounded camera IO, polled once per drivetrain loop. A heartbeat is never a pose timestamp. */
public final class VisionSystem implements AutoCloseable {
  private final DoubleArraySubscriber[] poses;
  private final String[] names;
  private final VisionObservation[] latest;
  private final long[] processed;
  private final String[] status;
  private long accepted, rejected;

  public VisionSystem() { this(NetworkTableInstance.getDefault()); }

  public VisionSystem(NetworkTableInstance instance) {
    int count = LocalisationConstants.kLimelights.length;
    poses = new DoubleArraySubscriber[count];
    names = new String[count];
    latest = new VisionObservation[count];
    processed = new long[count];
    status = new String[count];
    for (int i = 0; i < count; i++) {
      names[i] = LocalisationConstants.kLimelights[i].name();
      if (names[i] == null || names[i].isBlank()) {
        status[i] = "Invalid camera name";
        continue;
      }
      poses[i] = instance.getTable(names[i]).getDoubleArrayTopic("botpose_wpiblue")
          .subscribe(new double[0], PubSubOption.keepDuplicates(true),
              PubSubOption.periodic(.02), PubSubOption.pollStorage(1));
      status[i] = "Waiting for botpose_wpiblue";
    }
  }

  public void update(double now) {
    for (int i = 0; i < poses.length; i++) {
      if (poses[i] == null) continue;
      var packet = poses[i].getAtomic();
      if (packet.timestamp == processed[i]) {
        if (latest[i] != null && now - latest[i].captureTime() > VisionObservation.MAX_AGE_SECONDS)
          status[i] = "No fresh pose frames";
        continue;
      }
      processed[i] = packet.timestamp;
      var result = VisionObservation.decode(names[i], packet.timestamp, packet.value, now);
      status[i] = result.reason();
      // A rejected current frame is not silently replaced by an old one. Short
      // target holdover is explicitly owned and time-limited by the aim tracker.
      latest[i] = result.observation();
      if (result.valid()) accepted++; else rejected++;
    }
  }

  public int cameraCount() { return poses.length; }
  public VisionObservation observation(int index) { return index >= 0 && index < latest.length ? latest[index] : null; }
  public String status(int index) { return index >= 0 && index < status.length ? status[index] : "No cameras configured"; }
  public String cameraName(int index) {
    return index >= 0 && index < names.length && names[index] != null ? names[index] : "none";
  }
  public long acceptedFrames() { return accepted; }
  public long rejectedFrames() { return rejected; }
  @Override public void close() { for (var pose : poses) if (pose != null) pose.close(); }
}
