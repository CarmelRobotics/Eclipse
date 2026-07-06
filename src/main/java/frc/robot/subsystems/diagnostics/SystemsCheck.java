package frc.robot.subsystems.diagnostics;

import java.util.LinkedHashMap;
import java.util.Map;

import com.ctre.phoenix6.hardware.ParentDevice;

import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.subsystems.drive.CommandSwerveDrivetrain;
import frc.robot.subsystems.lintake.Lintake;
import frc.robot.subsystems.localisation.LimelightHelpers;
import frc.robot.subsystems.localisation.LocalisationConstants;
import frc.robot.subsystems.shooter.Shooter;

/**
 * Robot health diagnostics: a continuous no-motion fault monitor plus an on-demand pit
 * self-test. Both are read-only -- nothing here ever commands a motor, so it's always safe
 * to run, even with people around the robot in the pit.
 *
 * <p>Two products:
 * <ul>
 *   <li><b>Continuous fault monitor</b> ({@link #periodic()}): every loop, publishes a
 *       connected/disconnected flag for every CTRE device plus each Limelight's liveness,
 *       and an aggregate {@code Faults/AnyFault}. This is what catches the "a motor silently
 *       dropped off the CAN bus" failure that is otherwise invisible until something looks
 *       obviously wrong on the field. Deliberately NO controller rumble -- rumble is reserved
 *       for the shot-ready cue, so a fault here shows on the dashboard only.</li>
 *   <li><b>Pit self-test</b> ({@link #fullCheckCommand()}): a one-shot pass/fail sweep of
 *       connectivity + Limelight streaming + gyro signal + battery, collapsed into a single
 *       {@code SystemsCheck/Result} verdict and a human-readable {@code SystemsCheck/Report}.
 *       Bound to a dashboard button and auto-run on entering Test mode.</li>
 * </ul>
 */
public class SystemsCheck extends SubsystemBase {
    // A healthy battery off-load should sit above this in the pit. Under match load the
    // voltage sags well below it, so this bound is only meaningful for the static self-test,
    // never for the continuous monitor.
    private static final double kBatteryPitWarnVolts = 12.0;
    // A Limelight's "hb" NetworkTables value ticks up once per processed frame. If it hasn't
    // advanced in this long, the camera is unplugged, unpowered, or crashed.
    private static final double kHeartbeatStaleSeconds = 1.0;

    private final CommandSwerveDrivetrain m_drivetrain;
    // Every CTRE device on the robot, keyed by a stable human-readable name.
    private final Map<String, ParentDevice> m_devices = new LinkedHashMap<>();

    // Per-camera heartbeat tracking: the last value we saw and when it last changed.
    private final double[] m_lastHeartbeat;
    private final double[] m_lastHeartbeatChange;

    public SystemsCheck(CommandSwerveDrivetrain drivetrain, Shooter shooter, Lintake lintake) {
        m_drivetrain = drivetrain;

        // Drivetrain: 4 modules x (drive motor + steer motor + CANcoder), plus the Pigeon.
        String[] corners = {"FrontLeft", "FrontRight", "BackLeft", "BackRight"};
        for (int i = 0; i < corners.length; i++) {
            var module = drivetrain.getModule(i);
            m_devices.put("Drive/" + corners[i] + "/Drive", module.getDriveMotor());
            m_devices.put("Drive/" + corners[i] + "/Steer", module.getSteerMotor());
            m_devices.put("Drive/" + corners[i] + "/Encoder", module.getEncoder());
        }
        m_devices.put("Drive/Pigeon", drivetrain.getPigeon2());

        // Mechanisms contribute their own device lists so the naming stays with the owner.
        m_devices.putAll(shooter.getDevices());
        m_devices.putAll(lintake.getDevices());

        int cameraCount = LocalisationConstants.kLimelights.length;
        m_lastHeartbeat = new double[cameraCount];
        m_lastHeartbeatChange = new double[cameraCount];
        // Leave change-times at 0 so cameras read "down" until their first real heartbeat
        // arrives after boot -- better a momentary false-down at startup than a false-up.
    }

    @Override
    public void periodic() {
        // --- Continuous fault monitor (no motion, no rumble) ---
        int faultCount = 0;
        StringBuilder offline = new StringBuilder();
        for (Map.Entry<String, ParentDevice> entry : m_devices.entrySet()) {
            boolean connected = entry.getValue().isConnected();
            SmartDashboard.putBoolean("Faults/" + entry.getKey(), connected);
            if (!connected) {
                faultCount++;
                if (offline.length() > 0) offline.append(", ");
                offline.append(entry.getKey());
            }
        }

        for (int i = 0; i < LocalisationConstants.kLimelights.length; i++) {
            String name = LocalisationConstants.kLimelights[i].name();
            if (!isCameraAlive(i, name)) {
                faultCount++;
                if (offline.length() > 0) offline.append(", ");
                offline.append("Limelight/").append(name);
            }
        }

        SmartDashboard.putBoolean("Faults/AnyFault", faultCount > 0);
        SmartDashboard.putNumber("Faults/Count", faultCount);
        SmartDashboard.putString("Faults/Offline", faultCount == 0 ? "none" : offline.toString());
    }

    // True if camera i's heartbeat has advanced within the staleness window. Also updates the
    // stored heartbeat/timestamp, so this must be called once per loop per camera.
    private boolean isCameraAlive(int index, String name) {
        double hb = LimelightHelpers.getLimelightNTDouble(name, "hb");
        if (hb != m_lastHeartbeat[index]) {
            m_lastHeartbeat[index] = hb;
            m_lastHeartbeatChange[index] = Timer.getFPGATimestamp();
        }
        boolean alive = Timer.getFPGATimestamp() - m_lastHeartbeatChange[index] < kHeartbeatStaleSeconds;
        SmartDashboard.putBoolean("Faults/Limelight/" + name, alive);
        return alive;
    }

    /**
     * One-shot pit self-test. Read-only: checks every device is on the bus, every camera is
     * streaming, the gyro signal is valid, and the battery is healthy off-load, then collapses
     * it into a single PASS/FAIL verdict plus a report listing anything wrong. Runs while
     * disabled (ignoringDisable) so you can run it in the pit without enabling the robot.
     */
    public Command fullCheckCommand() {
        return Commands.runOnce(this::runFullCheck).ignoringDisable(true).withName("SystemsCheck");
    }

    private void runFullCheck() {
        StringBuilder report = new StringBuilder();
        int issues = 0;

        // 1. Every CTRE device present on the bus.
        for (Map.Entry<String, ParentDevice> entry : m_devices.entrySet()) {
            boolean connected = entry.getValue().isConnected();
            SmartDashboard.putBoolean("SystemsCheck/" + entry.getKey(), connected);
            if (!connected) {
                issues++;
                report.append("OFFLINE: ").append(entry.getKey()).append('\n');
            }
        }

        // 2. Every Limelight streaming frames.
        for (int i = 0; i < LocalisationConstants.kLimelights.length; i++) {
            String name = LocalisationConstants.kLimelights[i].name();
            boolean alive = isCameraAlive(i, name);
            SmartDashboard.putBoolean("SystemsCheck/Limelight/" + name, alive);
            if (!alive) {
                issues++;
                report.append("NO STREAM: Limelight ").append(name).append('\n');
            }
        }

        // 3. Gyro signal valid (present and finite).
        var yaw = m_drivetrain.getPigeon2().getYaw().refresh();
        boolean gyroOk = yaw.getStatus().isOK() && Double.isFinite(yaw.getValueAsDouble());
        SmartDashboard.putBoolean("SystemsCheck/Gyro", gyroOk);
        if (!gyroOk) {
            issues++;
            report.append("GYRO signal invalid\n");
        }

        // 4. Battery healthy off-load (only meaningful because nothing here draws current).
        double volts = RobotController.getBatteryVoltage();
        boolean batteryOk = volts >= kBatteryPitWarnVolts;
        SmartDashboard.putBoolean("SystemsCheck/Battery", batteryOk);
        SmartDashboard.putNumber("SystemsCheck/BatteryVolts", volts);
        if (!batteryOk) {
            issues++;
            report.append(String.format("LOW BATTERY: %.2f V (want >= %.1f)\n", volts, kBatteryPitWarnVolts));
        }

        boolean pass = issues == 0;
        SmartDashboard.putBoolean("SystemsCheck/Pass", pass);
        SmartDashboard.putString("SystemsCheck/Result",
            pass ? "PASS" : ("FAIL: " + issues + " issue" + (issues == 1 ? "" : "s")));
        SmartDashboard.putString("SystemsCheck/Report",
            pass ? "All systems nominal" : report.toString().trim());
    }
}
