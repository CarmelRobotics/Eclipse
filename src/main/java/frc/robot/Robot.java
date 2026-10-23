// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.util.LoggedTunableNumber;

public class Robot extends TimedRobot {
  private Command m_autonomousCommand;
  private int m_diagLoopCount = 0;  // decimates robotPeriodic dashboard writes to 10 Hz

  private final RobotContainer m_robotContainer;

  public Robot() {
    // NO DOGLOG / NO STRUCTURED LOGGING (deliberate). DogLog 2026.5.0 crashed the robot
    // with a NATIVE out-of-memory (pthread_create EAGAIN), including mid-match on
    // 2026-07-14: its logExtras option (default TRUE) polls the radio's HTTP status page
    // via java.net.http.HttpClient every cycle (verified by decompiling the jar), and when
    // that page doesn't answer, the client's thread pool grows without bound until the JVM
    // can't spawn threads. It looks like a heap leak but isn't -- no Java-side mitigation
    // (GC calls, less logging) touches it. Root-caused and removed 2026-07-11; the removal
    // was lost in the 2026-07-14 revert to the 07-09 code and the crash immediately
    // returned, so it's now removed again -- vendor dep deleted, all call sites stripped.
    // If a vendor logging library is ever reconsidered, audit what it does with the
    // network first. Live telemetry is SmartDashboard-only.

    // Dashboard tunables (ShotTuning/*) are live. Set false for competition so a stray
    // dashboard edit can't change gains or offsets mid-match.
    LoggedTunableNumber.setTuningMode(true);

    // Brownout guard: on a roboRIO 2 this lets the controller ride battery sag down to
    // 6.0 V instead of cutting outputs at the 6.75 V default -- transient dips during a
    // full-power shot-while-driving stay survivable. (No-op on a RIO 1.)
    try {
      RobotController.setBrownoutVoltage(6.0);
    } catch (Exception e) {
      // RIO 1 or unsupported image; the default threshold applies.
    }

    m_robotContainer = new RobotContainer();
  }

  @Override
  public void robotPeriodic() {
    CommandScheduler.getInstance().run();

    // Battery health, live at 10 Hz: if shots start missing low or the robot stutters,
    // check these first. (Live-only since DogLog's removal -- watch them AT the field;
    // there is no log file to check after the fact.)
    if (++m_diagLoopCount % 5 == 0) {
      SmartDashboard.putNumber("Battery/Voltage", RobotController.getBatteryVoltage());
      SmartDashboard.putBoolean("Battery/BrownedOut", RobotController.isBrownedOut());
    }
  }

  @Override
  public void disabledInit() {}

  @Override
  public void disabledPeriodic() {}

  @Override
  public void disabledExit() {}

  @Override
  public void autonomousInit() {
    m_autonomousCommand = m_robotContainer.getAutonomousCommand();

    if (m_autonomousCommand != null) {
      CommandScheduler.getInstance().schedule(m_autonomousCommand);
    }
  }

  @Override
  public void autonomousPeriodic() {}

  @Override
  public void autonomousExit() {}

  @Override
  public void teleopInit() {
    if (m_autonomousCommand != null) {
      m_autonomousCommand.cancel();
    }
  }

  @Override
  public void teleopPeriodic() {}

  @Override
  public void teleopExit() {}

  @Override
  public void testInit() {
    CommandScheduler.getInstance().cancelAll();
  }

  @Override
  public void testPeriodic() {}

  @Override
  public void testExit() {}
}
