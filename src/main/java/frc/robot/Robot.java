// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import dev.doglog.DogLog;
import dev.doglog.DogLogOptions;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.util.LoggedTunableNumber;

public class Robot extends TimedRobot {
  private Command m_autonomousCommand;

  private final RobotContainer m_robotContainer;

  public Robot() {
    // DogLog writes all logged signals to a WPILOG file (open later in AdvantageScope)
    // and mirrors them live to NetworkTables. captureDs also records joystick/DS data.
    // Set ntPublish(false) for competition if NT bandwidth becomes a concern.
    DogLog.setOptions(new DogLogOptions().withNtPublish(true).withCaptureDs(true));
    DogLog.setEnabled(true);

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

    // Battery health in every log: if shots start missing low or the robot stutters,
    // check these traces first -- sag is invisible on the dashboard after the fact.
    DogLog.log("Battery/Voltage", RobotController.getBatteryVoltage());
    DogLog.log("Battery/BrownedOut", RobotController.isBrownedOut());
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
