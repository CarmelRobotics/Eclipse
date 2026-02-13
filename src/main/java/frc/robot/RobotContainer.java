// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.climb.Climb;
import frc.robot.subsystems.drive.LocalisedSwerveDrivetrain;
import frc.robot.subsystems.lintake.Lintake;
import frc.robot.subsystems.shooter.Shooter;

public class RobotContainer {
  private final LocalisedSwerveDrivetrain m_drivetrain = new LocalisedSwerveDrivetrain();
  private final Shooter m_shooter = new Shooter(m_drivetrain::getPose);
  private final Lintake m_lintake = new Lintake();
  private final Climb m_climb = new Climb();

  public RobotContainer() {
    configureBindings();
  }

  private void configureBindings() {}

  public Command getAutonomousCommand() {
    return Commands.print("No autonomous command configured");
  }
}
