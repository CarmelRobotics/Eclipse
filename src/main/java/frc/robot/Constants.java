package frc.robot;

import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.RotationsPerSecond;

import frc.robot.subsystems.drive.TunerConstants;

public final class Constants {
    public static final double kMaxSpeed = 1.0 * TunerConstants.kSpeedAt12Volts.in(MetersPerSecond);
    public static final double kMaxAngularRate = RotationsPerSecond.of(0.75).in(RadiansPerSecond);
}
