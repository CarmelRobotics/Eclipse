package frc.robot;

import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.RotationsPerSecond;

import frc.robot.subsystems.drive.TunerConstants;

public final class Constants {
    // Chassis auto-aim restored by request; validated vision and driver override gate it.
    public static final boolean kTeleopAutoAlignEnabled = true;
    public static final double kMaxSpeed = 1.0 * TunerConstants.kSpeedAt12Volts.in(MetersPerSecond);
    public static final double kMaxAngularRate = RotationsPerSecond.of(1).in(RadiansPerSecond);

    private Constants() {}
}
