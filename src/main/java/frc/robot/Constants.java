package frc.robot;

import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.RotationsPerSecond;

import frc.robot.subsystems.drive.TunerConstants;

public final class Constants {
    // Competition lockout: dashboard edits cannot re-enable teleop chassis auto-alignment.
    // Autonomous path following and autonomous shooting retain their heading control.
    public static final boolean kTeleopAutoAlignEnabled = false;
    public static final double kMaxSpeed = 1.0 * TunerConstants.kSpeedAt12Volts.in(MetersPerSecond);
    public static final double kMaxAngularRate = RotationsPerSecond.of(1).in(RadiansPerSecond);

    private Constants() {}
}
