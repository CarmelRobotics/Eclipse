package frc.robot;

import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.RotationsPerSecond;

import frc.robot.subsystems.drive.TunerConstants;
import frc.robot.util.LimelightInfo;

public final class Constants {
    public static final int kControllerPort = 0;

    public static final double kMaxSpeed = 1.0 * TunerConstants.kSpeedAt12Volts.in(MetersPerSecond);
    public static final double kMaxAngularRate =  RotationsPerSecond.of(1).in(RadiansPerSecond);

    public static final LimelightInfo[] kLimelights = {
        new LimelightInfo("limelight-four", 0.35, 0, 0.15, 0, 20, 0)
    };
}
