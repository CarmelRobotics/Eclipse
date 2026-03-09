package frc.robot.subsystems.localisation;

import com.pathplanner.lib.config.ModuleConfig;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.system.plant.DCMotor;
import frc.robot.subsystems.drive.TunerConstants;

public final class LocalisationConstants {
    public static final String kFieldKey = "Field";

    public static final LimelightInfo[] kLimelights = {
        new LimelightInfo("limelight-four", 0.35, 0, 0.15, 0, 20, 0)
    };

    private static final double kMass = 0;
    private static final double kMOI = 0;
    private static final ModuleConfig ModuleConfig = 
        new ModuleConfig(1.05, 6, 1.2, DCMotor.getKrakenX60(1), 60, 2);

    public static final RobotConfig RobotConfig = 
        new RobotConfig(kMass, kMOI, ModuleConfig, 
            new Translation2d(TunerConstants.kFrontLeftXPos, TunerConstants.kFrontLeftYPos),
            new Translation2d(TunerConstants.kFrontRightXPos, TunerConstants.kFrontRightYPos),
            new Translation2d(TunerConstants.kBackLeftXPos, TunerConstants.kBackRightYPos),
            new Translation2d(TunerConstants.kBackRightXPos, TunerConstants.kBackRightYPos)
    );
    public static final PPHolonomicDriveController PathController = 
        new PPHolonomicDriveController(
            new PIDConstants(10, 0, 0),
            new PIDConstants(7, 0, 0)
        );
}
