package frc.robot.subsystems.drive;

import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.configs.*;
import com.ctre.phoenix6.swerve.*;
import com.ctre.phoenix6.swerve.SwerveModuleConstants.*;
import com.ctre.phoenix6.signals.StaticFeedforwardSignValue;

import com.pathplanner.lib.config.*;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.units.measure.*;

public final class LocalisedSwerveConstants {
    // Smartdashboard Keys
    public static final String kFieldKey = "Field";

    // CAN Ids
    //TODO: done
    private static final int kFrontLeftDriveMotorId = 3;
    private static final int kFrontLeftSteerMotorId = 4;
    private static final int kFrontLeftEncoderId = 33;
    //TODO: done
    private static final int kFrontRightDriveMotorId = 22;
    private static final int kFrontRightSteerMotorId = 9;
    private static final int kFrontRightEncoderId = 47;
    //TODO: done
    private static final int kBackLeftDriveMotorId = 55;
    private static final int kBackLeftSteerMotorId = 61;
    private static final int kBackLeftEncoderId = 62;
    //TODO: done
    private static final int kBackRightDriveMotorId = 19;
    private static final int kBackRightSteerMotorId = 54;
    private static final int kBackRightEncoderId = 46;
    //TODO: done
    private static final int kPigeonId = 1;

    // Gains
    //TODO: done
    private static final Slot0Configs steerGains = new Slot0Configs()
        .withKP(33).withKI(0).withKD(0.5)
        .withKS(0.1).withKV(2.49).withKA(0)
        .withStaticFeedforwardSign(StaticFeedforwardSignValue.UseClosedLoopSign);
    //TODO: done
    private static final Slot0Configs driveGains = new Slot0Configs()
        .withKP(0.1).withKI(0).withKD(0)
        .withKS(0).withKV(0.124);

    // Config
    //TODO: done
    private static final ClosedLoopOutputType kSteerClosedLoopOutput = ClosedLoopOutputType.Voltage;
    private static final ClosedLoopOutputType kDriveClosedLoopOutput = ClosedLoopOutputType.Voltage;
    private static final DriveMotorArrangement kDriveMotorType = DriveMotorArrangement.TalonFX_Integrated;
    private static final SteerMotorArrangement kSteerMotorType = SteerMotorArrangement.TalonFX_Integrated;
    private static final SteerFeedbackType kSteerFeedbackType = SteerFeedbackType.RemoteCANcoder;

    // Ratios
    //TODO: FIXED
    private static final double kCoupleRatio = 3.75;
    private static final double kDriveGearRatio = 5.273475;
    private static final double kSteerGearRatio = 26.09090909090909;
    private static final Distance kWheelRadius = Inches.of(2.05);

    // Friction
    //TODO: done
    private static final Voltage kSteerFrictionVoltage = Volts.of(0.2);
    private static final Voltage kDriveFrictionVoltage = Volts.of(0.2);

    // Electrical Compensation
    //TODO: FIXED
    private static final Current kSlipCurrent = Amps.of(120.0);
    public static final LinearVelocity kSpeedAt12Volts = MetersPerSecond.of(6.00);

    //TODO: Fixed
    private static final boolean kInvertLeftSide = false;
    private static final boolean kInvertRightSide = false;

    // Front Left
    //TODO: fixed
    private static final Distance kFrontLeftXPos = Inches.of(11);
    private static final Distance kFrontLeftYPos = Inches.of(11);
    private static final Angle kFrontLeftEncoderOffset = Rotations.of(0.21630859375);
    private static final boolean kFrontLeftEncoderInverted = false;
    private static final boolean kFrontLeftSteerMotorInverted = false;

    // Front Right
    //TODO: FIXED
    private static final Distance kFrontRightXPos = Inches.of(11);
    private static final Distance kFrontRightYPos = Inches.of(-11);
    private static final Angle kFrontRightEncoderOffset = Rotations.of(-0.07861328125);
    private static final boolean kFrontRightEncoderInverted = false;
    private static final boolean kFrontRightSteerMotorInverted = false;

    // Back Left
    //TODO: FIXED
    private static final Distance kBackLeftXPos = Inches.of(-11);
    private static final Distance kBackLeftYPos = Inches.of(11);
    private static final Angle kBackLeftEncoderOffset = Rotations.of(-0.442626953125);
    private static final boolean kBackLeftEncoderInverted = false;
    private static final boolean kBackLeftSteerMotorInverted = false;

    // Back Right
    //TODO: FIXED
    private static final Distance kBackRightXPos = Inches.of(-11);
    private static final Distance kBackRightYPos = Inches.of(-11);
    private static final Angle kBackRightEncoderOffset = Rotations.of(-0.37841796875);
    private static final boolean kBackRightEncoderInverted = false;
    private static final boolean kBackRightSteerMotorInverted = false;

    //TODO: FIXED
    private static final TalonFXConfiguration driveInitialConfigs = new TalonFXConfiguration();
    private static final TalonFXConfiguration steerInitialConfigs = new TalonFXConfiguration()
        .withCurrentLimits(
            new CurrentLimitsConfigs()
                .withStatorCurrentLimit(Amps.of(60))
                .withStatorCurrentLimitEnable(true)
        );
    private static final CANcoderConfiguration encoderInitialConfigs = new CANcoderConfiguration();

    //TODO: FIXED
    private static final SwerveModuleConstantsFactory<TalonFXConfiguration, TalonFXConfiguration, CANcoderConfiguration> ConstantCreator =
        new SwerveModuleConstantsFactory<TalonFXConfiguration, TalonFXConfiguration, CANcoderConfiguration>()
            .withDriveMotorGearRatio(kDriveGearRatio)
            .withSteerMotorGearRatio(kSteerGearRatio)
            .withCouplingGearRatio(kCoupleRatio)
            .withWheelRadius(kWheelRadius)
            .withSteerMotorGains(steerGains)
            .withDriveMotorGains(driveGains)
            .withSteerMotorClosedLoopOutput(kSteerClosedLoopOutput)
            .withDriveMotorClosedLoopOutput(kDriveClosedLoopOutput)
            .withSlipCurrent(kSlipCurrent)
            .withSpeedAt12Volts(kSpeedAt12Volts)
            .withDriveMotorType(kDriveMotorType)
            .withSteerMotorType(kSteerMotorType)
            .withFeedbackSource(kSteerFeedbackType)
            .withDriveMotorInitialConfigs(driveInitialConfigs)
            .withSteerMotorInitialConfigs(steerInitialConfigs)
            .withEncoderInitialConfigs(encoderInitialConfigs)
            .withSteerFrictionVoltage(kSteerFrictionVoltage)
            .withDriveFrictionVoltage(kDriveFrictionVoltage);

    //TODO: FIXED
    public static final SwerveModuleConstants<TalonFXConfiguration, TalonFXConfiguration, CANcoderConfiguration> FrontLeft =
        ConstantCreator.createModuleConstants(
            kFrontLeftSteerMotorId, kFrontLeftDriveMotorId, kFrontLeftEncoderId, kFrontLeftEncoderOffset,
            kFrontLeftXPos, kFrontLeftYPos, kInvertLeftSide, kFrontLeftSteerMotorInverted, kFrontLeftEncoderInverted
        );
    public static final SwerveModuleConstants<TalonFXConfiguration, TalonFXConfiguration, CANcoderConfiguration> FrontRight =
        ConstantCreator.createModuleConstants(
            kFrontRightSteerMotorId, kFrontRightDriveMotorId, kFrontRightEncoderId, kFrontRightEncoderOffset,
            kFrontRightXPos, kFrontRightYPos, kInvertRightSide, kFrontRightSteerMotorInverted, kFrontRightEncoderInverted
        );
    public static final SwerveModuleConstants<TalonFXConfiguration, TalonFXConfiguration, CANcoderConfiguration> BackLeft =
        ConstantCreator.createModuleConstants(
            kBackLeftSteerMotorId, kBackLeftDriveMotorId, kBackLeftEncoderId, kBackLeftEncoderOffset,
            kBackLeftXPos, kBackLeftYPos, kInvertLeftSide, kBackLeftSteerMotorInverted, kBackLeftEncoderInverted
        );
    public static final SwerveModuleConstants<TalonFXConfiguration, TalonFXConfiguration, CANcoderConfiguration> BackRight =
        ConstantCreator.createModuleConstants(
            kBackRightSteerMotorId, kBackRightDriveMotorId, kBackRightEncoderId, kBackRightEncoderOffset,
            kBackRightXPos, kBackRightYPos, kInvertRightSide, kBackRightSteerMotorInverted, kBackRightEncoderInverted
        );

    //TODO: FIXED
    public static final SwerveDrivetrainConstants DrivetrainConstants = 
        new SwerveDrivetrainConstants().withPigeon2Id(kPigeonId);

    //TODO: FIXED
    private static final ModuleConfig moduleConfig = 
        new ModuleConfig(
            kWheelRadius, kSpeedAt12Volts, 1.0, 
            DCMotor.getKrakenX60(1), 
            Amps.of(60), 1
        );

    //TODO: FIXED
    private static final Translation2d[] kModuleOffsets = {
        new Translation2d(kFrontLeftXPos, kFrontLeftYPos),
        new Translation2d(kFrontRightXPos, kFrontRightYPos),
        new Translation2d(kBackLeftXPos, kBackLeftYPos),
        new Translation2d(kBackRightXPos, kBackRightYPos)
    };

    public static final RobotConfig RobotConfig =
        new RobotConfig(
            Kilogram.of(50), KilogramSquareMeters.of(6.883),
            moduleConfig, kModuleOffsets
        );

    public static final PPHolonomicDriveController PathController =
        new PPHolonomicDriveController(
            new PIDConstants(10, 0, 0),
            new PIDConstants(7, 0, 0)
        );

    public static final LimelightInfo[] kLimelights = {
    };
}
