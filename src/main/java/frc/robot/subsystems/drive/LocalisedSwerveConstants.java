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
    private static final int kFrontLeftDriveMotorId = 0;
    private static final int kFrontLeftSteerMotorId = 1;
    private static final int kFrontLeftEncoderId = 2;

    private static final int kFrontRightDriveMotorId = 3;
    private static final int kFrontRightSteerMotorId = 4;
    private static final int kFrontRightEncoderId = 5;

    private static final int kBackLeftDriveMotorId = 6;
    private static final int kBackLeftSteerMotorId = 7;
    private static final int kBackLeftEncoderId = 8;

    private static final int kBackRightDriveMotorId = 9;
    private static final int kBackRightSteerMotorId = 10;
    private static final int kBackRightEncoderId = 11;

    private static final int kPigeonId = 12;

    // Gains
    private static final Slot0Configs steerGains = new Slot0Configs()
        .withKP(25).withKI(0).withKD(0.5)
        .withKS(0.1).withKV(1.5).withKA(0)
        .withStaticFeedforwardSign(StaticFeedforwardSignValue.UseClosedLoopSign);
    
    private static final Slot0Configs driveGains = new Slot0Configs()
        .withKP(0.1).withKI(0).withKD(0)
        .withKS(0).withKV(0.124);

    // Config
    private static final ClosedLoopOutputType kSteerClosedLoopOutput = ClosedLoopOutputType.Voltage;
    private static final ClosedLoopOutputType kDriveClosedLoopOutput = ClosedLoopOutputType.Voltage;
    private static final DriveMotorArrangement kDriveMotorType = DriveMotorArrangement.TalonFX_Integrated;
    private static final SteerMotorArrangement kSteerMotorType = SteerMotorArrangement.TalonFX_Integrated;
    private static final SteerFeedbackType kSteerFeedbackType = SteerFeedbackType.RemoteCANcoder;

    // Ratios
    private static final double kCoupleRatio = 4.909090909090909;
    private static final double kDriveGearRatio = 6.976076555023923;
    private static final double kSteerGearRatio = 12.1;
    private static final Distance kWheelRadius = Inches.of(2.05);

    // Friction
    private static final Voltage kSteerFrictionVoltage = Volts.of(0.2);
    private static final Voltage kDriveFrictionVoltage = Volts.of(0.2);

    // Electrical Compensation
    private static final Current kSlipCurrent = Amps.of(120.0);
    private static final LinearVelocity kSpeedAt12Volts = MetersPerSecond.of(4.69);

    private static final boolean kInvertLeftSide = true;
    private static final boolean kInvertRightSide = false;

    // Front Left
    private static final Distance kFrontLeftXPos = Inches.of(1.175);
    private static final Distance kFrontLeftYPos = Inches.of(1.175);
    private static final Angle kFrontLeftEncoderOffset = Rotations.of(0.352783203125);
    private static final boolean kFrontLeftEncoderInverted = false;
    private static final boolean kFrontLeftSteerMotorInverted = true;

    // Front Right
    private static final Distance kFrontRightXPos = Inches.of(1.175);
    private static final Distance kFrontRightYPos = Inches.of(-1.175);
    private static final Angle kFrontRightEncoderOffset = Rotations.of(-0.1533203125);
    private static final boolean kFrontRightEncoderInverted = false;
    private static final boolean kFrontRightSteerMotorInverted = true;

    // Back Left
    private static final Distance kBackLeftXPos = Inches.of(-1.175);
    private static final Distance kBackLeftYPos = Inches.of(1.175);
    private static final Angle kBackLeftEncoderOffset = Rotations.of(0.492919921875);
    private static final boolean kBackLeftEncoderInverted = false;
    private static final boolean kBackLeftSteerMotorInverted = true;

    // Back Right
    private static final Distance kBackRightXPos = Inches.of(-1.175);
    private static final Distance kBackRightYPos = Inches.of(-1.175);
    private static final Angle kBackRightEncoderOffset = Rotations.of(0.225341796875);
    private static final boolean kBackRightEncoderInverted = false;
    private static final boolean kBackRightSteerMotorInverted = true;

    private static final TalonFXConfiguration driveInitialConfigs = new TalonFXConfiguration();
    private static final TalonFXConfiguration steerInitialConfigs = new TalonFXConfiguration()
        .withCurrentLimits(
            new CurrentLimitsConfigs()
                .withStatorCurrentLimit(Amps.of(60))
                .withStatorCurrentLimitEnable(true)
        );
    private static final CANcoderConfiguration encoderInitialConfigs = new CANcoderConfiguration();

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

    public static final SwerveDrivetrainConstants DrivetrainConstants = 
        new SwerveDrivetrainConstants().withPigeon2Id(kPigeonId);

    private static final ModuleConfig moduleConfig = 
        new ModuleConfig(
            kWheelRadius, kSpeedAt12Volts, 1.0, 
            DCMotor.getKrakenX60(1), 
            Amps.of(60), 1
        );

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
        new LimelightInfo("limelight-left", 0, 0, 0, 0, 0, 0),
        new LimelightInfo("limelight-right", 0, 0, 0, 0, 0, 0)
    };
}
