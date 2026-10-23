package frc.robot.auto;

import com.pathplanner.lib.config.RobotConfig;
import edu.wpi.first.math.kinematics.ChassisSpeeds;

/**
 * Approximate planar force envelope, not a calibrated digital twin.
 * Uses the repo's motor/gearing/mass, 120 A stator and the selected auto supply budget.
 * Supply current and motor/torque current are NOT interchangeable at low duty cycle.
 * No hardware current limits are changed by this test utility.
 */
final class DrivePhysicsModel {
  private DrivePhysicsModel() {}

  static double accelerationLimit(RobotConfig config, ChassisSpeeds robotSpeed,
      boolean braking, double batteryVolts) {
    double maxWheelSpeed = 0;
    for (var module : config.toSwerveModuleStates(robotSpeed)) {
      maxWheelSpeed = Math.max(maxWheelSpeed, Math.abs(module.speedMetersPerSecond));
    }
    var module = config.moduleConfig;
    var motor = module.driveMotor; // Already includes the drive reduction.
    double emf = maxWheelSpeed / module.wheelRadiusMeters / motor.KvRadPerSecPerVolt;
    // I_supply = I_motor * V_motor / V_battery; V_motor = emf + I_motor * R.
    double supplyLimited = (-emf + Math.sqrt(emf * emf
        + 4 * motor.rOhms * batteryVolts * module.driveCurrentLimit)) / (2 * motor.rOhms);
    double voltageLimited = Math.max(0, (batteryVolts - emf) / motor.rOhms);
    double motorCurrent = braking ? 120 : Math.min(120, Math.min(supplyLimited, voltageLimited));
    double torque = Math.max(0, motor.getTorque(motorCurrent) - (braking ? 0 : module.torqueLoss));
    double forcePerWheel = torque / module.wheelRadiusMeters;
    double frictionPerWheel = module.wheelCOF * config.massKG * 9.81 / 4;
    // Reserve some tire force for steering/rotation rather than assuming all four
    // modules always contribute their entire straight-line force to translation.
    return 0.80 * Math.min(forcePerWheel, frictionPerWheel) * 4 / config.massKG;
  }
}
