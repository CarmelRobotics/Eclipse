package frc.robot.subsystems.shooter;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import java.util.function.BooleanSupplier;

/** Select once on the trigger press; acquiring localization must not turn a manual shot mid-hold. */
public final class TeleopShotCommand {
  private TeleopShotCommand() {}

  public static Command select(BooleanSupplier localized, Command automatic, Command manual) {
    return Commands.either(automatic, manual, localized);
  }
}
