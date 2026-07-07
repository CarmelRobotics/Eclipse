package frc.robot.subsystems.power;

/**
 * A named power budget: per-motor-group supply current limits (amps) that the
 * {@link PowerManager} applies as one unit based on what the robot is doing.
 *
 * <p>The key idea (from team 581): these ceilings are <b>situational, not simultaneous</b>.
 * The robot never needs full drive power and full flywheel power at the same instant, so
 * each mode hands the battery to whatever matters right now instead of budgeting for the
 * worst case everywhere at once (which is what a single static limit must do).
 *
 * <p>{@code IDLE} equals the compiled static defaults (drive 40 A from TunerConstants,
 * flywheel 35 A from ShooterConstants), so it is the safe baseline the manager falls back to
 * whenever the feature is disabled.
 */
public enum PowerMode {
  /** Baseline = the static compiled limits. */
  IDLE(40, 35),
  /** Aiming/shooting: the drive is mostly holding a heading, so give the flywheel headroom
   *  to hold speed through a volley and pull the drive budget back. */
  SCORING(25, 50),
  /** Sprinting across the field: the flywheel is only idling, so let the drive pull hard. */
  SPRINT(65, 20),
  /** Autonomous: collect-and-score cycles need both healthy. */
  AUTO(55, 45);

  final double driveSupplyCurrent;
  final double flywheelSupplyCurrent;

  PowerMode(double driveSupplyCurrent, double flywheelSupplyCurrent) {
    this.driveSupplyCurrent = driveSupplyCurrent;
    this.flywheelSupplyCurrent = flywheelSupplyCurrent;
  }
}
