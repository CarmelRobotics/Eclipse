package frc.robot.subsystems.strategy;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.drive.CommandSwerveDrivetrain;
import frc.robot.subsystems.shooter.ShooterConstants;

/**
 * Centralizes field-region classification and the default action for each region.
 * Geometry is blue-origin, matching the drivetrain pose and shooter targets.
 */
public final class ZoneController extends SubsystemBase {
  public enum CurrentZone {
    ALLIANCE,
    BUMP,
    TRENCH,
    NEUTRAL_LEFT,
    NEUTRAL_RIGHT,
    OPPONENT_LEFT,
    OPPONENT_RIGHT,
    UNKNOWN
  }

  public enum ZoneAction {
    SHOOT_HUB,
    FERRY_LEFT,
    FERRY_RIGHT,
    STOW_SHOOTER
  }

  private static final double kBlueBarrierMinX = 4.02;
  private static final double kBlueBarrierMaxX = 5.23;
  private static final double kRedBarrierMinX = 11.31;
  private static final double kRedBarrierMaxX = 12.52;
  private static final double kTrenchDepth = 1.668;
  private static final double kBarrierMargin = 0.30;
  private static final double kSideHysteresis = 0.50;

  private final CommandSwerveDrivetrain m_drivetrain;
  private CurrentZone m_currentZone = CurrentZone.UNKNOWN;
  private ZoneAction m_action = ZoneAction.SHOOT_HUB;
  private boolean m_highSide = false;

  public ZoneController(CommandSwerveDrivetrain drivetrain) {
    m_drivetrain = drivetrain;
  }

  @Override
  public void periodic() {
    double y = m_drivetrain.getState().Pose.getY();
    if (m_highSide && y < ShooterConstants.kFieldWidthMeters / 2.0 - kSideHysteresis) {
      m_highSide = false;
    } else if (!m_highSide && y > ShooterConstants.kFieldWidthMeters / 2.0 + kSideHysteresis) {
      m_highSide = true;
    }

    m_currentZone = classifyZone(m_drivetrain.getState().Pose.getX(), y);
    m_action = determineAction(m_currentZone);
    SmartDashboard.putString("zone/current", m_currentZone.toString());
    SmartDashboard.putString("zone/action", m_action.toString());
    SmartDashboard.putBoolean("zone/high side", m_highSide);
  }

  public CurrentZone getCurrentZone() {
    return m_currentZone;
  }

  public ZoneAction getAction() {
    return m_action;
  }

  public boolean shouldShootHub() {
    return m_action == ZoneAction.SHOOT_HUB;
  }

  public boolean shouldFerry() {
    return m_action == ZoneAction.FERRY_LEFT || m_action == ZoneAction.FERRY_RIGHT;
  }

  public CurrentZone classifyZone(double x, double y) {
    if (inBarrier(x, y)) {
      if (inTrenchCorridor(y)) {
        return CurrentZone.TRENCH;
      }
      // The hub-side half of the barrier remains a legal hub-shot region. The
      // opposite half is treated like neutral field for the default action.
      return m_drivetrain.isInAllianceZone() ? CurrentZone.BUMP : CurrentZone.UNKNOWN;
    }
    if (m_drivetrain.isInAllianceZone()) {
      return CurrentZone.ALLIANCE;
    }

    boolean left = !m_highSide;
    boolean red = m_drivetrain.getHubPosition().equals(ShooterConstants.kRedHubPosition);
    if (red) {
      return left ? CurrentZone.OPPONENT_LEFT : CurrentZone.OPPONENT_RIGHT;
    }
    return left ? CurrentZone.NEUTRAL_LEFT : CurrentZone.NEUTRAL_RIGHT;
  }

  public ZoneAction determineAction(CurrentZone zone) {
    return switch (zone) {
      case ALLIANCE, BUMP -> ZoneAction.SHOOT_HUB;
      case TRENCH -> ZoneAction.STOW_SHOOTER;
      case NEUTRAL_LEFT, OPPONENT_LEFT -> ZoneAction.FERRY_LEFT;
      case NEUTRAL_RIGHT, OPPONENT_RIGHT -> ZoneAction.FERRY_RIGHT;
      case UNKNOWN -> ZoneAction.FERRY_LEFT;
    };
  }

  private boolean inBarrier(double x, double y) {
    boolean inBlue = x >= kBlueBarrierMinX - kBarrierMargin
        && x <= kBlueBarrierMaxX + kBarrierMargin;
    boolean inRed = x >= kRedBarrierMinX - kBarrierMargin
        && x <= kRedBarrierMaxX + kBarrierMargin;
    return (inBlue || inRed) && y >= 0.0 && y <= ShooterConstants.kFieldWidthMeters;
  }

  private boolean inTrenchCorridor(double y) {
    return y <= kTrenchDepth || y >= ShooterConstants.kFieldWidthMeters - kTrenchDepth;
  }
}
