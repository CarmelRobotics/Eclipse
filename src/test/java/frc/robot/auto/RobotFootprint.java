package frc.robot.auto;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;

/** 0.90 m bumper, with the team's 9-inch extension on the REAR only.
 * Full bumper-width intake is conservative; dimensions still need physical verification.
 */
final class RobotFootprint {
  static final double REAR_REACH = .45 + 9 * .0254;

  private RobotFootprint() {}

  static Translation2d[] corners(Pose2d pose) {
    Translation2d[] result = new Translation2d[4];
    int i = 0;
    for (double x : new double[] {-REAR_REACH, .45}) {
      for (double y : new double[] {-.45, .45}) {
        result[i++] = new Translation2d(x, y).rotateBy(pose.getRotation())
            .plus(pose.getTranslation());
      }
    }
    return result;
  }

  static double minX(Pose2d pose) {
    double min = Double.POSITIVE_INFINITY;
    for (var point : corners(pose)) min = Math.min(min, point.getX());
    return min;
  }

  static double maxX(Pose2d pose) {
    double max = Double.NEGATIVE_INFINITY;
    for (var point : corners(pose)) max = Math.max(max, point.getX());
    return max;
  }

  static boolean insideField(Pose2d pose) {
    for (var point : corners(pose)) {
      if (point.getX() < 0 || point.getX() > 16.54
          || point.getY() < 0 || point.getY() > 8.07) return false;
    }
    return true;
  }

  /** Separating-axis test: oriented footprint versus axis-aligned solid obstacle.
   * Avoids false collisions from the empty corners of a rotated AABB.
   */
  static boolean overlaps(Pose2d pose, double x0, double x1, double y0, double y1) {
    var robot = corners(pose);
    var obstacle = new Translation2d[] {new Translation2d(x0, y0),
        new Translation2d(x0, y1), new Translation2d(x1, y0), new Translation2d(x1, y1)};
    double c = pose.getRotation().getCos(), s = pose.getRotation().getSin();
    for (var axis : new Translation2d[] {new Translation2d(1, 0), new Translation2d(0, 1),
        new Translation2d(c, s), new Translation2d(-s, c)}) {
      double rMin = Double.POSITIVE_INFINITY, rMax = Double.NEGATIVE_INFINITY;
      double oMin = Double.POSITIVE_INFINITY, oMax = Double.NEGATIVE_INFINITY;
      for (var p : robot) {
        double projection = p.getX() * axis.getX() + p.getY() * axis.getY();
        rMin = Math.min(rMin, projection);
        rMax = Math.max(rMax, projection);
      }
      for (var p : obstacle) {
        double projection = p.getX() * axis.getX() + p.getY() * axis.getY();
        oMin = Math.min(oMin, projection);
        oMax = Math.max(oMax, projection);
      }
      if (rMax <= oMin || oMax <= rMin) return false;
    }
    return true;
  }
}
