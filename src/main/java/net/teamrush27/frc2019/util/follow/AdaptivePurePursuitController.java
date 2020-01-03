package net.teamrush27.frc2019.util.follow;

import net.teamrush27.frc2019.util.math.Pose2d;
import net.teamrush27.frc2019.util.math.Rotation2d;
import net.teamrush27.frc2019.util.math.Translation2d;
import net.teamrush27.frc2019.util.math.Twist2d;

/**
 * Implements an adaptive pure pursuit controller. See:
 * https://www.ri.cmu.edu/pub_files/pub1/kelly_alonzo_1994_4/kelly_alonzo_1994_4 .pdf
 *
 * <p>Basically, we find a spot on the path we'd like to follow and calculate the arc necessary to
 * make us land on that spot. The target spot is a specified distance ahead of us, and we look
 * further ahead the greater our tracking error. We also return the maximum speed we'd like to be
 * going when we reach the target spot.
 */
public class AdaptivePurePursuitController {
  private static final double REALLY_BIG_NUMBER = 1E6;

  public static class Command {
    public Twist2d delta = Twist2d.identity();
    public double crossTrackError;
    public double maxVelocity;
    public double endVelocity;
    public Translation2d lookaheadPoint;
    public double remainingPathLength;

    public Command() {}

    public Command(
        Twist2d delta,
        double crossTrackError,
        double maxVelocity,
        double endVelocity,
        Translation2d lookaheadPoint,
        double remainingPathLength) {
      this.delta = delta;
      this.crossTrackError = crossTrackError;
      this.maxVelocity = maxVelocity;
      this.endVelocity = endVelocity;
      this.lookaheadPoint = lookaheadPoint;
      this.remainingPathLength = remainingPathLength;
    }
  }

  Path path;
  boolean atEndOfPath = false;
  final boolean reversed;
  final Lookahead lookahead;

  public AdaptivePurePursuitController(Path path, boolean reversed, Lookahead lookahead) {
    this.path = path;
    this.reversed = reversed;
    this.lookahead = lookahead;
  }

  /**
   * Gives the Pose2d.Delta that the robot should take to follow the path
   *
   * @param pose robot pose
   * @return movement command for the robot to follow
   */
  public Command update(Pose2d pose) {
    if (reversed) {
      pose =
          new Pose2d(
              pose.getTranslation(), pose.getRotation().rotateBy(Rotation2d.fromRadians(Math.PI)));
    }

    final Path.TargetPointReport report = path.getTargetPoint(pose.getTranslation(), lookahead);
    if (isFinished()) {
      // Stop.
      return new Command(
          Twist2d.identity(),
          report.closestPointDistance,
          report.maxSpeed,
          0.0,
          report.lookaheadPoint,
          report.remainingPathDistance);
    }

    final Arc arc = new Arc(pose, report.lookaheadPoint);
    double scaleFactor = 1.0;
    // Ensure we don't overshoot the end of the path (once the lookahead speed drops to zero).
    if (report.lookaheadPointSpeed < 1E-6 && report.remainingPathDistance < arc.length) {
      scaleFactor = Math.max(0.0, report.remainingPathDistance / arc.length);
      atEndOfPath = true;
    } else {
      atEndOfPath = false;
    }
    if (reversed) {
      scaleFactor *= -1;
    }

    return new Command(
        new Twist2d(
            scaleFactor * arc.length,
            0.0,
            arc.length
                * getDirection(pose, report.lookaheadPoint)
                * Math.abs(scaleFactor)
                / arc.radius),
        report.closestPointDistance,
        report.maxSpeed,
        report.lookaheadPointSpeed * Math.signum(scaleFactor),
        report.lookaheadPoint,
        report.remainingPathDistance);
  }

  public boolean hasPassedMarker(String marker) {
    return path.hasPassedMarker(marker);
  }

  public static class Arc {
    public Translation2d center;
    public double radius;
    public double length;

    public Arc(Pose2d pose, Translation2d point) {
      center = getCenter(pose, point);
      radius = new Translation2d(center, point).norm();
      length = getLength(pose, point, center, radius);
    }
  }

  /**
   * Gives the center of the circle joining the lookahead point and robot pose
   *
   * @param pose robot pose
   * @param point lookahead point
   * @return center of the circle joining the lookahead point and robot pose
   */
  public static Translation2d getCenter(Pose2d pose, Translation2d point) {
    final Translation2d poseToPointHalfway = pose.getTranslation().interpolate(point, 0.5);
    final Rotation2d normal =
        pose.getTranslation().inverse().translateBy(poseToPointHalfway).direction().normal();
    final Pose2d perpendicularBisector = new Pose2d(poseToPointHalfway, normal);
    final Pose2d normalFromPose = new Pose2d(pose.getTranslation(), pose.getRotation().normal());
    if (normalFromPose.isColinear(perpendicularBisector.normal())) {
      // Special case: center is poseToPointHalfway.
      return poseToPointHalfway;
    }
    return normalFromPose.intersection(perpendicularBisector);
  }

  /**
   * Gives the radius of the circle joining the lookahead point and robot pose
   *
   * @param pose robot pose
   * @param point lookahead point
   * @return radius of the circle joining the lookahead point and robot pose
   */
  public static double getRadius(Pose2d pose, Translation2d point) {
    Translation2d center = getCenter(pose, point);
    return new Translation2d(center, point).norm();
  }

  /**
   * Gives the length of the arc joining the lookahead point and robot pose (assuming forward
   * motion).
   *
   * @param pose robot pose
   * @param point lookahead point
   * @return the length of the arc joining the lookahead point and robot pose
   */
  public static double getLength(Pose2d pose, Translation2d point) {
    final double radius = getRadius(pose, point);
    final Translation2d center = getCenter(pose, point);
    return getLength(pose, point, center, radius);
  }

  public static double getLength(
      Pose2d pose, Translation2d point, Translation2d center, double radius) {
    if (radius < REALLY_BIG_NUMBER) {
      final Translation2d centerToPoint = new Translation2d(center, point);
      final Translation2d centerToPose = new Translation2d(center, pose.getTranslation());
      // If the point is behind pose, we want the opposite of this angle. To determine if the point
      // is behind,
      // check the sign of the cross-product between the normal vector and the vector from pose to
      // point.
      final boolean behind =
          Math.signum(
                  Translation2d.cross(
                      pose.getRotation().normal().toTranslation(),
                      new Translation2d(pose.getTranslation(), point)))
              > 0.0;
      final Rotation2d angle = Translation2d.getAngle(centerToPose, centerToPoint);
      return radius
          * (behind ? 2.0 * Math.PI - Math.abs(angle.getRadians()) : Math.abs(angle.getRadians()));
    } else {
      return new Translation2d(pose.getTranslation(), point).norm();
    }
  }

  /**
   * Gives the direction the robot should turn to stay on the path
   *
   * @param pose robot pose
   * @param point lookahead point
   * @return the direction the robot should turn: -1 is left, +1 is right
   */
  public static int getDirection(Pose2d pose, Translation2d point) {
    Translation2d poseToPoint = new Translation2d(pose.getTranslation(), point);
    Translation2d robot = pose.getRotation().toTranslation();
    double cross = robot.x() * poseToPoint.y() - robot.y() * poseToPoint.x();
    return (cross < 0) ? -1 : 1; // if robot < pose turn left
  }

  /** @return has the robot reached the end of the path */
  public boolean isFinished() {
    return atEndOfPath;
  }
}
