package net.teamrush27.frc2019.util.follow;

import java.util.Optional;
import net.teamrush27.frc2019.constants.FollowingConstants;
import net.teamrush27.frc2019.util.math.Rotation2d;
import net.teamrush27.frc2019.util.math.Translation2d;
import net.teamrush27.frc2019.util.motion.MotionProfile;
import net.teamrush27.frc2019.util.motion.MotionProfileConstraints;
import net.teamrush27.frc2019.util.motion.MotionProfileGenerator;
import net.teamrush27.frc2019.util.motion.MotionProfileGoal;
import net.teamrush27.frc2019.util.motion.MotionState;
import org.apache.logging.log4j.LogManager;
import org.apache.logging.log4j.Logger;

/** Class representing a segment of the robot's autonomous path. */
public class PathSegment {
  private static final Logger LOG = LogManager.getLogger(PathSegment.class);

  private Translation2d start;
  private Translation2d end;
  private Translation2d center;
  private Translation2d deltaStart;
  private Translation2d deltaEnd;
  private double maxSpeed;
  private boolean isLine;
  private MotionProfile speedController;
  private boolean extrapolateLookahead;
  private String marker;

  /**
   * Constructor for a linear segment
   *
   * @param startX start x
   * @param startY start y
   * @param endX end x
   * @param endY end y
   * @param maxSpeed maximum speed allowed on the segment
   */
  public PathSegment(
      double startX,
      double startY,
      double endX,
      double endY,
      double maxSpeed,
      MotionState startState,
      double endSpeed) {
    this.start = new Translation2d(startX, startY);
    this.end = new Translation2d(endX, endY);

    this.deltaStart = new Translation2d(start, end);

    this.maxSpeed = maxSpeed;
    extrapolateLookahead = false;
    isLine = true;
    createMotionProfiler(startState, endSpeed);
  }

  public PathSegment(
      double startX,
      double startY,
      double endX,
      double endY,
      double maxSpeed,
      MotionState startState,
      double endSpeed,
      String marker) {
    this.start = new Translation2d(startX, startY);
    this.end = new Translation2d(endX, endY);

    this.deltaStart = new Translation2d(start, end);

    this.maxSpeed = maxSpeed;
    extrapolateLookahead = false;
    isLine = true;
    this.marker = marker;
    createMotionProfiler(startState, endSpeed);
  }

  /**
   * Constructor for an arc segment
   *
   * @param startX start x
   * @param startY start y
   * @param endX end x
   * @param endY end y
   * @param centerX center x
   * @param centerY center y
   * @param maxSpeed maximum speed allowed on the segment
   */
  public PathSegment(
      double startX,
      double startY,
      double endX,
      double endY,
      double centerX,
      double centerY,
      double maxSpeed,
      MotionState startState,
      double endSpeed) {
    this.start = new Translation2d(startX, startY);
    this.end = new Translation2d(endX, endY);
    this.center = new Translation2d(centerX, centerY);

    this.deltaStart = new Translation2d(center, start);
    this.deltaEnd = new Translation2d(center, end);

    this.maxSpeed = maxSpeed;
    extrapolateLookahead = false;
    isLine = false;
    createMotionProfiler(startState, endSpeed);
  }

  public PathSegment(
      double startX,
      double startY,
      double endX,
      double endY,
      double centerX,
      double centerY,
      double maxSpeed,
      MotionState startState,
      double endSpeed,
      String marker) {
    this.start = new Translation2d(startX, startY);
    this.end = new Translation2d(endX, endY);
    this.center = new Translation2d(centerX, centerY);

    this.deltaStart = new Translation2d(center, start);
    this.deltaEnd = new Translation2d(center, end);

    this.maxSpeed = maxSpeed;
    extrapolateLookahead = false;
    isLine = false;
    this.marker = marker;
    createMotionProfiler(startState, endSpeed);
  }

  /** @return max speed of the segment */
  public double getMaxSpeed() {
    return maxSpeed;
  }

  public void createMotionProfiler(MotionState startState, double endSpeed) {
    MotionProfileConstraints motionConstraints =
        new MotionProfileConstraints(maxSpeed, FollowingConstants.MAX_ACCELERATION);
    MotionProfileGoal goalState = new MotionProfileGoal(getLength(), endSpeed);
    speedController =
        MotionProfileGenerator.generateProfile(motionConstraints, goalState, startState);
    // System.out.println(speedController);
  }

  /** @return starting point of the segment */
  public Translation2d getStart() {
    return start;
  }

  /** @return end point of the segment */
  public Translation2d getEnd() {
    return end;
  }

  /** @return the total length of the segment */
  public double getLength() {
    if (isLine) {
      return deltaStart.norm();
    } else {
      return deltaStart.norm() * Translation2d.getAngle(deltaStart, deltaEnd).getRadians();
    }
  }

  /**
   * Set whether or not to extrapolate the lookahead point. Should only be true for the last segment
   * in the path
   *
   * @param shouldExtrapolate
   */
  public void extrapolateLookahead(boolean shouldExtrapolate) {
    extrapolateLookahead = shouldExtrapolate;
  }

  /**
   * Gets the point on the segment closest to the robot
   *
   * @param position the current position of the robot
   * @return the point on the segment closest to the robot
   */
  public Translation2d getClosestPoint(Translation2d position) {
    if (isLine) {
      Translation2d delta = new Translation2d(start, end);
      double closestPoint =
          ((position.x() - start.x()) * delta.x() + (position.y() - start.y()) * delta.y())
              / (delta.x() * delta.x() + delta.y() * delta.y());
      if (closestPoint >= 0 && closestPoint <= 1)
        return new Translation2d(
            start.x() + closestPoint * delta.x(), start.y() + closestPoint * delta.y());
      return (closestPoint < 0) ? start : end;
    } else {
      Translation2d deltaPosition = new Translation2d(center, position);
      deltaPosition = deltaPosition.scale(deltaStart.norm() / deltaPosition.norm());
      if (Translation2d.cross(deltaPosition, deltaStart)
              * Translation2d.cross(deltaPosition, deltaEnd)
          < 0) {
        return center.translateBy(deltaPosition);
      } else {
        Translation2d startDistance = new Translation2d(position, start);
        Translation2d endDistance = new Translation2d(position, end);
        return (endDistance.norm() < startDistance.norm()) ? end : start;
      }
    }
  }

  /**
   * Calculates the point on the segment <code>dist</code> distance from the starting point along
   * the segment.
   *
   * @param distance distance from the starting point
   * @return point on the segment <code>dist</code> distance from the starting point
   */
  public Translation2d getPointByDistance(double distance) {
    double length = getLength();
    if (!extrapolateLookahead && distance > length) {
      distance = length;
    }
    if (isLine) {
      return start.translateBy(deltaStart.scale(distance / length));
    } else {
      double deltaAngle =
          Translation2d.getAngle(deltaStart, deltaEnd).getRadians()
              * ((Translation2d.cross(deltaStart, deltaEnd) >= 0) ? 1 : -1);
      deltaAngle *= distance / length;
      Translation2d t = deltaStart.rotateBy(Rotation2d.fromRadians(deltaAngle));
      return center.translateBy(t);
    }
  }

  /**
   * Gets the remaining distance left on the segment from point <code>point</code>
   *
   * @param position result of <code>getClosestPoint()</code>
   * @return distance remaining
   */
  public double getRemainingDistance(Translation2d position) {
    if (isLine) {
      return new Translation2d(end, position).norm();
    } else {
      Translation2d deltaPosition = new Translation2d(center, position);
      double angle = Translation2d.getAngle(deltaEnd, deltaPosition).getRadians();
      double totalAngle = Translation2d.getAngle(deltaStart, deltaEnd).getRadians();
      return angle / totalAngle * getLength();
    }
  }

  private double getDistanceTravelled(Translation2d robotPosition) {
    Translation2d pathPosition = getClosestPoint(robotPosition);
    double remainingDist = getRemainingDistance(pathPosition);
    return getLength() - remainingDist;
  }

  public double getSpeedByDistance(double distance) {
    if (distance < speedController.startPos()) {
      distance = speedController.startPos();
    } else if (distance > speedController.endPos()) {
      distance = speedController.endPos();
    }
    Optional<MotionState> state = speedController.firstStateByPos(distance);
    if (state.isPresent()) {
      return state.get().velocity();
    } else {
      LOG.info("Velocity does not exist at that position!");
      return 0.0;
    }
  }

  public double getSpeedByClosestPoint(Translation2d robotPosition) {
    return getSpeedByDistance(getDistanceTravelled(robotPosition));
  }

  public MotionState getEndState() {
    return speedController.endState();
  }

  public MotionState getStartState() {
    return speedController.startState();
  }

  public String getMarker() {
    return marker;
  }

  public String toString() {
    if (isLine) {
      return "("
          + "start: "
          + start
          + ", end: "
          + end
          + ", speed: "
          + maxSpeed // + ", profile: " +
          // speedController
          + ")";
    } else {
      return "("
          + "start: "
          + start
          + ", end: "
          + end
          + ", center: "
          + center
          + ", speed: "
          + maxSpeed
          + ")"; // + ", profile: " + speedController + ")";
    }
  }
}
