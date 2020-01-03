package net.teamrush27.frc2019.util.follow;

import java.util.ArrayList;
import java.util.HashSet;
import java.util.List;
import net.teamrush27.frc2019.constants.FollowingConstants;
import net.teamrush27.frc2019.util.math.Translation2d;
import net.teamrush27.frc2019.util.motion.MotionState;

/**
 * Class representing the robot's autonomous path.
 *
 * <p>Field Coordinate System: Uses a right hand coordinate system. Positive x is right, positive y
 * is up, and the origin is at the bottom left corner of the field. For angles, 0 degrees is facing
 * right (1, 0) and angles increase as you turn counter clockwise.
 */
// TODO tear out List<PathSegment> in favor of Trajectory<TimedState<Pose2dWithCurvature>>
public class Path {
  List<PathSegment> segments;
  PathSegment prevSegment;
  HashSet<String> markersCrossed = new HashSet<String>();

  public void extrapolateLast() {
    PathSegment lastPosition = segments.get(segments.size() - 1);
    lastPosition.extrapolateLookahead(true);
  }

  public Translation2d getEndPosition() {
    return segments.get(segments.size() - 1).getEnd();
  }

  public Path() {
    segments = new ArrayList<PathSegment>();
  }

  /**
   * add a segment to the Path
   *
   * @param segment the segment to add
   */
  public void addSegment(PathSegment segment) {
    segments.add(segment);
  }

  /** @return the last MotionState in the path */
  public MotionState getLastMotionState() {
    if (segments.size() > 0) {
      MotionState endState = segments.get(segments.size() - 1).getEndState();
      return new MotionState(0.0, 0.0, endState.velocity(), endState.acceleration());
    } else {
      return new MotionState(0, 0, 0, 0);
    }
  }

  /**
   * get the remaining distance left for the robot to travel on the current segment
   *
   * @param robotPosition robot position
   * @return remaining distance on current segment
   */
  public double getSegmentRemainingDist(Translation2d robotPosition) {
    PathSegment currentSegment = segments.get(0);
    return currentSegment.getRemainingDistance(currentSegment.getClosestPoint(robotPosition));
  }

  /** @return the length of the current segment */
  public double getSegmentLength() {
    PathSegment currentSegment = segments.get(0);
    return currentSegment.getLength();
  }

  public static class TargetPointReport {
    public Translation2d closestPoint;
    public double closestPointDistance;
    public double closestPointSpeed;
    public Translation2d lookaheadPoint;
    public double maxSpeed;
    public double lookaheadPointSpeed;
    public double remainingSegmentDistance;
    public double remainingPathDistance;

    public TargetPointReport() {}

    @Override
    public String toString() {
      return "TargetPointReport{"
          + "closestPoint="
          + closestPoint
          + ", closestPointDistance="
          + closestPointDistance
          + ", closestPointSpeed="
          + closestPointSpeed
          + ", lookaheadPoint="
          + lookaheadPoint
          + ", maxSpeed="
          + maxSpeed
          + ", lookaheadPointSpeed="
          + lookaheadPointSpeed
          + ", remainingSegmentDistance="
          + remainingSegmentDistance
          + ", remainingPathDistance="
          + remainingPathDistance
          + '}';
    }
  }

  /**
   * Gives the position of the lookahead point (and removes any segments prior to this point).
   *
   * @param robotPos Translation of the current robot pose.
   * @return report containing everything we might want to know about the target point.
   */
  public TargetPointReport getTargetPoint(Translation2d robotPos, Lookahead lookahead) {
    TargetPointReport currentReport = new TargetPointReport();
    PathSegment currentSegment = segments.get(0);
    currentReport.closestPoint = currentSegment.getClosestPoint(robotPos);
    currentReport.closestPointDistance =
        new Translation2d(robotPos, currentReport.closestPoint).norm();
    currentReport.remainingSegmentDistance =
        currentSegment.getRemainingDistance(currentReport.closestPoint);
    currentReport.remainingPathDistance = currentReport.remainingSegmentDistance;
    for (int i = 1; i < segments.size(); ++i) {
      currentReport.remainingPathDistance += segments.get(i).getLength();
    }
    currentReport.closestPointSpeed =
        currentSegment.getSpeedByDistance(
            currentSegment.getLength() - currentReport.remainingSegmentDistance);
    double lookaheadDistance =
        lookahead.getLookaheadForSpeed(currentReport.closestPointSpeed)
            + currentReport.closestPointDistance;
    if (currentReport.remainingSegmentDistance < lookaheadDistance && segments.size() > 1) {
      lookaheadDistance -= currentReport.remainingSegmentDistance;
      for (int i = 1; i < segments.size(); ++i) {
        currentSegment = segments.get(i);
        final double length = currentSegment.getLength();
        if (length < lookaheadDistance && i < segments.size() - 1) {
          lookaheadDistance -= length;
        } else {
          break;
        }
      }
    } else {
      lookaheadDistance += (currentSegment.getLength() - currentReport.remainingSegmentDistance);
    }
    currentReport.maxSpeed = currentSegment.getMaxSpeed();
    currentReport.lookaheadPoint = currentSegment.getPointByDistance(lookaheadDistance);
    currentReport.lookaheadPointSpeed = currentSegment.getSpeedByDistance(lookaheadDistance);
    checkSegmentDone(currentReport.closestPoint);
    return currentReport;
  }

  /**
   * Gives the speed the robot should be traveling at the given position
   *
   * @param robotPos position of the robot
   * @return speed robot should be traveling
   */
  public double getSpeed(Translation2d robotPos) {
    PathSegment currentSegment = segments.get(0);
    return currentSegment.getSpeedByClosestPoint(robotPos);
  }

  /**
   * Checks if the robot has finished traveling along the current segment then removes it from the
   * path if it has
   *
   * @param robotPos robot position
   */
  public void checkSegmentDone(Translation2d robotPos) {
    PathSegment currentSegment = segments.get(0);
    double remainingDist =
        currentSegment.getRemainingDistance(currentSegment.getClosestPoint(robotPos));
    if (remainingDist < FollowingConstants.SEGMENT_COMPLETION_TOLERANCE) {
      removeCurrentSegment();
    }
  }

  public void removeCurrentSegment() {
    prevSegment = segments.remove(0);
    String marker = prevSegment.getMarker();
    if (marker != null) markersCrossed.add(marker);
  }

  /** Ensures that all speeds in the path are attainable and robot can slow down in time */
  public void verifySpeeds() {
    double maxStartSpeed = 0.0;
    double[] startSpeeds = new double[segments.size() + 1];
    startSpeeds[segments.size()] = 0.0;
    for (int i = segments.size() - 1; i >= 0; i--) {
      PathSegment segment = segments.get(i);
      maxStartSpeed +=
          Math.sqrt(
              maxStartSpeed * maxStartSpeed
                  + 2 * FollowingConstants.MAX_ACCELERATION * segment.getLength());
      startSpeeds[i] = segment.getStartState().velocity();
      // System.out.println(maxStartSpeed + ", " + startSpeeds[i]);
      if (startSpeeds[i] > maxStartSpeed) {
        startSpeeds[i] = maxStartSpeed;
        // System.out.println("Segment starting speed is too high!");
      }
      maxStartSpeed = startSpeeds[i];
    }
    for (int i = 0; i < segments.size(); i++) {
      PathSegment segment = segments.get(i);
      double endSpeed = startSpeeds[i + 1];
      MotionState startState =
          (i > 0) ? segments.get(i - 1).getEndState() : new MotionState(0, 0, 0, 0);
      startState = new MotionState(0, 0, startState.velocity(), startState.velocity());
      segment.createMotionProfiler(startState, endSpeed);
    }
  }

  public boolean hasPassedMarker(String marker) {
    return markersCrossed.contains(marker);
  }

  public String toString() {
    String str = "";
    for (PathSegment s : segments) {
      str += s.toString() + "\n";
    }
    return str;
  }
}
