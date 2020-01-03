package net.teamrush27.frc2019.util.motion;

import net.teamrush27.frc2019.util.math.MathUtils;

/**
 * A MotionSegment is a movement from a start MotionState to an end MotionState with a constant
 * acceleration.
 */
public class MotionSegment {
  protected MotionState start;
  protected MotionState end;

  public MotionSegment(MotionState start, MotionState end) {
    this.start = start;
    this.end = end;
  }

  /**
   * Verifies that:
   *
   * <p>1. All segments have a constant acceleration.
   *
   * <p>2. All segments have monotonic position (sign of velocity doesn't change).
   *
   * <p>3. The time, position, velocity, and acceleration of the profile are consistent.
   */
  public boolean isValid() {
    if (!MathUtils.epsilonEquals(
        start().acceleration(), end().acceleration(), MathUtils.DEFAULT_MAX_ERROR)) {
      // Acceleration is not constant within the segment.
      System.err.println(
          "Segment acceleration not constant! Start acc: "
              + start().acceleration()
              + ", End acc: "
              + end().acceleration());
      return false;
    }
    if (Math.signum(start().velocity()) * Math.signum(end().velocity()) < 0.0
        && !MathUtils.epsilonEquals(start().velocity(), 0.0, MathUtils.DEFAULT_MAX_ERROR)
        && !MathUtils.epsilonEquals(end().velocity(), 0.0, MathUtils.DEFAULT_MAX_ERROR)) {
      // Velocity direction reverses within the segment.
      System.err.println(
          "Segment velocity reverses! Start vel: "
              + start().velocity()
              + ", End vel: "
              + end().velocity());
      return false;
    }
    if (!start().extrapolate(end().time()).equals(end())) {
      // A single segment is not consistent.
      if (start().time() == end().time() && Double.isInfinite(start().acceleration())) {
        // One allowed exception: If acc is infinite and dt is zero.
        return true;
      }
      System.err.println("Segment not consistent! Start: " + start() + ", End: " + end());
      return false;
    }
    return true;
  }

  public boolean containsTime(double time) {
    return time >= start().time() && time <= end().time();
  }

  public boolean containsPosition(double position) {
    return position >= start().position() && position <= end().position()
        || position <= start().position() && position >= end().position();
  }

  public MotionState start() {
    return start;
  }

  public void setStart(MotionState start) {
    this.start = start;
  }

  public MotionState end() {
    return end;
  }

  public void setEnd(MotionState end) {
    this.end = end;
  }

  @Override
  public String toString() {
    return "Start: " + start() + ", End: " + end();
  }
}
