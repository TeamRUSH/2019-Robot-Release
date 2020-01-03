package net.teamrush27.frc2019.util.motion;

import net.teamrush27.frc2019.util.math.MathUtils;

/**
 * A MotionProfileGoal defines a desired position and maximum velocity (at this position), along
 * with the behavior that should be used to determine if we are at the goal and what to do if it is
 * infeasible to reach the goal within the desired velocity bounds.
 */
public class MotionProfileGoal {
  /**
   * A goal consists of a desired position and specified maximum velocity magnitude. But what should
   * we do if we would reach the goal at a velocity greater than the maximum? This enum allows a
   * user to specify a preference on behavior in this case.
   *
   * <p>Example use-cases of each:
   *
   * <p>OVERSHOOT - Generally used with a goal maxAbsoluteVelocity of 0.0 to stop at the desired
   * position without violating any constraints.
   *
   * <p>VIOLATE_MAX_ACCELERATION - If we absolutely do not want to pass the goal and are unwilling
   * to violate the maxAbsoluteVelocity (for example, there is an obstacle in front of us - slam the
   * brakes harder than we'd like in order to avoid hitting it).
   *
   * <p>VIOLATE_MAX_ABSOLUTE_VELOCITY - If the max velocity is just a general guideline and not a
   * hard performance limit, it's better to slightly exceed it to avoid skidding wheels.
   */
  public static enum CompletionBehavior {
    // Overshoot the goal if necessary (at a velocity greater than max_abs_vel) and come back.
    // Only valid if the goal velocity is 0.0 (otherwise VIOLATE_MAX_ACCEL will be used).
    OVERSHOOT,
    // If we cannot slow down to the goal velocity before crossing the goal, allow exceeding the max
    // accel
    // constraint.
    VIOLATE_MAX_ACCELERATION,
    // If we cannot slow down to the goal velocity before crossing the goal, allow exceeding the
    // goal velocity.
    VIOLATE_MAX_ABSOLUTE_VELOCITY
  }

  protected double position;
  protected double maxAbsoluteVelocity;
  protected CompletionBehavior completionBehavior = CompletionBehavior.OVERSHOOT;
  protected double positionTolerance = 1E-3;
  protected double velocityTolerance = 1E-2;

  public MotionProfileGoal() {}

  public MotionProfileGoal(double position) {
    this.position = position;
    this.maxAbsoluteVelocity = 0.0;
    sanityCheck();
  }

  public MotionProfileGoal(double position, double maxAbsoluteVelocity) {
    this.position = position;
    this.maxAbsoluteVelocity = maxAbsoluteVelocity;
    sanityCheck();
  }

  public MotionProfileGoal(
      double position, double maxAbsoluteVelocity, CompletionBehavior completionBehavior) {
    this.position = position;
    this.maxAbsoluteVelocity = maxAbsoluteVelocity;
    this.completionBehavior = completionBehavior;
    sanityCheck();
  }

  public MotionProfileGoal(
      double position,
      double maxAbsoluteVelocity,
      CompletionBehavior completionBehavior,
      double positionTolerance,
      double velocityTolerance) {
    this.position = position;
    this.maxAbsoluteVelocity = maxAbsoluteVelocity;
    this.completionBehavior = completionBehavior;
    this.positionTolerance = positionTolerance;
    this.velocityTolerance = velocityTolerance;
    sanityCheck();
  }

  public MotionProfileGoal(MotionProfileGoal other) {
    this(
        other.position,
        other.maxAbsoluteVelocity,
        other.completionBehavior,
        other.positionTolerance,
        other.velocityTolerance);
  }

  /**
   * @return A flipped MotionProfileGoal (where the position is negated, but all other attributes
   *     remain the same).
   */
  public MotionProfileGoal flipped() {
    return new MotionProfileGoal(
        -position, maxAbsoluteVelocity, completionBehavior, positionTolerance, velocityTolerance);
  }

  public double position() {
    return position;
  }

  public double maxAbsoluteVelocity() {
    return maxAbsoluteVelocity;
  }

  public double positionTolerance() {
    return positionTolerance;
  }

  public double velocityTolerance() {
    return velocityTolerance;
  }

  public CompletionBehavior completion_behavior() {
    return completionBehavior;
  }

  public boolean atGoalState(MotionState state) {
    return atGoalPos(state.position())
        && (Math.abs(state.velocity()) < (maxAbsoluteVelocity + velocityTolerance)
            || completionBehavior == CompletionBehavior.VIOLATE_MAX_ABSOLUTE_VELOCITY);
  }

  public boolean atGoalPos(double pos) {
    return MathUtils.epsilonEquals(pos, this.position, positionTolerance);
  }

  /**
   * This method makes sure that the completion behavior is compatible with the max goal velocity.
   */
  protected void sanityCheck() {
    if (maxAbsoluteVelocity > velocityTolerance
        && completionBehavior == CompletionBehavior.OVERSHOOT) {
      completionBehavior = CompletionBehavior.VIOLATE_MAX_ACCELERATION;
    }
  }

  @Override
  public String toString() {
    return "pos: "
        + position
        + " (+/- "
        + positionTolerance
        + "), max_abs_vel: "
        + maxAbsoluteVelocity
        + " (+/- "
        + velocityTolerance
        + "), completion behavior: "
        + completionBehavior.name();
  }

  @Override
  public boolean equals(Object obj) {
    if (!(obj instanceof MotionProfileGoal)) {
      return false;
    }
    final MotionProfileGoal other = (MotionProfileGoal) obj;
    return (other.completion_behavior() == completion_behavior())
        && (other.position() == position())
        && (other.maxAbsoluteVelocity() == maxAbsoluteVelocity())
        && (other.positionTolerance() == positionTolerance())
        && (other.velocityTolerance() == velocityTolerance());
  }
}
