package net.teamrush27.frc2019.util.motion;

import net.teamrush27.frc2019.util.math.MathUtils;

/** A MotionState is a completely specified state of 1D motion through time. */
public class MotionState {
  protected final double time;
  protected final double position;
  protected final double velocity;
  protected final double acceleration;

  public static MotionState INVALID_STATE =
      new MotionState(Double.NaN, Double.NaN, Double.NaN, Double.NaN);

  public MotionState(double time, double position, double velocity, double acceleration) {
    this.time = time;
    this.position = position;
    this.velocity = velocity;
    this.acceleration = acceleration;
  }

  public MotionState(MotionState state) {
    this(state.time, state.position, state.velocity, state.acceleration);
  }

  public double time() {
    return time;
  }

  public double position() {
    return position;
  }

  public double velocity() {
    return velocity;
  }

  public double velocitySquared() {
    return velocity * velocity;
  }

  public double acceleration() {
    return acceleration;
  }

  /**
   * Extrapolates this MotionState to the specified time by applying this MotionState's
   * acceleration.
   *
   * @param time The time of the new MotionState.
   * @return A MotionState that is a valid predecessor (if t<=0) or successor (if t>=0) of this
   *     state.
   */
  public MotionState extrapolate(double time) {
    return extrapolate(time, acceleration);
  }

  /**
   * Extrapolates this MotionState to the specified time by applying a given acceleration to the (t,
   * pos, vel) portion of this MotionState.
   *
   * @param t The time of the new MotionState.
   * @param acc The acceleration to apply.
   * @return A MotionState that is a valid predecessor (if t<=0) or successor (if t>=0) of this
   *     state (with the specified accel).
   */
  public MotionState extrapolate(double t, double acc) {
    final double dt = t - this.time;
    return new MotionState(
        t, position + velocity * dt + .5 * acc * dt * dt, velocity + acc * dt, acc);
  }

  /**
   * Find the next time (first time > MotionState.t()) that this MotionState will be at pos. This is
   * an inverse of the extrapolate() method.
   *
   * @param position The position to query.
   * @return The time when we are next at pos() if we are extrapolating with a positive dt. NaN if
   *     we never reach pos.
   */
  public double nextTimeAtPos(double position) {
    if (MathUtils.epsilonEquals(position, this.position, MathUtils.DEFAULT_MAX_ERROR)) {
      // Already at pos.
      return time;
    }
    if (MathUtils.epsilonEquals(acceleration, 0.0, MathUtils.DEFAULT_MAX_ERROR)) {
      // Zero acceleration case.
      final double deltaPosition = position - this.position;
      if (!MathUtils.epsilonEquals(velocity, 0.0, MathUtils.DEFAULT_MAX_ERROR)
          && Math.signum(deltaPosition) == Math.signum(velocity)) {
        // Constant velocity heading towards pos.
        return deltaPosition / velocity + time;
      }
      return Double.NaN;
    }

    // Solve the quadratic formula.
    // ax^2 + bx + c == 0
    // x = dt
    // a = .5 * acc
    // b = vel
    // c = this.pos - pos
    final double disc = velocity * velocity - 2.0 * acceleration * (this.position - position);
    if (disc < 0.0) {
      // Extrapolating this MotionState never reaches the desired pos.
      return Double.NaN;
    }
    final double sqrtDisc = Math.sqrt(disc);
    final double maxDeltaT = (-velocity + sqrtDisc) / acceleration;
    final double minDeltaT = (-velocity - sqrtDisc) / acceleration;
    if (minDeltaT >= 0.0 && (maxDeltaT < 0.0 || minDeltaT < maxDeltaT)) {
      return time + minDeltaT;
    }
    if (maxDeltaT >= 0.0) {
      return time + maxDeltaT;
    }
    // We only reach the desired pos in the past.
    return Double.NaN;
  }

  @Override
  public String toString() {
    return "(t=" + time + ", pos=" + position + ", vel=" + velocity + ", acc=" + acceleration + ")";
  }

  /**
   * Checks if two MotionStates are epsilon-equals (all fields are equal within a nominal
   * tolerance).
   */
  @Override
  public boolean equals(Object other) {
    return (other instanceof MotionState)
        && equals((MotionState) other, MathUtils.DEFAULT_MAX_ERROR);
  }

  /**
   * Checks if two MotionStates are epsilon-equals (all fields are equal within a specified
   * tolerance).
   */
  public boolean equals(MotionState other, double epsilon) {
    return coincident(other, epsilon)
        && MathUtils.epsilonEquals(acceleration, other.acceleration, epsilon);
  }

  /**
   * Checks if two MotionStates are coincident (t, pos, and vel are equal within a nominal
   * tolerance, but acceleration may be different).
   */
  public boolean coincident(MotionState other) {
    return coincident(other, MathUtils.DEFAULT_MAX_ERROR);
  }

  /**
   * Checks if two MotionStates are coincident (t, pos, and vel are equal within a specified
   * tolerance, but acceleration may be different).
   */
  public boolean coincident(MotionState other, double epsilon) {
    return MathUtils.epsilonEquals(time, other.time, epsilon)
        && MathUtils.epsilonEquals(position, other.position, epsilon)
        && MathUtils.epsilonEquals(velocity, other.velocity, epsilon);
  }

  /**
   * Returns a MotionState that is the mirror image of this one. Pos, vel, and acc are all negated,
   * but time is not.
   */
  public MotionState flipped() {
    return new MotionState(time, -position, -velocity, -acceleration);
  }
}
