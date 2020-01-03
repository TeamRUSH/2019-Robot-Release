package net.teamrush27.frc2019.util.motion;

import net.teamrush27.frc2019.util.motion.MotionProfileGoal.CompletionBehavior;

/**
 * A controller for tracking a profile generated to attain a MotionProfileGoal. The controller uses
 * feedforward on acceleration, velocity, and position; proportional feedback on velocity and
 * position; and integral feedback on position.
 */
public class ProfileFollower {
  protected double p;
  protected double i;
  protected double v;
  protected double ffV;
  protected double ffA;

  protected double minOutput = Double.NEGATIVE_INFINITY;
  protected double maxOutput = Double.POSITIVE_INFINITY;
  protected MotionState latestActualState;
  protected MotionState initialState;
  protected double latestPosError;
  protected double latestVelError;
  protected double totalError;

  protected MotionProfileGoal goal = null;
  protected MotionProfileConstraints constraints = null;
  protected SetpointGenerator setpointGenerator = new SetpointGenerator();
  protected SetpointGenerator.Setpoint latestSetpoint = null;

  /**
   * Create a new ProfileFollower.
   *
   * @param p The proportional gain on position error.
   * @param i The integral gain on position error.
   * @param v The proportional gain on velocity error (or derivative gain on position error).
   * @param ffV The feedforward gain on velocity. Should be 1.0 if the units of the profile match
   *     the units of the output.
   * @param ffA The feedforward gain on acceleration.
   */
  public ProfileFollower(double p, double i, double v, double ffV, double ffA) {
    resetProfile();
    setGains(p, i, v, ffV, ffA);
  }

  public void setGains(double p, double i, double v, double ffV, double ffA) {
    this.p = p;
    this.i = i;
    this.v = v;
    this.ffV = ffV;
    this.ffA = ffA;
  }

  /**
   * Completely clear all state related to the current profile (min and max outputs are maintained).
   */
  public void resetProfile() {
    totalError = 0.0;
    initialState = MotionState.INVALID_STATE;
    latestActualState = MotionState.INVALID_STATE;
    latestPosError = Double.NaN;
    latestVelError = Double.NaN;
    setpointGenerator.reset();
    goal = null;
    constraints = null;
    resetSetpoint();
  }

  /** Specify a goal and constraints for achieving the goal. */
  public void setGoalAndConstraints(MotionProfileGoal goal, MotionProfileConstraints constraints) {
    if (goal != null && !goal.equals(goal) && latestSetpoint != null) {
      // Clear the final state bit since the goal has changed.
      latestSetpoint.finalSetpoint = false;
    }
    this.goal = goal;
    this.constraints = constraints;
  }

  public void setGoal(MotionProfileGoal goal) {
    setGoalAndConstraints(goal, constraints);
  }

  /** @return The current goal (null if no goal has been set since the latest call to reset()). */
  public MotionProfileGoal getGoal() {
    return goal;
  }

  public void setConstraints(MotionProfileConstraints constraints) {
    setGoalAndConstraints(goal, constraints);
  }

  public MotionState getSetpoint() {
    return (latestSetpoint == null ? MotionState.INVALID_STATE : latestSetpoint.motionState);
  }

  /**
   * Reset just the setpoint. This means that the latest_state provided to update() will be used
   * rather than feeding forward the previous setpoint the next time update() is called. This almost
   * always forces a MotionProfile update, and may be warranted if tracking error gets very large.
   */
  public void resetSetpoint() {
    latestSetpoint = null;
  }

  public void resetIntegral() {
    totalError = 0.0;
  }

  /**
   * Update the setpoint and apply the control gains to generate a control output.
   *
   * @param latestState The latest *actual* state, used only for feedback purposes (unless this is
   *     the first iteration or reset()/resetSetpoint() was just called, in which case this is the
   *     new start state for the profile).
   * @param time The timestamp for which the setpoint is desired.
   * @return An output that reflects the control output to apply to achieve the new setpoint.
   */
  public synchronized double update(MotionState latestState, double time) {

    latestActualState = latestState;
    MotionState previousState = latestState;
    if (latestSetpoint != null) {
      previousState = latestSetpoint.motionState;
    } else {
      initialState = previousState;
    }
    final double deltaTime = Math.max(0.0, time - previousState.time());
    latestSetpoint = setpointGenerator.getSetpoint(constraints, goal, previousState, time);

    // Update error.
    latestPosError = latestSetpoint.motionState.position() - latestState.position();
    latestVelError = latestSetpoint.motionState.velocity() - latestState.velocity();

    // Calculate the feedforward and proportional terms.
    double output =
        p * latestPosError
            + v * latestVelError
            + ffV * latestSetpoint.motionState.velocity()
            + (Double.isNaN(latestSetpoint.motionState.acceleration())
                ? 0.0
                : ffA * latestSetpoint.motionState.acceleration());
    if (output >= minOutput && output <= maxOutput) {
      // Update integral.
      totalError += latestPosError * deltaTime;
      output += i * totalError;
    } else {
      // Reset integral windup.
      totalError = 0.0;
    }
    // Clamp to limits.
    output = Math.max(minOutput, Math.min(maxOutput, output));

    return output;
  }

  public void setMinOutput(double minOutput) {
    this.minOutput = minOutput;
  }

  public void setMaxOutput(double maxOutput) {
    this.maxOutput = maxOutput;
  }

  public double getPosError() {
    return latestPosError;
  }

  public double getVelError() {
    return latestVelError;
  }

  /**
   * We are finished the profile when the final setpoint has been generated. Note that this does not
   * check whether we are anywhere close to the final setpoint, however.
   *
   * @return True if the final setpoint has been generated for the current goal.
   */
  public boolean isFinishedProfile() {
    return goal != null && latestSetpoint != null && latestSetpoint.finalSetpoint;
  }

  /**
   * We are on target if our actual state achieves the goal (where the definition of achievement
   * depends on the goal's completion behavior).
   *
   * @return True if we have actually achieved the current goal.
   */
  public boolean onTarget() {
    if (goal == null || latestSetpoint == null) {
      return false;
    }
    // For the options that don't achieve the goal velocity exactly, also count any instance where
    // we have passed
    // the finish line.
    final double goalToStart = goal.position() - initialState.position();
    final double goalToActual = goal.position() - latestActualState.position();
    final boolean passedGoalState = Math.signum(goalToStart) * Math.signum(goalToActual) < 0.0;
    return goal.atGoalState(latestActualState)
        || (goal.completion_behavior() != CompletionBehavior.OVERSHOOT && passedGoalState);
  }
}
