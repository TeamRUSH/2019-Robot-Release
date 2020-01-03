package net.teamrush27.frc2019.util.motion;

import java.util.Optional;

/**
 * A SetpointGenerate does just-in-time motion profile generation to supply a stream of setpoints
 * that obey the given constraints to a controller. The profile is regenerated when any of the
 * inputs change, but is cached (and trimmed as we go) if the only update is to the current state.
 *
 * <p>Note that typically for smooth control, a user will feed the last iteration's setpoint as the
 * argument to getSetpoint(), and should only use a measured state directly on the first iteration
 * or if a large disturbance is detected.
 */
public class SetpointGenerator {
  /**
   * A Setpoint is just a MotionState and an additional flag indicating whether this is setpoint
   * achieves the goal (useful for higher-level logic to know that it is now time to do something
   * else).
   */
  public static class Setpoint {
    public MotionState motionState;
    public boolean finalSetpoint;

    public Setpoint(MotionState motionState, boolean finalSetpoint) {
      this.motionState = motionState;
      this.finalSetpoint = finalSetpoint;
    }

    @Override
    public String toString() {
      final StringBuffer sb = new StringBuffer("Setpoint{");
      sb.append("motionState=").append(motionState);
      sb.append(", finalSetpoint=").append(finalSetpoint);
      sb.append('}');
      return sb.toString();
    }
  }

  protected MotionProfile motionProfile = null;
  protected MotionProfileGoal motionProfileGoal = null;
  protected MotionProfileConstraints motionProfileConstraints = null;

  public SetpointGenerator() {}

  /** Force a reset of the profile. */
  public void reset() {
    motionProfile = null;
    motionProfileGoal = null;
    motionProfileConstraints = null;
  }

  /**
   * Get a new Setpoint (and generate a new MotionProfile if necessary).
   *
   * @param constraints The constraints to use.
   * @param goal The goal to use.
   * @param previousState The previous setpoint (or measured state of the system to do a reset).
   * @param time The time to generate a setpoint for.
   * @return The new Setpoint at time t.
   */
  public synchronized Setpoint getSetpoint(
      MotionProfileConstraints constraints,
      MotionProfileGoal goal,
      MotionState previousState,
      double time) {

    boolean regenerate =
        motionProfileConstraints == null
            || !motionProfileConstraints.equals(constraints)
            || motionProfileGoal == null
            || !motionProfileGoal.equals(goal)
            || motionProfile == null;

    if (!regenerate && !motionProfile.isEmpty()) {
      Optional<MotionState> expectedState = motionProfile.stateByTime(previousState.time());
      regenerate = !expectedState.isPresent() || !expectedState.get().equals(previousState);
    }
    if (regenerate) {
      // Regenerate the profile, as our current profile does not satisfy the inputs.
      motionProfileConstraints = constraints;
      motionProfileGoal = goal;
      motionProfile = MotionProfileGenerator.generateProfile(constraints, goal, previousState);
      // System.out.println(String.format("Regenerating profile with constraints %s and goal %s :
      // %s",constraints,goal,motionProfile));
    }

    // Sample the profile at time t.
    Setpoint newSetpoint = null;
    if (!motionProfile.isEmpty() && motionProfile.isValid()) {
      MotionState setpoint;
      if (time > motionProfile.endTime()) {
        setpoint = motionProfile.endState();
      } else if (time < motionProfile.startTime()) {
        setpoint = motionProfile.startState();
      } else {
        setpoint = motionProfile.stateByTime(time).get();
      }
      // Shorten the profile and return the new setpoint.
      motionProfile.trimBeforeTime(time);
      newSetpoint =
          new Setpoint(
              setpoint, motionProfile.isEmpty() || motionProfileGoal.atGoalState(setpoint));
    }

    // Invalid or empty profile - just output the same state again.
    if (newSetpoint == null) {
      newSetpoint = new Setpoint(previousState, true);
    }

    if (newSetpoint.finalSetpoint) {
      // Ensure the final setpoint matches the goal exactly.
      newSetpoint.motionState =
          new MotionState(
              newSetpoint.motionState.time(),
              motionProfileGoal.position(),
              Math.signum(newSetpoint.motionState.velocity())
                  * Math.max(
                      motionProfileGoal.maxAbsoluteVelocity(),
                      Math.abs(newSetpoint.motionState.velocity())),
              0.0);
    }

    return newSetpoint;
  }

  /**
   * Get the full profile from the latest call to getSetpoint(). Useful to check estimated time or
   * distance to goal.
   *
   * @return The profile from the latest call to getSetpoint(), or null if there is not yet a
   *     profile.
   */
  public MotionProfile getProfile() {
    return motionProfile;
  }
}
