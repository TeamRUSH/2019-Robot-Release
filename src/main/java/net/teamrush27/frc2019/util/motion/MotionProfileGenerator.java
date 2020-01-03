package net.teamrush27.frc2019.util.motion;

import net.teamrush27.frc2019.util.motion.MotionProfileGoal.CompletionBehavior;

/**
 * A MotionProfileGenerator generates minimum-time MotionProfiles to travel from a given MotionState
 * to a given MotionProfileGoal while obeying a set of MotionProfileConstraints.
 */
public class MotionProfileGenerator {
  // Static class.
  private MotionProfileGenerator() {}

  protected static MotionProfile generateFlippedProfile(
      MotionProfileConstraints constraints,
      MotionProfileGoal goalState,
      MotionState previousState) {
    MotionProfile profile =
        generateProfile(constraints, goalState.flipped(), previousState.flipped());
    for (MotionSegment segment : profile.segments()) {
      segment.setStart(segment.start().flipped());
      segment.setEnd(segment.end().flipped());
    }
    return profile;
  }

  /**
   * Generate a motion profile.
   *
   * @param constraints The constraints to use.
   * @param goalState The goal to use.
   * @param previousState The initial state to use.
   * @return A motion profile from prev_state to goal_state that satisfies constraints.
   */
  public static synchronized MotionProfile generateProfile(
      MotionProfileConstraints constraints,
      MotionProfileGoal goalState,
      MotionState previousState) {
    //    	System.out.println("generateProfile");
    double deltaPos = goalState.position() - previousState.position();
    if (deltaPos < 0.0 || (deltaPos == 0.0 && previousState.velocity() < 0.0)) {
      // For simplicity, we always assume the goal requires positive movement. If negative, we flip
      // to solve, then
      // flip the solution.
      return generateFlippedProfile(constraints, goalState, previousState);
    }
    // Invariant from this point on: deltaPos >= 0.0
    // Clamp the start state to be valid.
    MotionState startState =
        new MotionState(
            previousState.time(),
            previousState.position(),
            Math.signum(previousState.velocity())
                * Math.min(Math.abs(previousState.velocity()), constraints.maxAbsoluteVelocity()),
            Math.signum(previousState.acceleration())
                * Math.min(
                    Math.abs(previousState.acceleration()), constraints.maxAbsoluteAcceleration()));
    MotionProfile profile = new MotionProfile();
    profile.reset(startState);
    // If our velocity is headed away from the goal, the first thing we need to do is to stop.
    if (startState.velocity() < 0.0 && deltaPos > 0.0) {
      final double stopping_time =
          Math.abs(startState.velocity() / constraints.maxAbsoluteAcceleration());
      profile.appendControl(constraints.maxAbsoluteAcceleration(), stopping_time);
      startState = profile.endState();
      deltaPos = goalState.position() - startState.position();
    }
    // Invariant from this point on: start_state.vel() >= 0.0
    final double minAbsoluteVelocityAtGoalSquared =
        startState.velocitySquared() - 2.0 * constraints.maxAbsoluteAcceleration() * deltaPos;
    final double minAbsoluteVelocityAtGoal = Math.sqrt(Math.abs(minAbsoluteVelocityAtGoalSquared));
    final double maxAbsoluteVelocityAtGoal =
        Math.sqrt(
            startState.velocitySquared() + 2.0 * constraints.maxAbsoluteAcceleration() * deltaPos);
    double goalVelocity = goalState.maxAbsoluteVelocity();
    double maxAcceleration = constraints.maxAbsoluteAcceleration();
    if (minAbsoluteVelocityAtGoalSquared > 0.0
        && minAbsoluteVelocityAtGoal
            > (goalState.maxAbsoluteVelocity() + goalState.velocityTolerance())) {
      // Overshoot is unavoidable with the current constraints. Look at completion_behavior to see
      // what we should
      // do.
      if (goalState.completion_behavior() == CompletionBehavior.VIOLATE_MAX_ABSOLUTE_VELOCITY) {
        // Adjust the goal velocity.
        goalVelocity = minAbsoluteVelocityAtGoal;
      } else if (goalState.completion_behavior() == CompletionBehavior.VIOLATE_MAX_ACCELERATION) {
        if (Math.abs(deltaPos) < goalState.positionTolerance()) {
          // Special case: We are at the goal but moving too fast. This requires 'infinite'
          // acceleration,
          // which will result in NaNs below, so we can return the profile immediately.
          profile.appendSegment(
              new MotionSegment(
                  new MotionState(
                      profile.endTime(),
                      profile.endPos(),
                      profile.endState().velocity(),
                      Double.NEGATIVE_INFINITY),
                  new MotionState(
                      profile.endTime(),
                      profile.endPos(),
                      goalVelocity,
                      Double.NEGATIVE_INFINITY)));
          profile.consolidate();
          return profile;
        }
        // Adjust the max acceleration.
        maxAcceleration =
            Math.abs(goalVelocity * goalVelocity - startState.velocitySquared()) / (2.0 * deltaPos);
      } else {
        // We are going to overshoot the goal, so the first thing we need to do is come to a stop.
        final double stoppingTime =
            Math.abs(startState.velocity() / constraints.maxAbsoluteAcceleration());
        profile.appendControl(-constraints.maxAbsoluteAcceleration(), stoppingTime);
        // Now we need to travel backwards, so generate a flipped profile.
        profile.appendProfile(generateFlippedProfile(constraints, goalState, profile.endState()));
        profile.consolidate();
        return profile;
      }
    }
    goalVelocity = Math.min(goalVelocity, maxAbsoluteVelocityAtGoal);
    // Invariant from this point forward: We can achieve goal_vel at goal_state.pos exactly using no
    // more than +/-
    // max_acc.

    // What is the maximum velocity we can reach (Vmax)? This is the intersection of two curves: one
    // accelerating
    // towards the goal from profile.finalState(), the other coming from the goal at max vel (in
    // reverse). If Vmax
    // is greater than constraints.max_abs_vel, we will clamp and cruise.
    // Solve the following three equations to find Vmax (by substitution):
    // Vmax^2 = Vstart^2 + 2*a*d_accel
    // Vgoal^2 = Vmax^2 - 2*a*d_decel
    // deltaPos = d_accel + d_decel
    final double maxVelocity =
        Math.min(
            constraints.maxAbsoluteVelocity(),
            Math.sqrt(
                (startState.velocitySquared() + goalVelocity * goalVelocity) / 2.0
                    + deltaPos * maxAcceleration));
    //        System.out.println(String.format("maxVelocity: %s", startState.velocitySquared() +
    // goalVelocity * goalVelocity));

    // Accelerate to v_max
    if (maxVelocity > startState.velocity()) {
      //        	System.out.println("accel to v_max");
      final double accelerationTime = (maxVelocity - startState.velocity()) / maxAcceleration;
      profile.appendControl(maxAcceleration, accelerationTime);
      startState = profile.endState();
    }
    // Figure out how much distance will be covered during deceleration.
    final double distanceToDecelerate =
        Math.max(
            0.0,
            (startState.velocitySquared() - goalVelocity * goalVelocity)
                / (2.0 * constraints.maxAbsoluteAcceleration()));
    final double distanceCruise =
        Math.max(0.0, goalState.position() - startState.position() - distanceToDecelerate);
    // Cruise at constant velocity.
    if (distanceCruise > 0.0) {
      final double cruiseTime = distanceCruise / startState.velocity();
      //            System.out.println(String.format("distanceCruise > 0: %s - %s", cruiseTime,
      // startState));
      profile.appendControl(0.0, cruiseTime);
      startState = profile.endState();
    }
    // Decelerate to goal velocity.
    if (distanceToDecelerate > 0.0) {
      final double decelerationTime = (startState.velocity() - goalVelocity) / maxAcceleration;
      profile.appendControl(-maxAcceleration, decelerationTime);
    }

    profile.consolidate();
    return profile;
  }
}
