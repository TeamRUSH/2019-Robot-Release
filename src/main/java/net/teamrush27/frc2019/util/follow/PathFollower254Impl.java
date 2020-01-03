package net.teamrush27.frc2019.util.follow;

// import net.teamrush27.frc2019.subsystems.impl.Elevator;
import net.teamrush27.frc2019.util.math.Pose2d;
import net.teamrush27.frc2019.util.math.Twist2d;
import net.teamrush27.frc2019.util.motion.MotionProfileConstraints;
import net.teamrush27.frc2019.util.motion.MotionProfileGoal;
import net.teamrush27.frc2019.util.motion.MotionProfileGoal.CompletionBehavior;
import net.teamrush27.frc2019.util.motion.MotionState;
import net.teamrush27.frc2019.util.motion.ProfileFollower;
import org.apache.logging.log4j.LogManager;
import org.apache.logging.log4j.Logger;

/**
 * A PathFollower follows a predefined path using a combination of feedforward and feedback control.
 * It uses an AdaptivePurePursuitController to choose a reference pose and generate a steering
 * command (curvature), and then a ProfileFollower to generate a profile (displacement and velocity)
 * command.
 */
public class PathFollower254Impl implements PathFollower {
  private static final Logger LOG = LogManager.getLogger(PathFollower254Impl.class);
  private static final double REALLY_BIG_NUMBER = 1E6;

  private AdaptivePurePursuitController steeringController;
  private Twist2d lastSteeringDelta;
  private ProfileFollower velocityController;
  private final double inertiaGain;
  private boolean overrideFinished = false;
  private boolean doneSteering = false;
  private DebugOutput debugOutput = new DebugOutput();
  //    private final Elevator elevator = Elevator.getInstance();

  private double maxProfileVelocity;
  private double maxProfileAcceleration;
  private final double goalPositionTolerance;
  private final double goalVelocityTolerance;
  private final double stopSteeringDistance;
  private double crossTrackError = 0.0;
  private double alongTrackError = 0.0;
  /** Create a new PathFollower for a given path. */
  public PathFollower254Impl(Path path, boolean reversed, Parameters parameters) {
    steeringController = new AdaptivePurePursuitController(path, reversed, parameters.lookahead);
    lastSteeringDelta = Twist2d.identity();
    velocityController =
        new ProfileFollower(
            parameters.profileP,
            parameters.profileI,
            parameters.profileV,
            parameters.profileFfV,
            parameters.profileFfA);
    velocityController.setConstraints(
        new MotionProfileConstraints(
            parameters.profileMaxAbsVelocity, parameters.profileMaxAbsAcceleration));
    maxProfileVelocity = parameters.profileMaxAbsVelocity;
    maxProfileAcceleration = parameters.profileMaxAbsAcceleration;
    goalPositionTolerance = parameters.goalPositionTolerance;
    goalVelocityTolerance = parameters.goalVelocityTolerance;
    inertiaGain = parameters.inertiaGain;
    stopSteeringDistance = parameters.stopSteeringDistance;
  }

  /**
   * Get new velocity commands to follow the path.
   *
   * @param time The current timestamp
   * @param pose The current robot pose
   * @param displacement The current robot displacement (total distance driven).
   * @param velocity The current robot velocity.
   * @return The velocity command to apply
   */
  @Override
  public synchronized Twist2d update(
      double time, Pose2d pose, double displacement, double velocity) {
    if (!steeringController.isFinished()) { // if we're still in a turn
      final AdaptivePurePursuitController.Command steeringCommand = steeringController.update(pose);
      debugOutput.lookaheadPointX = steeringCommand.lookaheadPoint.x();
      debugOutput.lookaheadPointY = steeringCommand.lookaheadPoint.y();
      debugOutput.lookaheadPointVelocity = steeringCommand.endVelocity;
      debugOutput.steeringCommandDeltaX = steeringCommand.delta.deltaX;
      debugOutput.steeringCommandDeltaY = steeringCommand.delta.deltaY;
      debugOutput.steeringCommandDeltaTheta = steeringCommand.delta.deltaTheta;
      crossTrackError = steeringCommand.crossTrackError;
      lastSteeringDelta = steeringCommand.delta;

      double maxAcceleration = 100; // elevator.isHigh() ? 100 : maxProfileAcceleration;

      velocityController.setGoalAndConstraints(
          new MotionProfileGoal(
              displacement + steeringCommand.delta.deltaX,
              Math.abs(steeringCommand.endVelocity),
              CompletionBehavior.VIOLATE_MAX_ACCELERATION,
              goalPositionTolerance,
              goalVelocityTolerance),
          new MotionProfileConstraints(
              Math.min(maxProfileVelocity, steeringCommand.maxVelocity), maxAcceleration));

      if (steeringCommand.remainingPathLength < stopSteeringDistance) {
        doneSteering = true;
      }
    }

    final MotionState latestState = new MotionState(time, displacement, velocity, 0.0);
    final double velocityCommand = velocityController.update(latestState, time);
    alongTrackError = velocityController.getPosError();
    final double curvature = lastSteeringDelta.deltaTheta / lastSteeringDelta.deltaX;
    double deltaTheta = lastSteeringDelta.deltaTheta;
    if (!Double.isNaN(curvature) && Math.abs(curvature) < REALLY_BIG_NUMBER) {
      // Regenerate angular velocity command from adjusted curvature.
      final double absoluteVelocitySetpoint = Math.abs(velocityController.getSetpoint().velocity());
      deltaTheta =
          lastSteeringDelta.deltaX * curvature * (1.0 + inertiaGain * absoluteVelocitySetpoint);
    }
    final double scale = velocityCommand / lastSteeringDelta.deltaX;
    final Twist2d twistToApply =
        new Twist2d(lastSteeringDelta.deltaX * scale, 0.0, deltaTheta * scale);

    // Fill out debug.
    debugOutput.t = time;
    debugOutput.poseX = pose.getTranslation().x();
    debugOutput.poseY = pose.getTranslation().y();
    debugOutput.poseTheta = pose.getRotation().getRadians();
    debugOutput.linearDisplacement = displacement;
    debugOutput.linearVelocity = velocity;
    debugOutput.profileDisplacement = velocityController.getSetpoint().position();
    debugOutput.profileVelocity = velocityController.getSetpoint().velocity();
    debugOutput.profileAcceleration = velocityController.getSetpoint().acceleration();
    debugOutput.velocityCommandDeltaX = twistToApply.deltaX;
    debugOutput.velocityCommandDeltaY = twistToApply.deltaY;
    debugOutput.velocityCommandDeltaTheta = twistToApply.deltaTheta;
    debugOutput.velocityCommandError = velocityController.getVelError();
    debugOutput.crossTrackError = crossTrackError;
    debugOutput.alongTrackError = alongTrackError;

    LOG.info(debugOutput.toCSV());

    return twistToApply;
  }

  @Override
  public double getCrossTrackError() {
    return crossTrackError;
  }

  @Override
  public double getAlongTrackError() {
    return alongTrackError;
  }

  @Override
  public DebugOutput getDebug() {
    return debugOutput;
  }

  @Override
  public boolean isFinished() {
    return (steeringController.isFinished()
            && velocityController.isFinishedProfile()
            && velocityController.onTarget())
        || overrideFinished;
  }

  @Override
  public void forceFinish() {
    overrideFinished = true;
  }

  @Override
  public boolean hasPassedMarker(String marker) {
    return steeringController.hasPassedMarker(marker);
  }
}
