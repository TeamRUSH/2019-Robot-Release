package net.teamrush27.frc2019.util.follow;

import net.teamrush27.frc2019.util.math.Pose2d;
import net.teamrush27.frc2019.util.math.Twist2d;

public interface PathFollower {

  Twist2d update(double time, Pose2d pose, double displacement, double velocity);

  double getCrossTrackError();

  double getAlongTrackError();

  DebugOutput getDebug();

  boolean isFinished();

  void forceFinish();

  boolean hasPassedMarker(String marker);

  public static class DebugOutput {
    public double t;
    public double poseX;
    public double poseY;
    public double poseTheta;
    public double linearDisplacement;
    public double linearVelocity;
    public double profileDisplacement;
    public double profileVelocity;
    public double profileAcceleration;
    public double velocityCommandDeltaX;
    public double velocityCommandDeltaY;
    public double velocityCommandDeltaTheta;
    public double velocityCommandError;
    public double steeringCommandDeltaX;
    public double steeringCommandDeltaY;
    public double steeringCommandDeltaTheta;
    public double crossTrackError;
    public double alongTrackError;
    public double lookaheadPointX;
    public double lookaheadPointY;
    public double lookaheadPointVelocity;

    public String toCSV() {
      return String.format(
          "%s,%s,%s,%s,%s,%s,%s,%s,%s,%s,%s,%s,%s,%s,%s,%s,%s,%s,%s,%s,%s",
          t,
          poseX,
          poseY,
          poseTheta,
          linearDisplacement,
          linearVelocity,
          profileDisplacement,
          profileVelocity,
          profileAcceleration,
          velocityCommandDeltaX,
          velocityCommandDeltaY,
          velocityCommandDeltaTheta,
          velocityCommandError,
          steeringCommandDeltaX,
          steeringCommandDeltaY,
          steeringCommandDeltaTheta,
          crossTrackError,
          alongTrackError,
          lookaheadPointX,
          lookaheadPointY,
          lookaheadPointVelocity);
    }

    @Override
    public String toString() {
      return "DebugOutput{"
          + "t="
          + t
          + ", poseX="
          + poseX
          + ", poseY="
          + poseY
          + ", poseTheta="
          + poseTheta
          + ", linearDisplacement="
          + linearDisplacement
          + ", linearVelocity="
          + linearVelocity
          + ", profileDisplacement="
          + profileDisplacement
          + ", profileVelocity="
          + profileVelocity
          + ", profileAcceleration="
          + profileAcceleration
          + ", velocityCommandDeltaX="
          + velocityCommandDeltaX
          + ", velocityCommandDeltaY="
          + velocityCommandDeltaY
          + ", velocityCommandDeltaTheta="
          + velocityCommandDeltaTheta
          + ", velocityCommandError="
          + velocityCommandError
          + ", steeringCommandDeltaX="
          + steeringCommandDeltaX
          + ", steeringCommandDeltaY="
          + steeringCommandDeltaY
          + ", steeringCommandDeltaTheta="
          + steeringCommandDeltaTheta
          + ", crossTrackError="
          + crossTrackError
          + ", alongTrackError="
          + alongTrackError
          + ", lookaheadPointX="
          + lookaheadPointX
          + ", lookaheadPointY="
          + lookaheadPointY
          + ", lookaheadPointVelocity="
          + lookaheadPointVelocity
          + '}';
    }
  }

  public static class Parameters {
    public final Lookahead lookahead;
    public final double inertiaGain;
    public final double profileP;
    public final double profileI;
    public final double profileV;
    public final double profileFfV;
    public final double profileFfA;
    public final double profileMaxAbsVelocity;
    public final double profileMaxAbsAcceleration;
    public final double goalPositionTolerance;
    public final double goalVelocityTolerance;
    public final double stopSteeringDistance;

    public Parameters(
        Lookahead lookahead,
        double inertiaGain,
        double profileP,
        double profileI,
        double profileV,
        double profileFfV,
        double profileFfA,
        double profileMaxAbsVelocity,
        double profileMaxAbsAcceleration,
        double goalPositionTolerance,
        double goalVelocityTolerance,
        double stopSteeringDistance) {
      this.lookahead = lookahead;
      this.inertiaGain = inertiaGain;
      this.profileP = profileP;
      this.profileI = profileI;
      this.profileV = profileV;
      this.profileFfV = profileFfV;
      this.profileFfA = profileFfA;
      this.profileMaxAbsVelocity = profileMaxAbsVelocity;
      this.profileMaxAbsAcceleration = profileMaxAbsAcceleration;
      this.goalPositionTolerance = goalPositionTolerance;
      this.goalVelocityTolerance = goalVelocityTolerance;
      this.stopSteeringDistance = stopSteeringDistance;
    }
  }
}
