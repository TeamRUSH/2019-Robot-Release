package net.teamrush27.frc2019.util.math;

import net.teamrush27.frc2019.Robot;
import net.teamrush27.frc2019.constants.ChezyConstants;
import net.teamrush27.frc2019.constants.RobotConstants;

/**
 * Provides forward and inverse kinematics equations for the robot modeling the wheelbase as a
 * differential drive (with a corrective factor to account for skidding).
 */
public class KinematicsUtils {

  private static final double MAX_ERROR = 1E-9;

  /**
   * Forward kinematics using only encoders, rotation is implicit (less accurate than below, but
   * useful for predicting motion)
   */
  public static Twist2d forwardKinematics(double leftWheelDelta, double rightWheelDelta) {
    double deltaV =
        (rightWheelDelta - leftWheelDelta) / 2 * Robot.ROBOT_CONFIGURATION.getScrubFactor();
    double deltaRotation = deltaV * 2 / RobotConstants.TRACK_WIDTH;
    return forwardKinematics(leftWheelDelta, rightWheelDelta, deltaRotation);
  }

  /** Forward kinematics using encoders and explicitly measured rotation (ex. from gyro) */
  public static Twist2d forwardKinematics(
      double leftWheelDelta, double rightWheelDelta, double deltaRotationsInRadians) {
    final double deltaX = (leftWheelDelta + rightWheelDelta) / 2.0;
    return new Twist2d(deltaX, 0, deltaRotationsInRadians);
  }

  /** For convenience, forward kinematic with an absolute rotation and previous rotation. */
  public static Twist2d forwardKinematics(
      Rotation2d previousHeading,
      double leftWheelDelta,
      double rightWheelDelta,
      Rotation2d currentHeading) {
    return forwardKinematics(
        leftWheelDelta,
        rightWheelDelta,
        previousHeading.inverse().rotateBy(currentHeading).getRadians());
  }

  /** Append the result of forward kinematics to a previous pose. */
  public static Pose2d integrateForwardKinematics(
      Pose2d currentPose,
      double leftWheelDelta,
      double rightWheelDelta,
      Rotation2d currentHeading) {
    Twist2d withGyro =
        forwardKinematics(
            currentPose.getRotation(), leftWheelDelta, rightWheelDelta, currentHeading);
    return integrateForwardKinematics(currentPose, withGyro);
  }

  /** For convenience, integrate forward kinematics with a Twist2d and previous rotation. */
  public static Pose2d integrateForwardKinematics(Pose2d currentPose, Twist2d forwardKinematics) {
    return currentPose.transformBy(Pose2d.exp(forwardKinematics));
  }

  /** Class that contains left and right wheel velocities */
  public static class DriveVelocity {
    @Override
    public String toString() {
      return String.format(
          "DriveVelocity [leftVelocity=%s, rightVelocity=%s]", leftVelocity, rightVelocity);
    }

    public final double leftVelocity;
    public final double rightVelocity;

    public DriveVelocity(double leftVelocity, double rightVelocity) {
      this.leftVelocity = leftVelocity;
      this.rightVelocity = rightVelocity;
    }
  }

  /** Uses inverse kinematics to convert a Twist2d into left and right wheel velocities */
  public static DriveVelocity inverseKinematics(Twist2d velocity) {
    if (Math.abs(velocity.deltaTheta) < MAX_ERROR) {
      return new DriveVelocity(velocity.deltaX, velocity.deltaX);
    }
    double deltaV =
        ChezyConstants.kDriveWheelTrackWidthInches
            * velocity.deltaTheta
            / (2 * Robot.ROBOT_CONFIGURATION.getScrubFactor());
    return new DriveVelocity(velocity.deltaX - deltaV, velocity.deltaX + deltaV);
  }
}
