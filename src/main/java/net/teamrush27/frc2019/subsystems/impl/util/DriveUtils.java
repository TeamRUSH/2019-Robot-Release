package net.teamrush27.frc2019.subsystems.impl.util;

import net.teamrush27.frc2019.constants.RobotConstants;

/**
 * Moved utility methods to own class
 *
 * @author cyocom
 * @author team254
 */
public class DriveUtils {
  public static double rotationsToInches(double rotations) {
    return rotations * (RobotConstants.DRIVE_WHEEL_DIAMETER * Math.PI);
  }

  public static double rpmToInchesPerSecond(double rpm) {
    return rotationsToInches(rpm) / 60.0;
  }

  public static double inchesToRotations(double inches) {
    return inches / (RobotConstants.DRIVE_WHEEL_DIAMETER * Math.PI);
  }

  public static double radiansToRotations(double radians) {
    return radians / (Math.PI * 2);
  }

  public static int inchesPerSecondToEncoderCountPer100ms(double inches_per_second) {
    return Double.valueOf(
            rotationsToEncoderCount(inchesPerSecondToRotationsPer100ms(inches_per_second)))
        .intValue();
  }

  public static double radiansPerSecondToEncoderCountPer100ms(double rads_per_second) {
    return rotationsToEncoderCount(radiansToRotations(rads_per_second)) / 10;
  }

  public static double rotationsToEncoderCount(double rotations) {
    return rotations * 4096.0;
  }

  public static double inchesPerSecondToRotationsPer100ms(double inches_per_second) {
    return inchesToRotations(inches_per_second) / 10.0;
  }

  public static double encoderCountToInches(double encoderCount) {
    return rotationsToInches(encoderCountToRotations(encoderCount));
  }

  public static int inchesToEncoderCount(double inches) {
    Double encoderCount = rotationsToEncoderCount(inchesToRotations(inches));
    return encoderCount.intValue();
  }

  public static double encoderCountToRotations(double encoderCount) {
    return encoderCount / 4096.0;
  }
}
