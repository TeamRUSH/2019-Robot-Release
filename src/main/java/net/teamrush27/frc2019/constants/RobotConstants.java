package net.teamrush27.frc2019.constants;

public class RobotConstants {

  // Wheel Base
  public static double DRIVE_WHEEL_DIAMETER = 4; // in inches
  public static double TRACK_WIDTH = 25; // in inches

  // Extremities
  // below in inches
  public static double CENTER_TO_FRONT_BUMPER = 14.5;
  public static double CENTER_TO_EXTENDED_HARVESTER = 23.11;
  public static double CENTER_TO_REAR_BUMPER = 14.5;
  public static double CENTER_TO_SIDE_BUMPER = 19.25;

  // Camera Position
  public static double CAMERA_X_OFFSET = 0.0;
  public static double CAMERA_Y_OFFSET = 0.0;
  public static double CAMERA_Z_OFFSET = 0.0;
  public static double CAMERA_PITCH_ANGLE = 0.0; // Measured on 4/26
  public static double CAMERA_YAW_ANGLE = 0.0;
  public static double CAMERA_DEADBAND = 0.0;

  // Looping Parameters
  public static double LOOPER_DELTA_TIME = 0.005; // 200hz

  // Talon Parameters
  public static int TALON_CONFIG_TIMEOUT = 100; // 100ms
}
