package net.teamrush27.frc2019.constants;

public class ChezyConstants {
  // DRIVE CONSTANTS
  public static double PID_P = 0.0; // 1.0; // 105.5/2617.26 * 1023 / 105.5 err at 60 in/s;
  public static double PID_I = 0.0; // 0.002;
  public static double PID_D = 0.0; // 10.0; // 100.0;
  public static double PID_F = 0.0; // .3908; // 1023 / 2617.26;
  public static int PID_I_ZONE = 700;
  public static double PID_RAMP_RATE = .100; // V/s

  public static double ROTATE_PID_P = 0.4; // 1.0;
  public static double ROTATE_PID_I = 0.0; // 0.002;
  public static double ROTATE_PID_D = 0.0; // 100.0;
  public static double ROTATE_PID_F = 0.75; // .45;
  public static int ROTATE_PID_I_ZONE = 700;

  public static double NOMINAL_OUTPUT = 0.5; // V

  // 13 f/s in in/s
  public static double MAX_VELOCITY = 13.0 * 12.0;

  // 5 f/s/s in in/s/s
  public static double MAX_ACCELERATION = 5.0 * 12.0;

  // FOLLOWING CONSTANTS
  public static double SEGMENT_COMPLETION_TOLERANCE = 0.1; // inches

  public static final double FOLLOWING_MAX_ACCELERATION = 120.0; // inches per second ^2
  public static double MAX_SPEED = 120.0; // inches per second
  public static double P = 0.4;
  public static double I = 0;
  public static double D = 0;
  public static double V = 1;
  public static double A = 0.01;
  public static double FEED_FORWARD_VELOCITY = 1;
  public static double FEED_FORWARD_ACCELERATION = 0.05;
  public static double GOAL_POSITION_TOLERANCE = 0.75;
  public static double GOAL_VELOCITY_TOLERANCE = 12.0;
  public static double STOP_STEERING_DISTANCE = 5.0;

  public static double MIN_LOOKAHEAD_DISTANCE = 12.0; // inches
  public static double MIN_LOOKAHEAD_SPEED = 9.0; // inches per second
  public static double MAX_LOOKAHEAD_DISTANCE = 24.0; // inches
  public static double MAX_LOOKAHEAD_SPEED = 120.0; // inches per second
  public static double LOOKAHEAD_DISTANCE_WINDOW = MAX_LOOKAHEAD_DISTANCE - MIN_LOOKAHEAD_DISTANCE;
  public static double LOOKAHEAD_SPEED_WINDOW = MAX_LOOKAHEAD_SPEED - MIN_LOOKAHEAD_SPEED;

  public static double INTERIAL_STEERING_GAIN =
      0.0; // angular velocity command is multiplied by this gain *
  // our speed
  // in inches per sec

  // ROBOT CONSTANTS
  // Wheel Base
  public static double DRIVE_WHEEL_DIAMETER = 7.75; // in inches
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
  public static int TALON_CONFIG_TIMEOUT = 0; // 100ms

  // Drive Motion Constants
  public static double kDriveWheelDiameterInches = 4;
  public static double kDriveWheelRadiusInches = kDriveWheelDiameterInches / 2;
  public static double kRobotLinearInertia = 70.0; // fat boy
  public static double kRobotAngularInertia = 15.0;
  public static double kRobotAngularDrag = 12.0;
  public static double kDriveWheelTrackWidthInches = 25.0;
  public static double kPathLookaheadTime = 0.1;
  public static double kPathMinLookaheadDistance = 3.0;
  public static double kPathKX = 5.0;

  /*
      * ks: 2.3183635008695855
  2019-03-20 23:04:29,784 INFO kv: 0.06657814178271085
  2019-03-20 23:04:29,785 INFO ka: 0.0233624038182055


      *
      * */
}
