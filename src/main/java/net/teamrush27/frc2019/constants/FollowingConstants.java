package net.teamrush27.frc2019.constants;

public class FollowingConstants {
  public static double SEGMENT_COMPLETION_TOLERANCE = 0.1; // inches

  public static final double MAX_ACCELERATION = 120.0; // inches per second ^2
  public static double MAX_SPEED = 120.0; // inches per second
  public static double P = .01;
  public static double I = 0;
  public static double D = 20;
  public static double V = .08;
  public static double A = 0;
  public static double FEED_FORWARD_VELOCITY = 1;
  public static double FEED_FORWARD_ACCELERATION = 0.05;
  public static double GOAL_POSITION_TOLERANCE = 0.75;
  public static double GOAL_VELOCITY_TOLERANCE = 12.0;
  public static double STOP_STEERING_DISTANCE = 9.0;

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
}
