package net.teamrush27.frc2019.constants;

public class TrackingConstants {
  public static final double HALF_FOV = Math.PI / 4;
  public static final double FOV_RADIUS = 96;

  public static double MAX_GOAL_TRACK_AGE = 1.0;
  public static double MAX_TRACKER_DISTANCE = 18.0;
  public static double CAMERA_FRAME_RATE = 30.0;
  public static double TRACK_REPORT_COMPARATOR_STABILITY_WEIGHT = 1.0;
  public static double TRACK_REPORT_COMPARATOR_AGE_WEIGHT = 1.0;
  public static double ON_TARGET_ERROR_THRESHOLD = 3.0;

  public static double GOAL_POSITION_TOLERANCE = 1; // inches
  public static double GOAL_VELOCITY_TOLERANCE = 5.0; // inches per second
}
