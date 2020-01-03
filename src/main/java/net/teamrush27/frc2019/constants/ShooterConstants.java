package net.teamrush27.frc2019.constants;

import net.teamrush27.frc2019.util.interpolate.InterpolatingDouble;
import net.teamrush27.frc2019.util.interpolate.InterpolatingTreeMap;
import net.teamrush27.frc2019.util.math.PolynomialRegression;

public class ShooterConstants {
  public static boolean IS_TUNING = false;
  public static double TUNING_FLOOR = 2900;
  public static double TUNING_CEILING = 3500;
  public static double TUNING_RPM_STEP = 50;
  public static double TUNING_RPM = 3500.0;

  public static double PID_P = 0.16;
  public static double PID_I = 0.00008;
  public static double PID_D = 0.0;
  public static double PID_F = 0.035;
  public static double RAMP_RATE = 60.0;

  public static double HOLD_PID_P = 0.0;
  public static double HOLD_PID_I = 0.0;
  public static double HOLD_PID_D = 0.0;

  public static double HOLD_RAMP_RATE = 720.0;

  public static int PID_I_ZONE = 1000; // 73 rpm

  public static double SETPOINT_DEADBAND_RPM = 1.0;

  // Used to determine when to switch to hold profile.
  public static double MIN_TRACK_STABILITY = 0.25;
  public static double START_ON_TARGET_RPM = 50.0;
  public static double STOP_ON_TARGET_RPM = 150.0;
  public static int F_BUFFER_SIZE = 20;
  public static int MIN_ON_TARGET_SAMPLES = 20; // Should be <= kShooterKvBufferSize

  public static int JAM_BUFFER_SIZE = 30;
  public static double DISTURBANCE_THRESHOLD = 25;
  public static double UNJAM_TIMEOUT = 1.5; // In secs
  public static double UNJAM_DURATION = 1.0; // In secs
  public static double MIN_SHOOTING_TIME = 1.0; // In secs
  public static double TIME_SHOOTING_TO_RETRACT_ARM = 1.0; // In secs

  public static double SPIN_DOWN_TIME = 0.25;

  public static double DEFAULT_DISTANCE_INCHES = 95.8;
  public static double DEFAULT_RPM = 2950.0;
  public static boolean USE_FLYWHEEL_AUTO_AIM_POLYNOMIAL =
      true; // Change to 'true' to use the best-fit polynomial
  // instead.
  public static InterpolatingTreeMap<InterpolatingDouble, InterpolatingDouble>
      FLYWHEEL_AUTO_AIM_RPM = new InterpolatingTreeMap<>();
  public static PolynomialRegression FLYWHEEL_AUTO_AIM_POLYNOMIAL;

  public static double OPTIMAL_RANGE = 100.0;
  public static double OPTIMAL_RANGE_FLOOR = 95.0;
  public static double OPTIMAL_RANGE_CEILING = 105.0;

  public static double ABSOLUTE_RANGE_FLOOR = 90.0;
  public static double ABSOLUTE_RANGE_CEILING = 130.0;

  public static double[][] DISTANCE_RPM_VALUES = {
    // At champs 4/27
    {90.0, 2890.0},
    {95.0, 2940.0},
    {100.0, 2990.0},
    {105.0, 3025.0},
    {110.0, 3075.0},
    {115.0, 3125.0},
    {120.0, 3175.0},
    {125.0, 3225.0},
    {130.0, 3275.0},
  };

  static {
    for (double[] pair : DISTANCE_RPM_VALUES) {
      FLYWHEEL_AUTO_AIM_RPM.put(new InterpolatingDouble(pair[0]), new InterpolatingDouble(pair[1]));
    }
    DEFAULT_RPM =
        FLYWHEEL_AUTO_AIM_RPM.getInterpolated(
                new InterpolatingDouble(ShooterConstants.DEFAULT_DISTANCE_INCHES))
            .value;

    FLYWHEEL_AUTO_AIM_POLYNOMIAL = new PolynomialRegression(DISTANCE_RPM_VALUES, 2);
  }
}
