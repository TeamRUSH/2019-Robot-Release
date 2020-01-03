package net.teamrush27.frc2019.util.math;

import java.util.List;

/** Contains basic math functions that are used often. */
public class MathUtils {

  public static double DEFAULT_MAX_ERROR = 1e-6;

  /** Prevent this class from being instantiated. */
  private MathUtils() {}

  /** Limits the given input to the given magnitude. */
  public static double limit(double input, double maxMagnitude) {
    return limit(input, -maxMagnitude, maxMagnitude);
  }

  public static double limit(double input, double min, double max) {
    return Math.min(max, Math.max(min, input));
  }

  public static boolean epsilonEquals(double a, double b) {
    return epsilonEquals(a, b, DEFAULT_MAX_ERROR);
  }

  public static boolean epsilonEquals(double a, double b, double epsilon) {
    return (a - epsilon <= b) && (a + epsilon >= b);
  }

  public static boolean allCloseTo(List<Double> list, double value, double epsilon) {
    boolean result = true;
    for (Double value_in : list) {
      result &= epsilonEquals(value_in, value, epsilon);
    }
    return result;
  }

  public static double interpolate(double a, double b, double x) {
    x = limit(x, 0.0, 1.0);
    return a + (b - a) * x;
  }
}
