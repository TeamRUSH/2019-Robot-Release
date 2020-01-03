package net.teamrush27.frc2019.util.interpolate;

/**
 * A Double that can be interpolated using the InterpolatingTreeMap.
 *
 * @see InterpolatingTreeMap
 */
public class InterpolatingDouble
    implements Interpolable<InterpolatingDouble>,
        InverseInterpolable<InterpolatingDouble>,
        Comparable<InterpolatingDouble> {
  public Double value = 0.0;

  public InterpolatingDouble(Double val) {
    value = val;
  }

  @Override
  public InterpolatingDouble interpolate(InterpolatingDouble other, double x) {
    Double dydx = other.value - value;
    Double searchY = dydx * x + value;
    return new InterpolatingDouble(searchY);
  }

  @Override
  public double inverseInterpolate(InterpolatingDouble upper, InterpolatingDouble query) {
    double upperToLower = upper.value - value;
    if (upperToLower <= 0) {
      return 0;
    }
    double queryToLower = query.value - value;
    if (queryToLower <= 0) {
      return 0;
    }
    return queryToLower / upperToLower;
  }

  @Override
  public int compareTo(InterpolatingDouble other) {
    if (other.value < value) {
      return 1;
    } else if (other.value > value) {
      return -1;
    } else {
      return 0;
    }
  }
}
