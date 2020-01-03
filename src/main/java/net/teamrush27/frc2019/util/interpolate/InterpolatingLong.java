package net.teamrush27.frc2019.util.interpolate;

/**
 * A Long that can be interpolated using the InterpolatingTreeMap.
 *
 * @see InterpolatingTreeMap
 */
public class InterpolatingLong
    implements Interpolable<InterpolatingLong>,
        InverseInterpolable<InterpolatingLong>,
        Comparable<InterpolatingLong> {
  public Long value = 0L;

  public InterpolatingLong(Long val) {
    value = val;
  }

  @Override
  public InterpolatingLong interpolate(InterpolatingLong other, double x) {
    Long dydx = other.value - value;
    Double searchY = dydx * x + value;
    return new InterpolatingLong(searchY.longValue());
  }

  @Override
  public double inverseInterpolate(InterpolatingLong upper, InterpolatingLong query) {
    long upperToLower = upper.value - value;
    if (upperToLower <= 0) {
      return 0;
    }
    long queryToLower = query.value - value;
    if (queryToLower <= 0) {
      return 0;
    }
    return queryToLower / (double) upperToLower;
  }

  @Override
  public int compareTo(InterpolatingLong other) {
    if (other.value < value) {
      return 1;
    } else if (other.value > value) {
      return -1;
    } else {
      return 0;
    }
  }
}
