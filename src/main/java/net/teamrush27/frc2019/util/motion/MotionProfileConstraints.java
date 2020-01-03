package net.teamrush27.frc2019.util.motion;

public class MotionProfileConstraints {
  /** A constant for consistent floating-point equality checking within this library. */
  protected double maxAbsoluteVelocity = Double.POSITIVE_INFINITY;

  protected double maxAbsoluteAcceleration = Double.POSITIVE_INFINITY;

  public MotionProfileConstraints(double maxVelocity, double maxAcceleration) {
    this.maxAbsoluteVelocity = Math.abs(maxVelocity);
    this.maxAbsoluteAcceleration = Math.abs(maxAcceleration);
  }

  /** @return The (positive) maximum allowed velocity. */
  public double maxAbsoluteVelocity() {
    return maxAbsoluteVelocity;
  }

  /** @return The (positive) maximum allowed acceleration. */
  public double maxAbsoluteAcceleration() {
    return maxAbsoluteAcceleration;
  }

  @Override
  public boolean equals(Object obj) {
    if (!(obj instanceof MotionProfileConstraints)) {
      return false;
    }
    final MotionProfileConstraints other = (MotionProfileConstraints) obj;
    return (other.maxAbsoluteAcceleration() == maxAbsoluteAcceleration())
        && (other.maxAbsoluteVelocity() == maxAbsoluteVelocity());
  }

  @Override
  public String toString() {
    final StringBuffer sb = new StringBuffer("MotionProfileConstraints{");
    sb.append("maxAbsoluteVelocity=").append(maxAbsoluteVelocity);
    sb.append(", maxAbsoluteAcceleration=").append(maxAbsoluteAcceleration);
    sb.append('}');
    return sb.toString();
  }
}
