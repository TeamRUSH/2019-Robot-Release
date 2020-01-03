package net.teamrush27.frc2019.util.math;

import java.text.DecimalFormat;
import net.teamrush27.frc2019.util.CSVWritable;

/**
 * A movement along an arc at constant curvature and velocity. We can use ideas from "differential
 * calculus" to create new Pose2d's from a Twist2d and visa versa.
 *
 * <p>A Twist can be used to represent a difference between two poses, a velocity, an acceleration,
 * etc.
 */
public class Twist2d implements CSVWritable {
  protected static final Twist2d IDENTITY = new Twist2d(0.0, 0.0, 0.0);

  public static final Twist2d identity() {
    return IDENTITY;
  }

  public final double deltaX;
  public final double deltaY;
  public final double deltaTheta; // Radians!

  public Twist2d(double deltaX, double deltaY, double deltaTheta) {
    this.deltaX = deltaX;
    this.deltaY = deltaY;
    this.deltaTheta = deltaTheta;
  }

  public Twist2d scaled(double scale) {
    return new Twist2d(deltaX * scale, deltaY * scale, deltaTheta * scale);
  }

  public double norm() {
    // Common case of dy == 0
    if (deltaY == 0.0) return Math.abs(deltaX);
    return Math.hypot(deltaX, deltaY);
  }

  public double curvature() {
    if (Math.abs(deltaTheta) < MathUtils.DEFAULT_MAX_ERROR && norm() < MathUtils.DEFAULT_MAX_ERROR)
      return 0.0;
    return deltaTheta / norm();
  }

  @Override
  public String toString() {
    final DecimalFormat fmt = new DecimalFormat("#0.000");
    return "("
        + fmt.format(deltaX)
        + ","
        + fmt.format(deltaY)
        + ","
        + fmt.format(Math.toDegrees(deltaTheta))
        + " deg)";
  }

  @Override
  public String toCSV() {
    final DecimalFormat fmt = new DecimalFormat("#0.000");
    return fmt.format(deltaX)
        + ", "
        + fmt.format(deltaY)
        + ", "
        + fmt.format(Math.toDegrees(deltaTheta));
  }

  @Override
  public String header(String base) {
    return base + "_x, " + base + "_y," + base + "_theta";
  }
}
