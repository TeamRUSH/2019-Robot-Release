package net.teamrush27.frc2019.util.trajectory.timing;

import net.teamrush27.frc2019.util.math.Pose2dWithCurvature;

public class CentripetalAccelerationConstraint implements TimingConstraint<Pose2dWithCurvature> {
  final double mMaxCentripetalAccel;

  public CentripetalAccelerationConstraint(final double max_centripetal_accel) {
    mMaxCentripetalAccel = max_centripetal_accel;
  }

  @Override
  public double getMaxVelocity(final Pose2dWithCurvature state) {
    return Math.sqrt(Math.abs(mMaxCentripetalAccel / state.getCurvature()));
  }

  @Override
  public MinMaxAcceleration getMinMaxAcceleration(
      final Pose2dWithCurvature state, final double velocity) {
    //    System.out.print("Centripetal: ");
    //    System.out.println(MinMaxAcceleration.kNoLimits);
    return MinMaxAcceleration.kNoLimits;
  }
}
