package net.teamrush27.frc2019.util.math;

public interface IPose2d<S> extends IRotation2d<S>, ITranslation2d<S> {
  public Pose2d getPose();

  public S transformBy(Pose2d transform);

  public S mirror();
}