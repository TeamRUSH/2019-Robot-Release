package net.teamrush27.frc2019.util.motion;

import net.teamrush27.frc2019.util.math.Pose2d;
import net.teamrush27.frc2019.util.math.Twist2d;

public interface IPathFollower {
  public Twist2d steer(Pose2d current_pose);

  public boolean isDone();
}
