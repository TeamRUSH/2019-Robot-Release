package net.teamrush27.frc2019.auto.modes.singlepath;

import net.teamrush27.frc2019.auto.AutoModeBase;
import net.teamrush27.frc2019.auto.AutoModeEndedException;
import net.teamrush27.frc2019.auto.actions.impl.DriveTrajectory;
import net.teamrush27.frc2019.util.trajectory.Trajectory;
import net.teamrush27.frc2019.util.trajectory.TrajectoryGenerator;

public class HabToCargoFrontRight extends AutoModeBase {

  @Override
  protected void routine() throws AutoModeEndedException {
    Trajectory habToCargoFront =
        TrajectoryGenerator.getInstance().getTrajectorySet().habToCargoFront.getRight();

    runAction(new DriveTrajectory(habToCargoFront, true, true));
  }
}
