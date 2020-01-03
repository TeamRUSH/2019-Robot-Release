package net.teamrush27.frc2019.auto.modes.singlepath;

import net.teamrush27.frc2019.auto.AutoModeBase;
import net.teamrush27.frc2019.auto.AutoModeEndedException;
import net.teamrush27.frc2019.auto.actions.impl.DriveTrajectory;
import net.teamrush27.frc2019.util.trajectory.Trajectory;
import net.teamrush27.frc2019.util.trajectory.TrajectoryGenerator;

public class HPToCargoSideCloseRight extends AutoModeBase {

  @Override
  protected void routine() throws AutoModeEndedException {
    Trajectory hpToCargoSideClose =
        TrajectoryGenerator.getInstance().getTrajectorySet().hpToCargoSideClose.getRight();

    runAction(new DriveTrajectory(hpToCargoSideClose, true, true));
  }
}
