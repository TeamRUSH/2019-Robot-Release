package net.teamrush27.frc2019.auto.modes.singlepath;

import net.teamrush27.frc2019.auto.AutoModeBase;
import net.teamrush27.frc2019.auto.AutoModeEndedException;
import net.teamrush27.frc2019.auto.actions.impl.DriveTrajectory;
import net.teamrush27.frc2019.util.trajectory.Trajectory;
import net.teamrush27.frc2019.util.trajectory.TrajectoryGenerator;

public class CargoFrontToHPRight extends AutoModeBase {

  @Override
  protected void routine() throws AutoModeEndedException {
    Trajectory cargoFrontToHP =
        TrajectoryGenerator.getInstance().getTrajectorySet().cargoFrontToHP.getRight();

    runAction(new DriveTrajectory(cargoFrontToHP, true, true));
  }
}
