package net.teamrush27.frc2019.auto.modes;

import java.util.Arrays;
import java.util.List;
import net.teamrush27.frc2019.auto.AutoModeBase;
import net.teamrush27.frc2019.auto.AutoModeEndedException;
import net.teamrush27.frc2019.auto.actions.impl.AutoSuperstructurePosition;
import net.teamrush27.frc2019.auto.actions.impl.DriveTrajectory;
import net.teamrush27.frc2019.auto.actions.impl.GripperStateAction;
import net.teamrush27.frc2019.auto.actions.impl.LimelightTrackingAction;
import net.teamrush27.frc2019.auto.actions.impl.ParallelAction;
import net.teamrush27.frc2019.auto.actions.impl.SeriesAction;
import net.teamrush27.frc2019.auto.actions.impl.WaitAction;
import net.teamrush27.frc2019.managers.SuperstructureManager.WantedState;
import net.teamrush27.frc2019.subsystems.impl.Gripper;
import net.teamrush27.frc2019.util.trajectory.Trajectory;
import net.teamrush27.frc2019.util.trajectory.TrajectoryGenerator;

public class RightCloseCargo extends AutoModeBase {

  @Override
  protected void routine() throws AutoModeEndedException {

    Trajectory habToCargoFront =
        TrajectoryGenerator.getInstance().getTrajectorySet().habToCargoFront.getRight();

    Trajectory cargoFrontToHP =
        TrajectoryGenerator.getInstance().getTrajectorySet().cargoFrontToHP.getRight();

    Trajectory hpToCargoSideClose =
        TrajectoryGenerator.getInstance().getTrajectorySet().hpToCargoSideClose.getRight();

    List commands =
        Arrays.asList(
            new ParallelAction(
                new AutoSuperstructurePosition(WantedState.ROCKET_LEVEL_1, true, true),
                new DriveTrajectory(habToCargoFront, true, true)),
            new LimelightTrackingAction(true, 400),
            new GripperStateAction(Gripper.WantedState.EXHAUST_HATCH, 0.1),
            new ParallelAction(
                new SeriesAction(
                    new WaitAction(0.5),
                    new GripperStateAction(Gripper.WantedState.INTAKE_HATCH),
                    new AutoSuperstructurePosition(WantedState.ROCKET_LEVEL_1, false, true)),
                new DriveTrajectory(cargoFrontToHP, false, false)),
            new LimelightTrackingAction(false, 400),
            new ParallelAction(
                new SeriesAction(
                    new WaitAction(0.5),
                    new AutoSuperstructurePosition(WantedState.ROCKET_LEVEL_1, true, true)),
                new DriveTrajectory(hpToCargoSideClose, false, false)),
            new LimelightTrackingAction(true, 400));

    runAction(new SeriesAction(commands));
  }
}
