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
import net.teamrush27.frc2019.auto.actions.impl.WaitUntilCrossXBoundaryCommand;
import net.teamrush27.frc2019.managers.SuperstructureManager.WantedState;
import net.teamrush27.frc2019.subsystems.impl.Gripper;
import net.teamrush27.frc2019.util.trajectory.Trajectory;
import net.teamrush27.frc2019.util.trajectory.TrajectoryGenerator;

public class RightRocket extends AutoModeBase {

  @Override
  protected void routine() throws AutoModeEndedException {
    Trajectory habToFarRocket =
        TrajectoryGenerator.getInstance().getTrajectorySet().habToRocketRear.getRight();
    Trajectory altRocketRearToHP =
        TrajectoryGenerator.getInstance().getTrajectorySet().altRocketRearToHP.getRight();
    Trajectory hpToRocketClose =
        TrajectoryGenerator.getInstance().getTrajectorySet().hpToRocketFront.getRight();

    Trajectory scootBack =
        TrajectoryGenerator.getInstance().getTrajectorySet().scootBack.getRight();

    List commands =
        Arrays.asList(
            new ParallelAction(
                new SeriesAction(
                    new WaitUntilCrossXBoundaryCommand(260),
                    new AutoSuperstructurePosition(WantedState.ROCKET_LEVEL_2, false, true)),
                new DriveTrajectory(habToFarRocket, true, true)),
            new LimelightTrackingAction(false, 1600),
            new GripperStateAction(Gripper.WantedState.EXHAUST_HATCH, .1),
            new ParallelAction(
                new DriveTrajectory(altRocketRearToHP, false),
                new SeriesAction(
                    new WaitAction(0.5),
                    new GripperStateAction(Gripper.WantedState.INTAKE_HATCH),
                    new AutoSuperstructurePosition(WantedState.HUMAN_LOAD, true, true))),
            new LimelightTrackingAction(true, 400),
            new ParallelAction(
                new SeriesAction(
                    new WaitAction(0.5),
                    new AutoSuperstructurePosition(WantedState.ROCKET_LEVEL_2, false, true)),
                new DriveTrajectory(hpToRocketClose, false)) /*,
        new LimelightTrackingAction(false, 1700)/*,
        new GripperStateAction(Gripper.WantedState.EXHAUST_HATCH, .4),
        new DriveTrajectory(scootBack, false),
        new GripperStateAction(Gripper.WantedState.INTAKE_HATCH),
        new AutoSuperstructurePosition(WantedState.STOW, false, false)
    */);

    runAction(new SeriesAction(commands));
  }
}
