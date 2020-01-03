package net.teamrush27.frc2019.auto.actions.impl;

import net.teamrush27.frc2019.auto.actions.Action;
import net.teamrush27.frc2019.base.RobotState;

public class WaitUntilCrossXBoundaryCommand implements Action {

  private double mXBoundary = 0;
  private boolean behind;

  public WaitUntilCrossXBoundaryCommand(double x) {
    this(x, false);
  }

  public WaitUntilCrossXBoundaryCommand(double x, boolean behind) {
    mXBoundary = x;
    this.behind = behind;
  }

  @Override
  public boolean isFinished() {
    return (RobotState.getInstance().getLatestFieldToVehicle().getValue().getTranslation().x()
            > mXBoundary)
        ^ behind;
  }

  @Override
  public void update() {}

  @Override
  public void done() {}

  @Override
  public void start() {}
}
