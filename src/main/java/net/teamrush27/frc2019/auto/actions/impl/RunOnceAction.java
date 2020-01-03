package net.teamrush27.frc2019.auto.actions.impl;

import net.teamrush27.frc2019.auto.actions.Action;

public abstract class RunOnceAction implements Action {

  @Override
  public boolean isFinished() {
    return true;
  }

  @Override
  public void update() {}

  @Override
  public void done() {}

  @Override
  public void start() {
    runOnce();
  }

  public abstract void runOnce();
}
