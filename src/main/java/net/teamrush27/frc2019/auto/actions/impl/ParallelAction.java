package net.teamrush27.frc2019.auto.actions.impl;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;
import net.teamrush27.frc2019.auto.actions.Action;

/**
 * Composite action, running all sub-actions at the same time. All actions are started then
 * periodically updated until all actions report being done.
 */
public class ParallelAction implements Action {

  private final List<Action> mActions;

  public ParallelAction(List<Action> actions) {
    mActions = new ArrayList<>(actions);
  }

  public ParallelAction(Action... actions) {
    mActions = Arrays.asList(actions);
  }

  @Override
  public boolean isFinished() {
    for (Action action : mActions) {
      if (!action.isFinished()) {
        return false;
      }
    }
    return true;
  }

  @Override
  public void update() {
    for (Action action : mActions) {
      action.update();
    }
  }

  @Override
  public void done() {
    for (Action action : mActions) {
      action.done();
    }
  }

  @Override
  public void start() {
    for (Action action : mActions) {
      action.start();
    }
  }
}
