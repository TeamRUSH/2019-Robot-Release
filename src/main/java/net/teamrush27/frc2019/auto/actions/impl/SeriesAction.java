package net.teamrush27.frc2019.auto.actions.impl;

import edu.wpi.first.wpilibj.Timer;
import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;
import net.teamrush27.frc2019.auto.actions.Action;
import org.apache.logging.log4j.LogManager;
import org.apache.logging.log4j.Logger;

/** Executes one action at a time. Useful as a member of {@link ParallelAction} */
public class SeriesAction implements Action {

  private final Logger LOG = LogManager.getLogger(SeriesAction.class);

  private double startTime;
  private Action mCurAction;
  private final List<Action> mRemainingActions;

  public SeriesAction(List<Action> actions) {
    mRemainingActions = new ArrayList<>(actions);
    mCurAction = null;
  }

  public SeriesAction(Action... actions) {
    mRemainingActions = new ArrayList(Arrays.asList(actions));
    mCurAction = null;
  }

  @Override
  public boolean isFinished() {
    return mRemainingActions.isEmpty() && mCurAction == null;
  }

  @Override
  public void start() {
    startTime = Timer.getFPGATimestamp();
  }

  @Override
  public void update() {
    if (mCurAction == null) {
      if (mRemainingActions.isEmpty()) {
        return;
      }

      mCurAction = mRemainingActions.remove(0);
      mCurAction.start();
    }

    mCurAction.update();

    if (mCurAction.isFinished()) {
      mCurAction.done();
      mCurAction = null;

      LOG.info("Action Completed. Elapsed Time: {}", Timer.getFPGATimestamp() - startTime);
    }
  }

  @Override
  public void done() {}
}
