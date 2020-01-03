package net.teamrush27.frc2019.auto;

import edu.wpi.first.wpilibj.DriverStation;
import net.teamrush27.frc2019.auto.actions.Action;
import org.apache.logging.log4j.LogManager;
import org.apache.logging.log4j.Logger;

/**
 * An abstract class that is the basis of the robot's autonomous routines. This is implemented in
 * auto modes (which are routines that do actions).
 */
public abstract class AutoModeBase {
  private static final Logger LOG = LogManager.getLogger(AutoModeBase.class);

  protected double mUpdateRate = 1.0 / 50.0;
  protected boolean mActive = false;

  protected abstract void routine() throws AutoModeEndedException;

  public void run() {
    mActive = true;

    try {
      routine();
    } catch (AutoModeEndedException e) {
      DriverStation.reportError("AUTO MODE DONE!!!! ENDED EARLY!!!!", false);
      return;
    }

    done();
  }

  public void done() {
    LOG.info("Auto mode done");
    mActive = false;
  }

  public void stop() {
    mActive = false;
  }

  public boolean isActive() {
    return mActive;
  }

  public boolean isActiveWithThrow() throws AutoModeEndedException {
    if (!isActive()) {
      throw new AutoModeEndedException();
    }

    return isActive();
  }

  public void runAction(Action action) throws AutoModeEndedException {
    isActiveWithThrow();
    action.start();

    while (isActiveWithThrow() && !action.isFinished()) {
      action.update();
      long waitTime = (long) (mUpdateRate * 1000.0);

      try {
        Thread.sleep(waitTime);
      } catch (InterruptedException e) {
        e.printStackTrace();
      }
    }

    action.done();
  }
}
