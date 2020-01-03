package net.teamrush27.frc2019.auto.actions.impl;

import edu.wpi.first.wpilibj.Timer;
import net.teamrush27.frc2019.subsystems.impl.Gripper;
import net.teamrush27.frc2019.subsystems.impl.Gripper.WantedState;
import org.apache.logging.log4j.LogManager;
import org.apache.logging.log4j.Logger;

public class GripperStateAction extends RunOnceAction {

  private static final Logger LOG = LogManager.getLogger(GripperStateAction.class);

  private WantedState wantedState;
  private double delay;
  private double startTime;

  public GripperStateAction(WantedState wantedState) {
    this(wantedState, 0);
  }

  public GripperStateAction(WantedState wantedState, double delay) {
    this.wantedState = wantedState;
    this.delay = delay;
  }

  @Override
  public void runOnce() {
    Gripper.getInstance().setWantedState(wantedState);
    this.startTime = Timer.getFPGATimestamp();
  }

  @Override
  public boolean isFinished() {
    if (delay == 0) {
      return true;
    }
    if (Timer.getFPGATimestamp() - startTime > delay) {
      LOG.info("Done setting Gripper State");
      return true;
    }
    return false;
  }
}
