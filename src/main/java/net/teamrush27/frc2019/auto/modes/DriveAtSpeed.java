package net.teamrush27.frc2019.auto.modes;

import net.teamrush27.frc2019.auto.AutoModeBase;
import net.teamrush27.frc2019.auto.AutoModeEndedException;
import net.teamrush27.frc2019.auto.actions.impl.ConstantVelocityAction;

public class DriveAtSpeed extends AutoModeBase {

  @Override
  protected void routine() throws AutoModeEndedException {
    runAction(new ConstantVelocityAction(60));
  }
}
