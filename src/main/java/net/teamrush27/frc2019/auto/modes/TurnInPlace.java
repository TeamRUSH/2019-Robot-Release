package net.teamrush27.frc2019.auto.modes;

import net.teamrush27.frc2019.auto.AutoModeBase;
import net.teamrush27.frc2019.auto.AutoModeEndedException;
import net.teamrush27.frc2019.auto.actions.impl.TurnInPlaceAction;

public class TurnInPlace extends AutoModeBase {

  @Override
  protected void routine() throws AutoModeEndedException {
    runAction(new TurnInPlaceAction(2 * Math.PI * 5, 18));
  }
}
