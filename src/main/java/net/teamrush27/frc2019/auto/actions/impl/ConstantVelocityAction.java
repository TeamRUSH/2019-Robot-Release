package net.teamrush27.frc2019.auto.actions.impl;

import net.teamrush27.frc2019.auto.actions.Action;
import net.teamrush27.frc2019.subsystems.impl.Drivetrain;
import net.teamrush27.frc2019.subsystems.impl.dto.DriveCommand;

public class ConstantVelocityAction implements Action {
  private static final Drivetrain mDrive = Drivetrain.getInstance();
  private double speed;

  public ConstantVelocityAction(double speed) {
    this.speed = speed;
  }

  @Override
  public boolean isFinished() {
    return false;
  }

  @Override
  public void update() {}

  @Override
  public void done() {}

  @Override
  public void start() {
    mDrive.setVelocitySetpoint(new DriveCommand(speed, speed));
  }
}
