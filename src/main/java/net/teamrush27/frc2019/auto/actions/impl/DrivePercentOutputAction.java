package net.teamrush27.frc2019.auto.actions.impl;

import net.teamrush27.frc2019.subsystems.impl.Drivetrain;
import net.teamrush27.frc2019.subsystems.impl.dto.DriveCommand;

public class DrivePercentOutputAction extends RunOnceAction {
  private DriveCommand driveCommand;
  private Drivetrain drivetrain = Drivetrain.getInstance();

  public DrivePercentOutputAction(DriveCommand command) {
    this.driveCommand = command;
  }

  @Override
  public void runOnce() {
    drivetrain.setOpenLoop(driveCommand);
  }
}
