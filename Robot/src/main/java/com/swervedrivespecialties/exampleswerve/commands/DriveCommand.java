package com.swervedrivespecialties.exampleswerve.commands;

import com.swervedrivespecialties.exampleswerve.Robot;
import com.swervedrivespecialties.exampleswerve.subsystems.DrivetrainSubsystem;
import edu.wpi.first.wpilibj.command.Command;
import org.frcteam2910.common.math.Vector2;

public class DriveCommand extends Command {
  private DrivetrainSubsystem drivetrain;

  public DriveCommand(DrivetrainSubsystem drivetrain) {
    this.drivetrain = drivetrain;

    requires(drivetrain);
  }

  @Override
  protected void execute() {
    double forward = Robot.getOi().getPrimaryJoystick().getRawAxis(1);
    // Square the forward stick
    forward = Math.copySign(Math.pow(forward, 2.0), forward);

    double strafe = Robot.getOi().getPrimaryJoystick().getRawAxis(0);
    // Square the strafe stick
    strafe = Math.copySign(Math.pow(strafe, 2.0), strafe);

    double rotation = Robot.getOi().getPrimaryJoystick().getRawAxis(4);
    // Square the rotation stick
    rotation = Math.copySign(Math.pow(rotation, 2.0), rotation);

    drivetrain.holonomicDrive(
        new Vector2(deadband(forward, 0.05), deadband(strafe, 0.05)),
        deadband(rotation, 0.05),
        true);
  }

  @Override
  protected boolean isFinished() {
    return false;
  }

  private double deadband(double x, double zero) {
    if (Math.abs(x) < zero) {
      return 0;
    }

    return x;
  }
}
