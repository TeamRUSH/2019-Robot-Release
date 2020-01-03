package com.swervedrivespecialties.exampleswerve.commands;

import com.swervedrivespecialties.exampleswerve.Robot;
import com.swervedrivespecialties.exampleswerve.subsystems.DrivetrainSubsystem;
import edu.wpi.first.wpilibj.command.Command;
import org.frcteam2910.common.drivers.Gyroscope;
import org.frcteam2910.common.math.Rotation2;
import org.frcteam2910.common.math.Vector2;

public class PIDHeadingDriveCommand extends Command {
  private DrivetrainSubsystem drivetrain;

  public PIDHeadingDriveCommand(DrivetrainSubsystem drivetrain) {
    this.drivetrain = drivetrain;

    requires(drivetrain);
  }

  double last_rot_err;

  @Override
  protected void execute() {
    double forward = Robot.getOi().getPrimaryJoystick().getRawAxis(1);
    // Square the forward stick
    forward = Math.copySign(Math.pow(forward, 2.0), forward);

    double strafe = Robot.getOi().getPrimaryJoystick().getRawAxis(0);
    // Square the strafe stick
    strafe = Math.copySign(Math.pow(strafe, 2.0), strafe);

    double rot_cmd = 0;

    double head_x = Robot.getOi().getPrimaryJoystick().getRawAxis(4);
    double head_y = -Robot.getOi().getPrimaryJoystick().getRawAxis(5);

    if (Math.sqrt(head_x * head_x + head_y * head_y) > 0.75) {
      Gyroscope gyro = drivetrain.getGyroscope();
      Rotation2 directed_heading =
          new Rotation2(head_x, head_y, true).rotateBy(new Rotation2(0, 1, true));
      Rotation2 err = gyro.getAngle().rotateBy(directed_heading.inverse());

      double rot_err = err.inverse().toRadians() / Math.PI - 1;
      rot_cmd = -rot_err; // - (rot_err - last_rot_err) * 10;

      last_rot_err = rot_err;
    }

    double rotation = Robot.getOi().getPrimaryJoystick().getRawAxis(4);
    // Square the rotation stick
    rotation = Math.copySign(Math.pow(rotation, 2.0), rotation);

    drivetrain.holonomicDrive(
        new Vector2(deadband(forward, 0.05), deadband(strafe, 0.05)), rot_cmd, true);
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
