package net.teamrush27.frc2019.auto.actions.impl;

import net.teamrush27.frc2019.auto.actions.Action;
import net.teamrush27.frc2019.constants.ChezyConstants;
import net.teamrush27.frc2019.subsystems.impl.Drivetrain;
import net.teamrush27.frc2019.subsystems.impl.dto.DriveCommand;
import net.teamrush27.frc2019.util.math.MathUtils;
import net.teamrush27.frc2019.util.math.Rotation2d;
import org.apache.logging.log4j.LogManager;
import org.apache.logging.log4j.Logger;

public class TurnInPlaceAction implements Action {
  private static final Logger LOG = LogManager.getLogger(TurnInPlaceAction.class);

  private Drivetrain drivetrain = Drivetrain.getInstance();
  private double target_radians;
  private double velocity;

  private Rotation2d current_heading;
  private double total_turns;

  public TurnInPlaceAction(double radians, double velocity) {
    this.target_radians = radians;
    this.velocity = velocity;
  }

  @Override
  public boolean isFinished() {
    if (current_heading == null) {
      return false;
    }

    return MathUtils.epsilonEquals(
        Math.abs(total_turns * 2 * Math.PI + current_heading.getRadians()), target_radians, 0.1);
  }

  @Override
  public void update() {
    Rotation2d drive_heading = drivetrain.getHeading();

    // we've gone around from +- 180 to the other
    if (current_heading != null
        && Math.abs(current_heading.getRadians() - drive_heading.getRadians()) > 0.5) {
      total_turns += Math.signum(current_heading.getRadians());

      LOG.info("{} {} {}", total_turns, current_heading, drive_heading);
    }

    current_heading = drive_heading;
  }

  @Override
  public void done() {
    double dist_left = drivetrain.getLeftDistanceInches();
    double dist_right = drivetrain.getRightDistanceInches();

    double avg = (Math.abs(dist_left) + Math.abs(dist_right)) / 2;

    double expected =
        (2 * Math.PI * total_turns + current_heading.getRadians())
            * ChezyConstants.kDriveWheelTrackWidthInches
            / 2;

    LOG.info("Scrub Factor at {} : {}", velocity, avg / expected);

    drivetrain.setVelocitySetpoint(DriveCommand.BRAKE);
  }

  @Override
  public void start() {
    drivetrain.resetEncoders();
    drivetrain.setHeading(Rotation2d.identity());
    drivetrain.setVelocitySetpoint(new DriveCommand(velocity, -velocity, true));
  }
}
