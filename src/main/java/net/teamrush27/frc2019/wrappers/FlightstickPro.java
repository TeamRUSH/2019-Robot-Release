package net.teamrush27.frc2019.wrappers;

import edu.wpi.first.wpilibj.Joystick;

public class FlightstickPro extends Joystick {

  public FlightstickPro(int port) {
    super(port);
  }

  // FlightstickPro
  // trigger = button 1
  // left button = button 2
  // right button = button 3
  // center button = button 4

  public double getThrottle() {
    return super.getZ();
  }

  public boolean getLeftButton() {
    return super.getRawButton(2);
  }

  public boolean getCenterButton() {
    return super.getRawButton(4);
  }

  public boolean getRightButton() {
    return super.getRawButton(3);
  }
}
