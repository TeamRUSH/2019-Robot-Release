package net.teamrush27.frc2019.wrappers;

import edu.wpi.first.wpilibj.XboxController;

public class LogitechPad extends XboxController {

  public LogitechPad(int port) {
    super(port);
  }

  public Double getRightTriggerAxis() {
    return super.getTriggerAxis(Hand.kRight);
  }

  public Double getLeftTriggerAxis() {
    return super.getTriggerAxis(Hand.kLeft);
  }

  public Double getLeftStickX() {
    return super.getX(Hand.kLeft);
  }

  public Double getLeftStickY() {
    return super.getY(Hand.kLeft);
  }

  public Double getRightStickX() {
    return super.getX(Hand.kRight);
  }

  public Double getRightStickY() {
    return super.getY(Hand.kRight);
  }
}
