package net.teamrush27.frc2019.subsystems.impl.enumerated;

public enum ShiftState {
  LOW_GEAR(1, 0),
  HIGH_GEAR(0, 1),
  UNKNOWN(.5, .5);

  private final double leftSetpoint;
  private final double rightSetpoint;

  ShiftState(double left, double right) {
    leftSetpoint = left;
    rightSetpoint = right;
  }

  public double getLeft() {
    return leftSetpoint;
  }

  public double getRight() {
    return rightSetpoint;
  }
}
