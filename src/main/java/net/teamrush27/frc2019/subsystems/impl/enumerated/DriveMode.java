package net.teamrush27.frc2019.subsystems.impl.enumerated;

import com.ctre.phoenix.motorcontrol.ControlMode;

public enum DriveMode {
  OPEN_LOOP(ControlMode.PercentOutput), // open loop voltage control
  VELOCITY_SETPOINT(ControlMode.Velocity), // velocity PID control
  CHEZY_PATH_FOLLOWING(ControlMode.Velocity), // used for autonomous driving
  TURN_TO_HEADING(ControlMode.MotionMagic), // turn in place
  LIMELIGHT_STEERING(ControlMode.PercentOutput); // open loop control but limelight for

  private final ControlMode requestedControlMode;

  DriveMode(ControlMode requestedControlMode) {
    this.requestedControlMode = requestedControlMode;
  }

  public ControlMode getRequestedControlMode() {
    return requestedControlMode;
  }
}
