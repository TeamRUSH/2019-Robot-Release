package net.teamrush27.frc2019.subsystems;

import net.teamrush27.frc2019.loops.ILooper;
import net.teamrush27.frc2019.subsystems.impl.dto.SmartDashboardCollection;

public abstract class Subsystem {

  public abstract void outputToSmartDashboard(SmartDashboardCollection collection);

  public abstract void stop();

  public abstract void zeroSensors();

  public abstract void registerEnabledLoops(ILooper enabledLooper);

  public abstract void test();

  // Optional design pattern for caching periodic reads to avoid hammering the HAL/CAN.
  public void readPeriodicInputs() {}

  // Optional design pattern for caching periodic writes to avoid hammering the HAL/CAN.
  public void writePeriodicOutputs() {}

  public String id() {
    return "IMPLEMENT ME";
  }
}
