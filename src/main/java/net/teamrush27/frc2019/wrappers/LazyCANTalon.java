package net.teamrush27.frc2019.wrappers;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.DemandType;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

/**
 * @author team254 This class is a thin wrapper around the CANTalon that reduces CAN bus / CPU
 *     overhead by skipping duplicate set commands. (By default the Talon flushes the Tx buffer on
 *     every set call).
 */
public class LazyCANTalon extends TalonSRX {

  protected double lastValue = Double.NaN;
  protected double lastDemand = Double.NaN;
  protected ControlMode lastControlMode = null;
  protected DemandType lastDemandType = null;
  protected int controlPeriodMs = 5;
  private int count = 0;

  public LazyCANTalon(int deviceNumber, int controlPeriodMs) {
    super(deviceNumber);
    this.controlPeriodMs = controlPeriodMs;
  }

  public LazyCANTalon(int deviceNumber) {
    super(deviceNumber);
  }

  @Override
  public void set(ControlMode controlMode, double value) {
    if (value != lastValue || controlMode != lastControlMode) {
      lastValue = value;
      lastControlMode = controlMode;
      if (count++ % 100 == 0 || !controlMode.equals(ControlMode.Velocity)) {
        //				System.out.println(String.format("changed talon %s to %s - %s",getDeviceID(),
        // controlMode.name(), value));
      }
      /*if (controlMode == ControlMode.Velocity) {
        new Throwable().printStackTrace(System.out);
      }*/
      //            setControlFramePeriod(ControlFrame.Control_3_General,controlPeriodMs);
      //            setControlFramePeriod(ControlFrame.Control_4_Advanced,controlPeriodMs);
      super.set(controlMode, value);
    }
  }

  @Override
  public void set(ControlMode controlMode, double value, DemandType demandType, double demand) {
    if (value != lastValue
        || controlMode != lastControlMode
        || demandType != lastDemandType
        || demand != lastDemand) {
      lastValue = value;
      lastControlMode = controlMode;
      lastDemandType = demandType;
      lastDemand = demand;

      super.set(controlMode, value, demandType, demand);
    }
  }

  public void configurePIDF(
      int control_slot, double kp, double ki, double kd, double kf, int i_zone, int timeout) {
    config_kP(control_slot, kp, timeout);
    config_kI(control_slot, ki, timeout);
    config_kD(control_slot, kd, timeout);
    config_kF(control_slot, kf, timeout);
    config_IntegralZone(control_slot, i_zone, timeout);
  }
}
