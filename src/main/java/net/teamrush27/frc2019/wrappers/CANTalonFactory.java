package net.teamrush27.frc2019.wrappers;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.LimitSwitchNormal;
import com.ctre.phoenix.motorcontrol.LimitSwitchSource;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.StatusFrameEnhanced;
import com.ctre.phoenix.motorcontrol.VelocityMeasPeriod;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

/**
 * @author team254 Creates CANTalon objects and configures all the parameters we care about to
 *     factory defaults. Closed-loop and sensor parameters are not set, as these are expected to be
 *     set by the application.
 */
public class CANTalonFactory {

  public static class Configuration {

    public double MAX_OUTPUT_VOLTAGE = 12;
    public double NOMINAL_VOLTAGE = 0;
    public double PEAK_VOLTAGE = 12;
    public boolean ENABLE_BRAKE = false;
    public boolean ENABLE_CURRENT_LIMIT = false;
    public boolean ENABLE_SOFT_LIMIT = false;
    public int CURRENT_LIMIT = 0;
    public int FORWARD_SOFT_LIMIT = 0;
    public boolean INVERTED = false;
    public int REVERSE_SOFT_LIMIT = 0;

    public int CONTROL_FRAME_PERIOD_MS = 5;
    public int MOTION_CONTROL_FRAME_PERIOD_MS = 100;
    public int GENERAL_STATUS_FRAME_RATE_MS = 5;
    public int FEEDBACK_STATUS_FRAME_RATE_MS = 100;
    public int QUAD_ENCODER_STATUS_FRAME_RATE_MS = 100;
    public int ANALOG_TEMP_VBAT_STATUS_FRAME_RATE_MS = 100;
    public int PULSE_WIDTH_STATUS_FRAME_RATE_MS = 100;
    public VelocityMeasPeriod VELOCITY_MEASUREMENT_PERIOD = VelocityMeasPeriod.Period_100Ms;
    public int VELOCITY_MEASUREMENT_ROLLING_AVERAGE_WINDOW = 64;

    public double VOLTAGE_RAMP_RATE = 0;

    public int CONFIG_TIMEOUT_MS = 0;
  }

  private static final Configuration kDefaultConfiguration = new Configuration();
  private static final Configuration kSlaveConfiguration = new Configuration();

  static {
    kSlaveConfiguration.CONTROL_FRAME_PERIOD_MS = 1000;
    kSlaveConfiguration.MOTION_CONTROL_FRAME_PERIOD_MS = 1000;
    kSlaveConfiguration.GENERAL_STATUS_FRAME_RATE_MS = 1000;
    kSlaveConfiguration.FEEDBACK_STATUS_FRAME_RATE_MS = 1000;
    kSlaveConfiguration.QUAD_ENCODER_STATUS_FRAME_RATE_MS = 1000;
    kSlaveConfiguration.ANALOG_TEMP_VBAT_STATUS_FRAME_RATE_MS = 1000;
    kSlaveConfiguration.PULSE_WIDTH_STATUS_FRAME_RATE_MS = 1000;
  }

  // Create a CANTalon with the default (out of the box) configuration.
  public static LazyCANTalon createDefaultTalon(int id) {
    return createTalon(id, kDefaultConfiguration);
  }

  public static TalonSRX createPermanentSlaveTalon(int id, int master_id) {
    final TalonSRX talon = createTalon(id, kSlaveConfiguration);
    talon.set(ControlMode.Follower, master_id);
    return talon;
  }

  public static LazyCANTalon createTalon(int id, Configuration config) {
    LazyCANTalon talon = new LazyCANTalon(id);

    talon.configPeakOutputForward(
        config.MAX_OUTPUT_VOLTAGE / config.PEAK_VOLTAGE, config.CONFIG_TIMEOUT_MS);
    talon.configPeakOutputReverse(
        -(config.MAX_OUTPUT_VOLTAGE / config.PEAK_VOLTAGE), config.CONFIG_TIMEOUT_MS);

    talon.configNominalOutputForward(config.NOMINAL_VOLTAGE, config.CONFIG_TIMEOUT_MS);
    talon.configNominalOutputReverse(config.NOMINAL_VOLTAGE, config.CONFIG_TIMEOUT_MS);

    talon.changeMotionControlFramePeriod(config.MOTION_CONTROL_FRAME_PERIOD_MS);
    talon.clearMotionProfileHasUnderrun(config.CONFIG_TIMEOUT_MS);
    talon.clearMotionProfileTrajectories();
    talon.clearStickyFaults(config.CONFIG_TIMEOUT_MS);
    talon.configVelocityMeasurementPeriod(
        config.VELOCITY_MEASUREMENT_PERIOD, config.CONFIG_TIMEOUT_MS);
    talon.setNeutralMode(config.ENABLE_BRAKE ? NeutralMode.Brake : NeutralMode.Coast);
    talon.enableCurrentLimit(config.ENABLE_CURRENT_LIMIT);
    talon.configForwardSoftLimitEnable(config.ENABLE_SOFT_LIMIT, config.CONFIG_TIMEOUT_MS);
    talon.configReverseSoftLimitEnable(config.ENABLE_SOFT_LIMIT, config.CONFIG_TIMEOUT_MS);
    talon.configForwardLimitSwitchSource(
        LimitSwitchSource.Deactivated, LimitSwitchNormal.Disabled, config.CONFIG_TIMEOUT_MS);
    talon.configReverseLimitSwitchSource(
        LimitSwitchSource.Deactivated, LimitSwitchNormal.Disabled, config.CONFIG_TIMEOUT_MS);

    talon.setInverted(config.INVERTED);

    talon.configContinuousCurrentLimit(config.CURRENT_LIMIT, config.CONFIG_TIMEOUT_MS);

    talon.configForwardSoftLimitThreshold(config.FORWARD_SOFT_LIMIT, config.CONFIG_TIMEOUT_MS);

    talon.setSelectedSensorPosition(0, 0, config.CONFIG_TIMEOUT_MS);
    talon.selectProfileSlot(0, 0);

    talon.configReverseSoftLimitThreshold(config.REVERSE_SOFT_LIMIT, config.CONFIG_TIMEOUT_MS);

    talon.configVelocityMeasurementPeriod(
        config.VELOCITY_MEASUREMENT_PERIOD, config.CONFIG_TIMEOUT_MS);
    talon.configVelocityMeasurementWindow(
        config.VELOCITY_MEASUREMENT_ROLLING_AVERAGE_WINDOW, config.CONFIG_TIMEOUT_MS);

    talon.configOpenloopRamp(config.VOLTAGE_RAMP_RATE, config.CONFIG_TIMEOUT_MS);

    talon.setStatusFramePeriod(
        StatusFrameEnhanced.Status_1_General,
        config.GENERAL_STATUS_FRAME_RATE_MS,
        config.CONFIG_TIMEOUT_MS);
    talon.setStatusFramePeriod(
        StatusFrameEnhanced.Status_2_Feedback0,
        config.FEEDBACK_STATUS_FRAME_RATE_MS,
        config.CONFIG_TIMEOUT_MS);
    talon.setStatusFramePeriod(
        StatusFrameEnhanced.Status_3_Quadrature,
        config.QUAD_ENCODER_STATUS_FRAME_RATE_MS,
        config.CONFIG_TIMEOUT_MS);
    talon.setStatusFramePeriod(
        StatusFrameEnhanced.Status_4_AinTempVbat,
        config.ANALOG_TEMP_VBAT_STATUS_FRAME_RATE_MS,
        config.CONFIG_TIMEOUT_MS);
    talon.setStatusFramePeriod(
        StatusFrameEnhanced.Status_8_PulseWidth,
        config.PULSE_WIDTH_STATUS_FRAME_RATE_MS,
        config.CONFIG_TIMEOUT_MS);

    return talon;
  }
}
