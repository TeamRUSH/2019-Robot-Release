package net.teamrush27.frc2019.subsystems.impl;

import com.ctre.phoenix.CANifier;
import com.ctre.phoenix.CANifier.PWMChannel;
import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.Faults;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.RemoteSensorSource;
import com.ctre.phoenix.motorcontrol.StickyFaults;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import net.teamrush27.frc2019.Robot;
import net.teamrush27.frc2019.base.RobotMap;
import net.teamrush27.frc2019.constants.RobotConstants;
import net.teamrush27.frc2019.loops.ILooper;
import net.teamrush27.frc2019.loops.Loop;
import net.teamrush27.frc2019.subsystems.Subsystem;
import net.teamrush27.frc2019.subsystems.impl.dto.SmartDashboardCollection;
import org.apache.logging.log4j.LogManager;
import org.apache.logging.log4j.Logger;

public class Wrist extends Subsystem {

  private static Logger LOG = LogManager.getLogger(Wrist.class);
  private static String TAG = "WRIST";
  private static Wrist INSTANCE = null;

  private static final double TICKS_PER_DEGREE = 4096d / 360d;
  private static final double DEGREES_PER_TICK = 360d / 4096d;

  private static final int PWM_RANGE = 1024;

  public static Wrist getInstance() {
    if (INSTANCE == null) {
      INSTANCE = new Wrist();
    }
    return INSTANCE;
  }

  public enum WantedState {
    OFF,
    OPEN_LOOP,
    CLOSED_LOOP
  }

  private enum SystemState {
    OFF,
    OPEN_LOOP,
    CLOSED_LOOP,
    HOLD
  }

  private WantedState wantedState = WantedState.OFF;
  private SystemState systemState = SystemState.OFF;
  private WristState wristState;

  private boolean zeroed = false;
  private boolean stateChanged = false;
  private double currentStateStartTime;

  private Loop loop =
      new Loop() {

        @Override
        public void onStart(double timestamp) {
          currentStateStartTime = timestamp;
        }

        @Override
        public void onLoop(double timestamp) {
          SystemState newState;
          switch (systemState) {
            case OPEN_LOOP:
              newState = handleOpenLoop(timestamp);
              break;
            case CLOSED_LOOP:
              newState = handleClosedLoop(timestamp);
              break;
            case HOLD:
              newState = handleHold(timestamp);
              break;
            case OFF:
            default:
              newState = handleOff(timestamp);
              break;
          }
          if (newState != systemState) {
            LOG.info("Wrist state {} to {}", systemState, newState);
            systemState = newState;
            currentStateStartTime = timestamp;
            stateChanged = true;
          } else {
            stateChanged = false;
          }
        }

        @Override
        public void onStop(double timestamp) {
          stop();
        }

        @Override
        public String id() {
          return TAG;
        }
      };

  private final TalonSRX wristMotor;
  private final CANifier wristSensor;

  public Wrist() {
    wristMotor = new TalonSRX(RobotMap.WRIST_MOTOR_CAN_ID);
    wristMotor.configFactoryDefault(RobotConstants.TALON_CONFIG_TIMEOUT);
    wristMotor.configVoltageCompSaturation(8, RobotConstants.TALON_CONFIG_TIMEOUT);
    wristMotor.enableVoltageCompensation(false);
    wristMotor.setInverted(true);
    wristMotor.setNeutralMode(NeutralMode.Brake);

    wristMotor.configRemoteFeedbackFilter(
        RobotMap.WRIST_CANIFIER_CAN_ID,
        RemoteSensorSource.CANifier_Quadrature,
        0,
        RobotConstants.TALON_CONFIG_TIMEOUT);
    wristMotor.configSelectedFeedbackSensor(
        FeedbackDevice.RemoteSensor0, 0, RobotConstants.TALON_CONFIG_TIMEOUT);
    wristMotor.setSensorPhase(false);

    wristSensor = new CANifier(RobotMap.WRIST_CANIFIER_CAN_ID);
    wristSensor.configFactoryDefault(RobotConstants.TALON_CONFIG_TIMEOUT);

    reloadGains();

    wristState = new WristState();
  }

  private void reloadGains() {
    wristMotor.config_kP(0, 5, RobotConstants.TALON_CONFIG_TIMEOUT);
    wristMotor.config_kI(0, 0, RobotConstants.TALON_CONFIG_TIMEOUT);
    wristMotor.config_kD(0, .1, RobotConstants.TALON_CONFIG_TIMEOUT);
    wristMotor.config_kF(0, 0, RobotConstants.TALON_CONFIG_TIMEOUT);
    wristMotor.configMotionAcceleration(
        degreesToTicks(360) / 10, RobotConstants.TALON_CONFIG_TIMEOUT);
    wristMotor.configMotionCruiseVelocity(
        degreesToTicks(180) / 10, RobotConstants.TALON_CONFIG_TIMEOUT);
  }

  private SystemState handleOff(double timestamp) {
    return defaultStateTransfer(timestamp);
  }

  private SystemState handleClosedLoop(double timestamp) {
    if (stateChanged) {
      wristMotor.configVoltageCompSaturation(6);
    }

    if (WantedState.CLOSED_LOOP.equals(wantedState)
        && Math.abs(getEncoderAngle() - wristState.demand) < 3) {
      return SystemState.HOLD;
    } else {
      return defaultStateTransfer(timestamp);
    }
  }

  private int degreesToTicks(double degrees) {
    return Double.valueOf(degrees * TICKS_PER_DEGREE).intValue();
  }

  private SystemState handleHold(double timestamp) {
    if (stateChanged) {
      wristMotor.configVoltageCompSaturation(4);
    }

    if (WantedState.CLOSED_LOOP.equals(wantedState)
        && Math.abs(getEncoderAngle() - wristState.demand) < 2) {
      return SystemState.HOLD;
    } else {
      return defaultStateTransfer(timestamp);
    }
  }

  private SystemState handleOpenLoop(double timestamp) {
    return defaultStateTransfer(timestamp);
  }

  private SystemState defaultStateTransfer(double timestamp) {
    switch (wantedState) {
      case OPEN_LOOP:
        return SystemState.OPEN_LOOP;
      case CLOSED_LOOP:
        return SystemState.CLOSED_LOOP;
      default:
      case OFF:
        return SystemState.OFF;
    }
  }

  @Override
  public void registerEnabledLoops(ILooper enabledLooper) {
    enabledLooper.register(loop);
  }

  @Override
  public void readPeriodicInputs() {
    Faults talonFaults = new Faults();

    wristMotor.getFaults(talonFaults);

    if (talonFaults.hasAnyFault()) {
      System.out.println(talonFaults.toString());
    }

    if (talonFaults.RemoteLossOfSignal) {
      LOG.info("LOST CANIFIER SIGNAL");
    }

    StickyFaults stickyFaults = new StickyFaults();
    wristMotor.getStickyFaults(stickyFaults);
    if (stickyFaults.hasAnyFault()) {
      System.out.println(stickyFaults.toString());
      wristMotor.clearStickyFaults();
    }

    if (wristMotor.hasResetOccurred()) {
      System.out.println("WRIST MOTOR RESET HAS OCCURED");
    }

    if (wristSensor.hasResetOccurred()) {
      System.out.println("CANIFIER RESET HAS OCCURED");
    }

    readInputs();

    /*
    if (talonFaults.RemoteLossOfSignal) {
      zeroed = false;
      zeroSensors();
    } else {
      readInputs();
    }
    */
  }

  private void readInputs() {
    double[] array = new double[2];

    wristSensor.getPWMInput(PWMChannel.PWMChannel0, array);
    wristState.wristPWMValue = array[0];
    wristState.wristPWMDegrees =
        (-wristState.wristPWMValue + Robot.ROBOT_CONFIGURATION.getWristHomePosition())
            * DEGREES_PER_TICK;

    wristState.wristEncoderTicks = wristMotor.getSelectedSensorPosition(0);
    wristState.wristEncoderDegrees = wristState.wristEncoderTicks * DEGREES_PER_TICK;
  }

  @Override
  public void writePeriodicOutputs() {
    switch (systemState) {
      case OPEN_LOOP:
        wristMotor.set(ControlMode.PercentOutput, wristState.demand);
        break;
      case CLOSED_LOOP:
        wristMotor.set(ControlMode.MotionMagic, degreesToTicks(wristState.demand));
        break;
      case HOLD:
        break;
      case OFF:
      default:
        wristMotor.set(ControlMode.Disabled, 0);
    }
  }

  @Override
  public void outputToSmartDashboard(SmartDashboardCollection collection) {
    //		double[] array = new double[2];
    //		wristSensor.getPWMInput(PWMChannel.PWMChannel0, array);
    //		LOG.info("{} {}", getEncoderAngle(), array[0]);

    // collection.setWristEncoderTicks(wristState.wristEncoderTicks);
    // collection.setWristEncoderDegrees(wristState.wristEncoderDegrees);

    // collection.setWristPWMValue(wristState.wristPWMValue);
    // collection.setWristPWMDegrees(wristState.wristPWMDegrees);

    SmartDashboard.putNumber("wrist.encoder_ticks", wristState.wristEncoderTicks);
    SmartDashboard.putNumber("wrist.encoder_degrees", wristState.wristEncoderDegrees);

    SmartDashboard.putNumber("wrist.pwm_value", wristState.wristPWMValue);
    SmartDashboard.putNumber("wrist.pwm_degrees", wristState.wristPWMDegrees);
  }

  @Override
  public void stop() {
    wantedState = WantedState.OFF;
  }

  @Override
  public void zeroSensors() {
    if (!zeroed) {
      readPeriodicInputs();

      int blah = -Double.valueOf(getPWMAngle() * TICKS_PER_DEGREE).intValue();

      wristSensor.setQuadraturePosition(blah, 0);

      readInputs();

      LOG.info("INITIAL POSITION {} {} {}", getPWMAngle(), getEncoderAngle(), blah);
      zeroed = true;
    }
  }

  public double getEncoderAngle() {
    return wristMotor.getSelectedSensorPosition() * DEGREES_PER_TICK;
  }

  public double getEncoderTicks() {
    return wristState.wristEncoderTicks;
  }

  public double getPWMAngle() {
    return wristState.wristPWMDegrees;
  }

  public double getPWMValue() {
    return wristState.wristPWMValue;
  }

  @Override
  public void test() {}

  public void setOpenLoopInput(double openLoopInput) {
    synchronized (wristState) {
      wristState.demand = openLoopInput;
    }
  }

  public void setClosedLoopInput(double closedLoopInput) {
    synchronized (wristState) {
      closedLoopInput = Math.max(closedLoopInput, -90);
      closedLoopInput = Math.min(closedLoopInput, 90);
      wristState.demand = closedLoopInput;
    }
  }

  public void setWantedState(WantedState wantedState) {
    synchronized (this) {
      this.wantedState = wantedState;
    }
  }

  @Override
  public String id() {
    return TAG;
  }

  private class WristState {

    int wristEncoderTicks;
    double wristEncoderDegrees;
    double wristPWMValue;
    double wristPWMDegrees;

    Double demand = 0d;
  }
}
