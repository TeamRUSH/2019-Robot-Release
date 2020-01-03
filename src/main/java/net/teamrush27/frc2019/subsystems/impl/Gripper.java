package net.teamrush27.frc2019.subsystems.impl;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import net.teamrush27.frc2019.Robot;
import net.teamrush27.frc2019.base.RobotMap;
import net.teamrush27.frc2019.constants.RobotConstants;
import net.teamrush27.frc2019.loops.ILooper;
import net.teamrush27.frc2019.loops.Loop;
import net.teamrush27.frc2019.subsystems.Subsystem;
import net.teamrush27.frc2019.subsystems.impl.dto.SmartDashboardCollection;
import net.teamrush27.frc2019.util.math.CircularBuffer;
import net.teamrush27.frc2019.wrappers.InvertableDigitalInput;
import org.apache.logging.log4j.LogManager;
import org.apache.logging.log4j.Logger;

public class Gripper extends Subsystem {

  private static final String TAG = "GRIPPER";
  private static final Logger LOG = LogManager.getLogger(Gripper.class);
  private static Gripper INSTANCE = null;

  public static Gripper getInstance() {
    if (INSTANCE == null) {
      INSTANCE = new Gripper();
    }
    return INSTANCE;
  }

  public enum WantedState {
    OFF,
    INTAKE_CARGO,
    EXHAUST_CARGO,
    INTAKE_HATCH,
    EXHAUST_HATCH
  }

  private enum SystemState {
    OFF,
    INTAKE_CARGO,
    HOLD_CARGO,
    EXHAUST_CARGO,
    HOLD_HATCH,
    EXHAUST_HATCH,
    UNJAM_HATCH
  }

  private WantedState wantedState = WantedState.INTAKE_HATCH;
  private SystemState systemState = SystemState.HOLD_HATCH;

  private boolean hasGamepiece = false;
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
            case INTAKE_CARGO:
              newState = handleIntakeCargo(timestamp);
              break;
            case HOLD_CARGO:
              newState = handleHoldCargo(timestamp);
              break;
            case EXHAUST_CARGO:
              newState = handleExhaustCargo(timestamp);
              break;
            case EXHAUST_HATCH:
              newState = handleExhaustHatch(timestamp);
              break;
            case HOLD_HATCH:
              newState = handleHoldHatch(timestamp);
              break;
            case UNJAM_HATCH:
              newState = handleUnjamHatch(timestamp);
              break;
            case OFF:
            default:
              newState = handleOff(timestamp);
              break;
          }
          if (newState != systemState) {
            LOG.info("Gripper state {} to {}", systemState, newState);
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

  public synchronized void unjam() {
    currentStateStartTime = Timer.getFPGATimestamp();
    systemState = SystemState.UNJAM_HATCH;
  }

  private final TalonSRX gripperMotor;
  private final TalonSRX jawMotor;

  private final DigitalInput jawHome;
  private final DigitalInput jawMax;

  private final CircularBuffer circularBuffer = new CircularBuffer(10);

  public Gripper() {
    gripperMotor = new TalonSRX(RobotMap.GRIPPER_MOTOR_CAN_ID);
    gripperMotor.configFactoryDefault(RobotConstants.TALON_CONFIG_TIMEOUT);
    gripperMotor.setNeutralMode(NeutralMode.Brake);
    gripperMotor.configContinuousCurrentLimit(20);
    gripperMotor.configVoltageCompSaturation(12);
    gripperMotor.enableVoltageCompensation(true);
    gripperMotor.setInverted(true);

    jawMotor = new TalonSRX(RobotMap.GRIPPER_JAWS_CAN_ID);
    jawMotor.configFactoryDefault(RobotConstants.TALON_CONFIG_TIMEOUT);
    jawMotor.enableCurrentLimit(true);
    jawMotor.setInverted(true);
    jawMotor.setSensorPhase(false);
    jawMotor.configContinuousCurrentLimit(20);
    jawMotor.configVoltageCompSaturation(12);
    jawMotor.enableVoltageCompensation(true);
    jawMotor.configSelectedFeedbackSensor(FeedbackDevice.Analog);
    jawMotor.config_kP(0, 100);
    jawMotor.config_kI(0, 0);
    jawMotor.config_kD(0, 1000);

    jawHome = new InvertableDigitalInput(RobotMap.GRIPPER_JAW_HOME_SENSOR_ID, true);
    jawMax = new InvertableDigitalInput(RobotMap.GRIPPER_JAW_MAX_SENSOR_ID, true);
  }

  boolean goHome = false;

  private SystemState handleOff(double timestamp) {
    gripperMotor.set(ControlMode.PercentOutput, 0);

    if (stateChanged) {
      goHome = true;
    }

    if (goHome) {
      jawMotor.set(ControlMode.Position, Robot.ROBOT_CONFIGURATION.getJawRetractPosition());

      if (jawMotor.getSelectedSensorVelocity() == 0 && timestamp - currentStateStartTime > 0.5) {
        goHome = false;
        jawMotor.set(ControlMode.Disabled, 0);
      }
    }

    return defaultStateTransfer(timestamp);
  }

  private SystemState handleHoldHatch(double timestamp) {

    jawMotor.set(ControlMode.Position, Robot.ROBOT_CONFIGURATION.getJawIntakePosition());
    gripperMotor.set(ControlMode.PercentOutput, 0);

    if (WantedState.INTAKE_HATCH.equals(wantedState)) {
      return SystemState.HOLD_HATCH;
    }

    return defaultStateTransfer(timestamp);
  }

  private SystemState handleExhaustHatch(double timestamp) {
    jawMotor.set(ControlMode.Position, Robot.ROBOT_CONFIGURATION.getJawExhaustPosition());
    gripperMotor.set(ControlMode.PercentOutput, 0);

    return defaultStateTransfer(timestamp);
  }

  double startExhaust = 0;

  private SystemState handleExhaustCargo(double timestamp) {
    if (stateChanged) {
      gripperMotor.configVoltageCompSaturation(12);
      goHome = true;
    }
    if (startExhaust == 0) {
      startExhaust = timestamp;
    }
    firstFoundBall = 0;
    gripperMotor.set(ControlMode.PercentOutput, -1);

    if (goHome) {
      jawMotor.set(ControlMode.Position, Robot.ROBOT_CONFIGURATION.getJawRetractPosition());

      if (jawMotor.getSelectedSensorVelocity() == 0 && timestamp - currentStateStartTime > 0.5) {
        goHome = false;
        jawMotor.set(ControlMode.Disabled, 0);
      }
    }

    if (WantedState.EXHAUST_CARGO.equals(wantedState) && timestamp - startExhaust > .4) {
      startExhaust = 0;
      wantedState = WantedState.OFF;
      return SystemState.OFF;
    }

    return defaultStateTransfer(timestamp);
  }

  private SystemState handleHoldCargo(double timestamp) {
    firstFoundBall = 0;
    if (stateChanged) {
      gripperMotor.configVoltageCompSaturation(4);
      goHome = true;
    }

    gripperMotor.set(ControlMode.PercentOutput, 1);

    if (goHome) {
      jawMotor.set(ControlMode.Position, Robot.ROBOT_CONFIGURATION.getJawRetractPosition());

      if (jawMotor.getSelectedSensorVelocity() == 0 && timestamp - currentStateStartTime > 0.5) {
        goHome = false;
        jawMotor.set(ControlMode.Disabled, 0);
      }
    }

    if (WantedState.EXHAUST_CARGO.equals(wantedState)) {
      return SystemState.EXHAUST_CARGO;
    }

    if (WantedState.INTAKE_CARGO.equals(wantedState)) {
      return SystemState.HOLD_CARGO;
    }

    return defaultStateTransfer(timestamp);
  }

  private double firstFoundBall = 0;

  private SystemState handleIntakeCargo(double timestamp) {
    if (stateChanged) {
      circularBuffer.clear();
      gripperMotor.configVoltageCompSaturation(12);
    }

    gripperMotor.set(ControlMode.PercentOutput, 1);
    jawMotor.set(ControlMode.Position, Robot.ROBOT_CONFIGURATION.getJawRetractPosition());

    if (WantedState.INTAKE_CARGO.equals(wantedState)
        && circularBuffer.getAverage() > 30
        && circularBuffer.isFull()
        && timestamp - currentStateStartTime > .1) {
      if (firstFoundBall == 0) {
        firstFoundBall = Timer.getFPGATimestamp();
      }
      if (Timer.getFPGATimestamp() - firstFoundBall > .25) {
        return SystemState.HOLD_CARGO;
      }
    }

    return defaultStateTransfer(timestamp);
  }

  private SystemState handleUnjamHatch(double timestamp) {
    jawMotor.set(ControlMode.Position, Robot.ROBOT_CONFIGURATION.getJawExhaustPosition());

    if (timestamp - currentStateStartTime < 0.2) {
      return SystemState.UNJAM_HATCH;
    }

    return defaultStateTransfer(timestamp);
  }

  private SystemState defaultStateTransfer(double timestamp) {
    switch (wantedState) {
      case INTAKE_CARGO:
        return SystemState.INTAKE_CARGO;
      case EXHAUST_CARGO:
        return SystemState.EXHAUST_CARGO;
      case INTAKE_HATCH:
        return SystemState.HOLD_HATCH;
      case EXHAUST_HATCH:
        return SystemState.EXHAUST_HATCH;
      default:
      case OFF:
        return SystemState.OFF;
    }
  }

  @Override
  public void readPeriodicInputs() {
    circularBuffer.addValue(Math.abs(gripperMotor.getOutputCurrent()));
  }

  @Override
  public void registerEnabledLoops(ILooper enabledLooper) {
    enabledLooper.register(loop);
  }

  @Override
  public void outputToSmartDashboard(SmartDashboardCollection collection) {
    if (SystemState.HOLD_CARGO.equals(systemState)) {
      LED.getInstance().setHasGamePiece(true);
    } else if (SystemState.HOLD_HATCH.equals(systemState)) {
      LED.getInstance().setHasGamePiece(true);
    } else {
      hasGamepiece = false;
      LED.getInstance().setHasGamePiece(false);
    }
    if (SystemState.INTAKE_CARGO.equals(systemState)) {
      LED.getInstance().setIntaking(true);
    } else {
      LED.getInstance().setIntaking(false);
    }
    if (SystemState.EXHAUST_CARGO.equals(systemState)
        || SystemState.EXHAUST_HATCH.equals(systemState)) {
      LED.getInstance().setExhausting(true);
    } else {
      LED.getInstance().setExhausting(false);
    }

    SmartDashboard.putString("gripper.state", systemState.toString());
    SmartDashboard.putNumber("gripper.jaw_position", jawMotor.getSelectedSensorPosition());
  }

  @Override
  public void stop() {}

  @Override
  public void zeroSensors() {
    int position = (jawMotor.getSelectedSensorPosition() % 1024 + 1024) % 1024;
    jawMotor.setSelectedSensorPosition(position);
  }

  @Override
  public void test() {}

  @Override
  public void writePeriodicOutputs() {}

  public void setWantedState(WantedState wantedState) {
    synchronized (wantedState) {
      this.wantedState = wantedState;
    }
  }

  public void transitionCargo() {
    synchronized (wantedState) {
      switch (wantedState) {
        case INTAKE_CARGO:
          wantedState = WantedState.EXHAUST_CARGO;
          break;
        case EXHAUST_CARGO:
          wantedState = WantedState.OFF;
          break;
        default:
          if (!SystemState.HOLD_HATCH.equals(systemState)) {
            wantedState = WantedState.INTAKE_CARGO;
          }
      }
    }
  }

  public void transitionHatch() {
    synchronized (systemState) {
      switch (systemState) {
        default:
        case OFF:
          wantedState = WantedState.INTAKE_HATCH;
          break;
        case EXHAUST_HATCH:
          wantedState = WantedState.OFF;
          break;
        case HOLD_HATCH:
          wantedState = WantedState.EXHAUST_HATCH;
          break;
        case HOLD_CARGO:
          break;
      }
    }
  }

  public boolean hasGamepiece() {
    return SystemState.HOLD_CARGO.equals(systemState) || SystemState.HOLD_HATCH.equals(systemState);
  }

  public boolean hasHatch() {
    return SystemState.HOLD_HATCH.equals(systemState);
  }

  public boolean hasCargo() {
    return SystemState.HOLD_CARGO.equals(systemState);
  }

  @Override
  public String id() {
    return TAG;
  }
}
