package net.teamrush27.frc2019.subsystems.impl;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import net.teamrush27.frc2019.base.RobotMap;
import net.teamrush27.frc2019.constants.RobotConstants;
import net.teamrush27.frc2019.loops.ILooper;
import net.teamrush27.frc2019.loops.Loop;
import net.teamrush27.frc2019.subsystems.Subsystem;
import net.teamrush27.frc2019.subsystems.impl.dto.SmartDashboardCollection;
import net.teamrush27.frc2019.wrappers.InvertableDigitalInput;
import org.apache.logging.log4j.LogManager;
import org.apache.logging.log4j.Logger;

public class SpiderLegs extends Subsystem {

  private static Logger LOG = LogManager.getLogger(SpiderLegs.class);
  private static String TAG = "SPIDERLEGS";
  private static SpiderLegs INSTANCE = null;

  private static final int REAR_HOME = 0;
  private static final int FRONT_HOME = 0;

  private static final int REAR_PRE = 7500;
  private static final int FRONT_PRE = 3500;

  private static final int FRONT_HOLD = 11750;
  private static final int FRONT_L2_HOLD = 12300;

  private static final int REAR_HOLD_L3 = 12750;
  private static final int REAR_HOLD_L2 = 10000;

  private static final int REAR_RETURN = 1000;

  private static final int EPSILON = 250;

  public static SpiderLegs getInstance() {
    if (INSTANCE == null) {
      INSTANCE = new SpiderLegs();
    }
    return INSTANCE;
  }

  public enum WantedState {
    OFF,
    PENDING_CLIMB,
    CLIMB,
    CLIMB_L2,
    OPEN_LOOP
  }

  private enum SystemState {
    OFF,
    PENDING_CLIMB,
    CLIMBING,
    CLIMBING_HOLD,
    CLIMBING_L2,
    CLIMBING_L2_HOLD,
    OPEN_LOOP
  }

  private WantedState wantedState = WantedState.OFF;
  private SystemState systemState = SystemState.OFF;

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
            case PENDING_CLIMB:
              newState = handlePendingClimb(timestamp);
              break;
            case CLIMBING:
              newState = handleClimb(timestamp);
              break;
            case CLIMBING_HOLD:
              newState = handleClimbHold(timestamp);
              break;
            case CLIMBING_L2:
              newState = handleClimbL2(timestamp);
              break;
            case CLIMBING_L2_HOLD:
              newState = handleClimbL2Hold(timestamp);
              break;
            case OFF:
            default:
              newState = handleOff(timestamp);
              break;
          }
          if (newState != systemState) {
            LOG.info("SpiderLegs state {} to {}", systemState, newState);
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

  private final TalonSRX frontLegMotor;

  private final TalonSRX rearLegMotorMaster;
  private final TalonSRX rearLegMotorSlave;

  private final DigitalInput frontLegHome;
  private final DigitalInput rearLegHome;

  private final AnalogInput underFrontWheel;
  private final AnalogInput underRearMiddleWheel;

  private Boolean frontHomed = false;
  private Boolean rearHomed = false;
  private Boolean frontOnGround = true;
  private Boolean rearOnGround = true;

  public SpiderLegs() {
    frontLegMotor = new TalonSRX(RobotMap.FRONT_SPIDER_LEG_MOTOR_CAN_ID);
    frontLegMotor.configFactoryDefault(RobotConstants.TALON_CONFIG_TIMEOUT);
    frontLegMotor.configSelectedFeedbackSensor(
        FeedbackDevice.CTRE_MagEncoder_Relative, 0, RobotConstants.TALON_CONFIG_TIMEOUT);
    frontLegMotor.setInverted(false);
    frontLegMotor.setNeutralMode(NeutralMode.Coast);
    frontLegMotor.configContinuousCurrentLimit(40, RobotConstants.TALON_CONFIG_TIMEOUT);
    frontLegMotor.enableCurrentLimit(true);
    frontLegMotor.setSensorPhase(false);

    rearLegMotorMaster = new TalonSRX(RobotMap.REAR_SPIDER_LEG_MOTOR_MASTER_CAN_ID);
    rearLegMotorMaster.configFactoryDefault(RobotConstants.TALON_CONFIG_TIMEOUT);
    rearLegMotorMaster.configSelectedFeedbackSensor(
        FeedbackDevice.CTRE_MagEncoder_Relative, 0, RobotConstants.TALON_CONFIG_TIMEOUT);
    rearLegMotorMaster.setInverted(true);
    rearLegMotorMaster.setNeutralMode(NeutralMode.Coast);
    rearLegMotorMaster.configContinuousCurrentLimit(40, RobotConstants.TALON_CONFIG_TIMEOUT);
    rearLegMotorMaster.enableCurrentLimit(true);

    rearLegMotorSlave = new TalonSRX(RobotMap.REAR_SPIDER_LEG_MOTOR_SLAVE_CAN_ID);
    rearLegMotorSlave.configFactoryDefault();
    rearLegMotorSlave.setInverted(false);
    rearLegMotorSlave.follow(rearLegMotorMaster);
    rearLegMotorSlave.setNeutralMode(NeutralMode.Coast);
    rearLegMotorSlave.configContinuousCurrentLimit(40, RobotConstants.TALON_CONFIG_TIMEOUT);
    rearLegMotorSlave.enableCurrentLimit(true);

    frontLegHome = new InvertableDigitalInput(RobotMap.FRONT_SPIDER_LEG_HOME_SENSOR_ID, true);
    rearLegHome = new InvertableDigitalInput(RobotMap.REAR_SPIDER_LEG_HOME_SENSOR_ID, true);

    underFrontWheel = new AnalogInput(RobotMap.SPIDER_LEG_FRONT_FLOOR_SENSOR_ID);
    underRearMiddleWheel = new AnalogInput(RobotMap.SPIDER_LEG_REAR_FLOOR_SENSOR_ID);

    reloadGains();
    setVoltageCompensation(false);
  }

  private void reloadGains() {
    frontLegMotor.config_kP(0, 1, RobotConstants.TALON_CONFIG_TIMEOUT);
    frontLegMotor.config_kI(0, 0, RobotConstants.TALON_CONFIG_TIMEOUT);
    frontLegMotor.config_kD(0, 0, RobotConstants.TALON_CONFIG_TIMEOUT);
    frontLegMotor.config_kF(0, 0, RobotConstants.TALON_CONFIG_TIMEOUT);
    frontLegMotor.configMotionCruiseVelocity(400, RobotConstants.TALON_CONFIG_TIMEOUT);
    frontLegMotor.configMotionAcceleration(4000, RobotConstants.TALON_CONFIG_TIMEOUT);

    rearLegMotorMaster.config_kP(0, 1, RobotConstants.TALON_CONFIG_TIMEOUT);
    rearLegMotorMaster.config_kI(0, 0, RobotConstants.TALON_CONFIG_TIMEOUT);
    rearLegMotorMaster.config_kD(0, 0, RobotConstants.TALON_CONFIG_TIMEOUT);
    rearLegMotorMaster.config_kF(0, 0, RobotConstants.TALON_CONFIG_TIMEOUT);
    rearLegMotorMaster.configMotionCruiseVelocity(400, RobotConstants.TALON_CONFIG_TIMEOUT);
    rearLegMotorMaster.configMotionAcceleration(4000, RobotConstants.TALON_CONFIG_TIMEOUT);
  }

  private SystemState handleOff(double timestamp) {
    rearLegMotorMaster.set(ControlMode.Disabled, REAR_HOME);
    frontLegMotor.set(ControlMode.Disabled, FRONT_HOME);

    return defaultStateTransfer(timestamp);
  }

  private SystemState handlePendingClimb(double timestamp) {
    frontLegMotor.set(ControlMode.MotionMagic, FRONT_PRE);
    rearLegMotorMaster.set(ControlMode.MotionMagic, REAR_PRE);
    //		TelemetryUtil.getInstance().addEntry(Timer.getFPGATimestamp(), "spiderlegs.rear.position",
    // String.valueOf(rearLegMotorMaster.getSelectedSensorPosition()));
    //		TelemetryUtil.getInstance().addEntry(Timer.getFPGATimestamp(), "spiderlegs.rear.velocity",
    // String.valueOf(rearLegMotorMaster.getSelectedSensorVelocity()));

    return defaultStateTransfer(timestamp);
  }

  private SystemState handleClimbL2(double timestamp) {
    frontLegMotor.set(ControlMode.MotionMagic, FRONT_L2_HOLD);

    if (frontLegMotor.getSelectedSensorPosition() > 6000) {
      rearLegMotorMaster.set(ControlMode.MotionMagic, REAR_HOLD_L2);
    }

    if (Math.abs(frontLegMotor.getSelectedSensorPosition() - FRONT_L2_HOLD) < EPSILON) {
      return SystemState.CLIMBING_L2_HOLD;
    }

    return defaultStateTransfer(timestamp);
  }

  private SystemState handleClimb(double timestamp) {
    frontLegMotor.set(ControlMode.MotionMagic, FRONT_HOLD);
    rearLegMotorMaster.set(ControlMode.MotionMagic, REAR_HOLD_L3);

    if (Math.abs(frontLegMotor.getSelectedSensorPosition() - FRONT_HOLD) < EPSILON) {
      return SystemState.CLIMBING_HOLD;
    }

    return defaultStateTransfer(timestamp);
  }

  double firstHeld = 0;

  private SystemState handleClimbL2Hold(double timestamp) {
    if (stateChanged) {
      setVoltageCompensation(true);
      frontOnGround = false;
      rearOnGround = false;
      firstHeld = Timer.getFPGATimestamp();
    }

    if (frontOnGround && Timer.getFPGATimestamp() - firstHeld > 1) {
      frontLegMotor.set(ControlMode.Disabled, FRONT_HOME);
    } else {
      frontLegMotor.set(ControlMode.MotionMagic, FRONT_L2_HOLD);
    }
    if (rearOnGround && Timer.getFPGATimestamp() - firstHeld > 1) {
      rearLegMotorMaster.set(ControlMode.MotionMagic, REAR_RETURN);
    } else {
      rearLegMotorMaster.set(ControlMode.MotionMagic, REAR_HOLD_L2);
    }

    return defaultStateTransfer(timestamp);
  }

  private SystemState handleClimbHold(double timestamp) {
    if (stateChanged) {
      setVoltageCompensation(true);
      frontOnGround = false;
      rearOnGround = false;
    }

    LOG.info(
        "FrontOnGround: {} - {}\tRearOnGround: {} - {}",
        frontOnGround,
        underFrontWheel.getAverageVoltage(),
        rearOnGround,
        underRearMiddleWheel.getAverageVoltage());

    if (frontOnGround) {
      frontLegMotor.set(ControlMode.Disabled, FRONT_HOME);
    } else {
      frontLegMotor.set(ControlMode.MotionMagic, FRONT_HOLD);
    }
    if (frontOnGround && rearOnGround) {
      rearLegMotorMaster.set(ControlMode.MotionMagic, REAR_RETURN);
    } else {
      rearLegMotorMaster.set(ControlMode.MotionMagic, REAR_HOLD_L3);
    }

    return defaultStateTransfer(timestamp);
  }

  private void setVoltageCompensation(boolean enabled) {
    if (enabled) {
      frontLegMotor.configVoltageCompSaturation(4);
      frontLegMotor.enableVoltageCompensation(true);
    } else {
      frontLegMotor.enableVoltageCompensation(false);
    }
  }

  private SystemState defaultStateTransfer(double timestamp) {
    switch (wantedState) {
      case PENDING_CLIMB:
        return SystemState.PENDING_CLIMB;
      case CLIMB:
        if (SystemState.CLIMBING_HOLD.equals(systemState)) {
          return SystemState.CLIMBING_HOLD;
        }
        return SystemState.CLIMBING;
      case CLIMB_L2:
        if (SystemState.CLIMBING_L2_HOLD.equals(systemState)) {
          return SystemState.CLIMBING_L2_HOLD;
        }
        return SystemState.CLIMBING_L2;
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
    if (WantedState.CLIMB_L2.equals(wantedState)) {
      if (underFrontWheel.getAverageVoltage() > 1.25 || frontOnGround) {
        frontOnGround = true;
      } else {
        frontOnGround = false;
      }

      if (underRearMiddleWheel.getAverageVoltage() > 1.8 || rearOnGround) {
        rearOnGround = true;
        frontOnGround = true;
      } else {
        rearOnGround = false;
      }
    } else {
      if (underFrontWheel.getAverageVoltage() > 1.25 || frontOnGround) {
        frontOnGround = true;
      } else {
        frontOnGround = false;
      }

      if (underRearMiddleWheel.getAverageVoltage() > 2.0 || rearOnGround) {
        rearOnGround = true;
        frontOnGround = true;
      } else {
        rearOnGround = false;
      }
    }
  }

  @Override
  public void outputToSmartDashboard(SmartDashboardCollection collection) {
    // LOG.trace("front {} {} {} - rear {} {} {}", frontLegHome.get(),
    //    frontLegMotor.getSelectedSensorPosition(), frontOnGround, rearLegHome.get(),
    //    rearLegMotorMaster.getSelectedSensorPosition(), rearOnGround);

    // collection.setSpiderlegsFrontPosition(frontLegMotor.getSelectedSensorPosition());
    // collection.setSpiderlegsRearPosition(rearLegMotorMaster.getSelectedSensorPosition());
    // collection.setSpiderlegsFrontHome(frontLegHome.get());
    // collection.setSpiderlegsRearHome(rearLegHome.get());

    SmartDashboard.putNumber(
        "spiderlegs.front.position", frontLegMotor.getSelectedSensorPosition());
    SmartDashboard.putNumber(
        "spiderlegs.rear.position", rearLegMotorMaster.getSelectedSensorPosition());
    /* SmartDashboard.putBoolean("spiderlegs.front.home", frontLegHome.get());
        SmartDashboard.putBoolean("spiderlegs.rear.home", rearLegHome.get());
    */
    SmartDashboard.putNumber("spiderlegs.front_detective", underFrontWheel.getAverageVoltage());
    SmartDashboard.putNumber("spiderlegs.rear_detective", underRearMiddleWheel.getAverageVoltage());
  }

  @Override
  public void stop() {
    wantedState = WantedState.OFF;
  }

  @Override
  public void zeroSensors() {
    if (frontLegHome.get()) {
      if (!frontHomed) {
        frontLegMotor.setSelectedSensorPosition(0);
        LED.getInstance().setFrontSpiderLegsHomed(true);
        LOG.info("front spiderlegs homed");
      }
      frontHomed = true;
    }

    if (rearLegHome.get()) {
      if (!rearHomed) {
        rearLegMotorMaster.setSelectedSensorPosition(0);
        LED.getInstance().setRearSpiderLegsHomed(true);
        LOG.info("rear spiderlegs homed");
      }
      rearHomed = true;
    }
  }

  public boolean shouldDrive() {
    return SystemState.CLIMBING_HOLD.equals(systemState)
        || SystemState.CLIMBING.equals(systemState)
        || SystemState.CLIMBING_L2_HOLD.equals(systemState)
        || SystemState.CLIMBING_L2.equals(systemState);
  }

  public boolean shouldHoldPosition() {
    return (SystemState.CLIMBING_HOLD.equals(systemState)
            || SystemState.CLIMBING_L2_HOLD.equals(systemState))
        && rearOnGround
        && Timer.getFPGATimestamp() - firstHeld > 1;
  }

  @Override
  public void test() {}

  @Override
  public String id() {
    return TAG;
  }

  public void setWantedState(WantedState wantedState) {
    synchronized (wantedState) {
      if (!((WantedState.CLIMB.equals(this.wantedState)
              || WantedState.CLIMB_L2.equals(this.wantedState))
          && (WantedState.PENDING_CLIMB.equals(wantedState)
              || WantedState.CLIMB.equals(wantedState)
              || WantedState.CLIMB_L2.equals(wantedState)))) {
        this.wantedState = wantedState;
      }
    }
  }
}
