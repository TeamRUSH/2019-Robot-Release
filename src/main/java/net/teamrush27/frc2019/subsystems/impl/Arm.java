package net.teamrush27.frc2019.subsystems.impl;

import com.revrobotics.CANError;
import com.revrobotics.CANPIDController.AccelStrategy;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.CANSparkMaxLowLevel.PeriodicFrame;
import com.revrobotics.ControlType;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import net.teamrush27.frc2019.Robot;
import net.teamrush27.frc2019.base.RobotMap;
import net.teamrush27.frc2019.loops.ILooper;
import net.teamrush27.frc2019.loops.Loop;
import net.teamrush27.frc2019.subsystems.Subsystem;
import net.teamrush27.frc2019.subsystems.impl.dto.ArmInput;
import net.teamrush27.frc2019.subsystems.impl.dto.SmartDashboardCollection;
import net.teamrush27.frc2019.util.math.CircularBuffer;
import net.teamrush27.frc2019.util.math.MathUtils;
import net.teamrush27.frc2019.wrappers.InvertableDigitalInput;
import org.apache.logging.log4j.LogManager;
import org.apache.logging.log4j.Logger;

public class Arm extends Subsystem {

  private static final String TAG = "ARM";
  private static final Logger LOG = LogManager.getLogger(Arm.class);
  private static Arm INSTANCE = null;

  // gear ratio / degrees in a full rotation (360)
  private static final double ROTATIONS_PER_DEGREE = 99.410158 / 360d; // 103.6
  // gear ratio / (sprocket diameter * 2 [accts for 3rd stage] * pi)
  private static final double ROTATIONS_PER_INCH = 1.0642462836;

  private static final double ROTATION_EPSILON = 3;
  private static final double EXTENSION_EPSILON = 1;

  private boolean lockPosition;

  public static Arm getInstance() {
    if (INSTANCE == null) {
      INSTANCE = new Arm();
    }
    return INSTANCE;
  }

  public enum WantedState {
    OFF,
    OPEN_LOOP,
    CLOSED_LOOP
  }

  public enum SystemState {
    OFF,
    OPEN_LOOP,
    CLOSED_LOOP
  }

  private WantedState wantedState = WantedState.OFF;
  private SystemState systemState = SystemState.OFF;

  private boolean stateChanged = false;
  private double currentStateStartTime = 0d;

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
            case OFF:
            default:
              newState = handleOff(timestamp);
              break;
          }
          if (newState != systemState) {
            LOG.info("Arm state {} to {}", systemState, newState);
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

  private final CANSparkMax rotationMotorMaster;
  private final CANSparkMax rotationMotorSlave;
  private final CANSparkMax extensionMotor;

  private final DigitalInput rotationHomeSensor;
  private final DigitalInput extensionHomeSensor;

  private Boolean rotationHomed = false;
  private Boolean extensionHomed = false;

  private ArmInput openLoopInput = new ArmInput(0d, 0d);
  private ArmInput closedLoopInput = new ArmInput(0d, 0d);

  private ArmState armState = new ArmState(0, 0, false, false);

  private CircularBuffer maxExtOutputBuffer = new CircularBuffer(30);
  private CircularBuffer currentBuffer = new CircularBuffer(30);
  private boolean forcePosition = false;

  public Arm() {
    rotationMotorMaster =
        new CANSparkMax(RobotMap.ARM_ROTATION_MASTER_CAN_ID, MotorType.kBrushless);
    rotationMotorMaster.restoreFactoryDefaults();
    rotationMotorMaster.setIdleMode(IdleMode.kBrake);
    rotationMotorMaster.setSmartCurrentLimit(50);
    rotationMotorMaster.setPeriodicFramePeriod(PeriodicFrame.kStatus0, 5);
    rotationMotorMaster.setPeriodicFramePeriod(PeriodicFrame.kStatus1, 5);
    rotationMotorMaster.setPeriodicFramePeriod(PeriodicFrame.kStatus2, 5);
    rotationMotorMaster.setInverted(false);

    rotationMotorSlave = new CANSparkMax(RobotMap.ARM_ROTATION_SLAVE_CAN_ID, MotorType.kBrushless);
    rotationMotorSlave.restoreFactoryDefaults();
    rotationMotorSlave.setIdleMode(IdleMode.kBrake);
    rotationMotorSlave.setSmartCurrentLimit(50);
    rotationMotorSlave.follow(rotationMotorMaster);
    rotationMotorSlave.setPeriodicFramePeriod(PeriodicFrame.kStatus0, 100);
    rotationMotorSlave.setPeriodicFramePeriod(PeriodicFrame.kStatus1, 500);
    rotationMotorSlave.setPeriodicFramePeriod(PeriodicFrame.kStatus2, 500);

    extensionMotor = new CANSparkMax(RobotMap.ARM_EXTENSION_CAN_ID, MotorType.kBrushless);
    extensionMotor.restoreFactoryDefaults();
    extensionMotor.enableVoltageCompensation(12);
    extensionMotor.setIdleMode(IdleMode.kBrake);
    extensionMotor.setInverted(true);
    extensionMotor.setSmartCurrentLimit(100);
    extensionMotor.setPeriodicFramePeriod(PeriodicFrame.kStatus1, 5);
    extensionMotor.setPeriodicFramePeriod(PeriodicFrame.kStatus2, 5);

    rotationHomeSensor = new InvertableDigitalInput(RobotMap.ARM_ROTATION_HOME_SENSOR_ID, true);
    extensionHomeSensor = new InvertableDigitalInput(RobotMap.ARM_EXTENSION_HOME_SENSOR_ID, true);

    reloadGains();
  }

  private void reloadGains() {
    rotationMotorMaster.getPIDController().setP(0.00016, 0);
    rotationMotorMaster.getPIDController().setI(0, 0);
    rotationMotorMaster.getPIDController().setD(0.0004, 0);
    rotationMotorMaster.getPIDController().setFF(0.000156, 0);
    rotationMotorMaster.getPIDController().setOutputRange(-1, 1, 0);
    rotationMotorMaster.getPIDController().setDFilter(0.25, 0);
    rotationMotorMaster
        .getPIDController()
        .setSmartMotionAccelStrategy(AccelStrategy.kTrapezoidal, 0);
    rotationMotorMaster
        .getPIDController()
        .setSmartMotionAllowedClosedLoopError(3 * ROTATIONS_PER_DEGREE, 0);
    rotationMotorMaster.getPIDController().setSmartMotionMaxAccel(5700, 0);
    rotationMotorMaster.getPIDController().setSmartMotionMaxVelocity(4000, 0);
    rotationMotorMaster.getPIDController().setSmartMotionMinOutputVelocity(0, 0);

    rotationMotorMaster.getPIDController().setP(0.2, 1);
    rotationMotorMaster.getPIDController().setI(0, 1);
    rotationMotorMaster.getPIDController().setD(2, 1);
    rotationMotorMaster.getPIDController().setFF(0, 1);
    rotationMotorMaster.getPIDController().setOutputRange(-.375, .375, 1);
    rotationMotorMaster.getPIDController().setDFilter(0.25, 1);

    extensionMotor.getPIDController().setP(0.00032, 0);
    extensionMotor.getPIDController().setI(0.0000001, 0);
    extensionMotor.getPIDController().setD(0.0004, 0);
    extensionMotor.getPIDController().setFF(0.000156, 0);
    extensionMotor.getPIDController().setOutputRange(-1, 1);
    extensionMotor.getPIDController().setDFilter(0.25, 0);
    extensionMotor.getPIDController().setSmartMotionAccelStrategy(AccelStrategy.kTrapezoidal, 0);
    extensionMotor
        .getPIDController()
        .setSmartMotionAllowedClosedLoopError(.5 * ROTATIONS_PER_INCH, 0);
    extensionMotor.getPIDController().setSmartMotionMaxAccel(7000, 0);
    extensionMotor.getPIDController().setSmartMotionMaxVelocity(5700, 0);
    extensionMotor.getPIDController().setSmartMotionMinOutputVelocity(0, 0);

    extensionMotor.getPIDController().setP(0.1, 1);
    extensionMotor.getPIDController().setI(0, 1);
    extensionMotor.getPIDController().setD(0, 1);
    extensionMotor.getPIDController().setFF(0, 1);
    extensionMotor.getPIDController().setOutputRange(-1, 1, 1);
    extensionMotor.getPIDController().setDFilter(0.25, 1);
  }

  @Override
  public void outputToSmartDashboard(SmartDashboardCollection collection) {
    // maxExtOutputBuffer.addValue(Math.abs(extensionMotor.getEncoder().getVelocity()));
    // currentBuffer.addValue(extensionMotor.getOutputCurrent());

    // collection.setArmRotation(armState.getRotationInDegrees());
    // collection.setArmRotationReset(armState.isRotationAtHome());
    // collection.setArmExtension(armState.getExtensionInInches());
    // collection.setArmExtensionReset(armState.isExtensionAtHome());
    // collection.setArmRotationOutput(rotationMotorMaster.getAppliedOutput());
    // collection.setArmExtensionOutput(extensionMotor.getAppliedOutput());
    // collection.setArmExtensionSpeed(maxExtOutputBuffer.getMax());
    // collection.setArmExtensionCurrent(currentBuffer.getMax());

    SmartDashboard.putNumber("arm.rotation", armState.getRotationInDegrees());
    SmartDashboard.putBoolean("arm.rotation.reset", armState.isRotationAtHome());
    SmartDashboard.putNumber("arm.extension", armState.getExtensionInInches());
    SmartDashboard.putBoolean("arm.extension.reset", armState.isExtensionAtHome());
    // SmartDashboard.putNumber("arm.rotation.output", rotationMotorMaster.getAppliedOutput());
    // SmartDashboard.putNumber("arm.extension.output", extensionMotor.getAppliedOutput());
    // SmartDashboard.putNumber("arm.extension.speed", maxExtOutputBuffer.getMax());
    // SmartDashboard.putNumber("arm.extension.current", currentBuffer.getMax());

  }

  private SystemState handleClosedLoop(double timestamp) {
    armState.setRotationDemandInDegrees(closedLoopInput.getRotationInput());
    armState.setExtensionDemandInInches(closedLoopInput.getExtensionInput());

    return defaultStateTransfer(timestamp);
  }

  private SystemState handleOpenLoop(double timestamp) {
    armState.setRotationDemandInDegrees(openLoopInput.getRotationInput());
    armState.setExtensionDemandInInches(openLoopInput.getExtensionInput());

    return defaultStateTransfer(timestamp);
  }

  private SystemState handleOff(double timestamp) {
    armState.setExtensionDemandInInches(0);
    armState.setRotationDemandInDegrees(0);

    return defaultStateTransfer(timestamp);
  }

  private SystemState defaultStateTransfer(double timestamp) {
    switch (wantedState) {
      case OPEN_LOOP:
        return SystemState.OPEN_LOOP;
      case CLOSED_LOOP:
        return SystemState.CLOSED_LOOP;
      case OFF:
      default:
        return SystemState.OFF;
    }
  }

  boolean lastHomed = false;

  @Override
  public void readPeriodicInputs() {
    ArmState newArmState =
        new ArmState( //
            rotationMotorMaster.getEncoder().getPosition(), //
            extensionMotor.getEncoder().getPosition(), //
            rotationHomeSensor.get(), //
            extensionHomeSensor.get());

    newArmState.setExtensionDemandInInches(armState.getExtensionDemandInInches());
    newArmState.setRotationDemandInDegrees(armState.getRotationDemandInDegrees());

    armState = newArmState;

    if (armState.isExtensionAtHome() && !lastHomed) {
      if (CANError.kOK.equals(extensionMotor.setEncPosition(0))) {
        LOG.info("extension homed");
        extensionHomed = true;
        LED.getInstance().setExtensionHomed(true);
        lastHomed = true;
      }
    } else if (!armState.isExtensionAtHome()) {
      lastHomed = false;
    }
  }

  public void setForcePosition(boolean forcePosition) {
    this.forcePosition = forcePosition;
  }

  @Override
  public void writePeriodicOutputs() {
    if (stateChanged) {
      return;
    }
    switch (systemState) {
      case CLOSED_LOOP:
        boolean rotation_close =
            forcePosition
                || MathUtils.epsilonEquals(
                    armState.getRotationInDegrees(),
                    armState.getRotationDemandInDegrees(),
                    ROTATION_EPSILON);
        boolean extension_close =
            forcePosition
                || MathUtils.epsilonEquals(
                    armState.getExtensionInInches(),
                    armState.getExtensionDemandInInches(),
                    EXTENSION_EPSILON);
        if (rotationHomed) {
          if (rotation_close && fixOnHold) {
            //            System.out.println("Holding Rotation in POSITION PID");
            Drivetrain.getInstance().fixArm();
            fixOnHold = false;
          }
          rotationMotorMaster
              .getPIDController()
              .setReference(
                  armState.getRotationDemandInRotations(),
                  rotation_close ? ControlType.kPosition : ControlType.kSmartMotion,
                  rotation_close ? 1 : 0);
        } else {
          rotationMotorMaster.set(0);
        }

        if (extensionHomed) {
          if (extension_close) {
            //            System.out.println("Holding Extension in POSITION PID");
          }

          extensionMotor
              .getPIDController()
              .setReference(
                  armState.getExtensionDemandInRotations(),
                  extension_close ? ControlType.kPosition : ControlType.kSmartMotion,
                  extension_close ? 1 : 0);
        } else {
          extensionMotor.set(0);
        }
        break;

      case OPEN_LOOP:
      case OFF:
        extensionMotor.set(armState.getExtensionDemandInInches());
        rotationMotorMaster.set(armState.getRotationDemandInDegrees());
      default:
    }
  }

  @Override
  public void stop() {
    // wantedState = WantedState.OFF;
  }

  public void reset() {
    extensionHomed = false;
  }

  @Override
  public void zeroSensors() {
    if (!rotationHomed) {
      if (armState.isRotationAtHome()) {
        if (CANError.kOK.equals(rotationMotorMaster.setEncPosition(0))) {
          LOG.info("rotation homed");
          rotationHomed = true;
          LED.getInstance().setRotationHomed(true);
        }
      }
    }

    if (!extensionHomed) {
      if (armState.isExtensionAtHome()) {
        if (CANError.kOK.equals(extensionMotor.setEncPosition(0))) {
          LOG.info("extension homed");
          extensionHomed = true;
          LED.getInstance().setExtensionHomed(true);
        }
      }
    }
  }

  @Override
  public void registerEnabledLoops(ILooper enabledLooper) {
    enabledLooper.register(loop);
  }

  public void setWantedState(WantedState wantedState) {
    this.wantedState = wantedState;
  }

  @Override
  public void test() {}

  public void setOpenLoopInput(ArmInput armInput) {
    synchronized (openLoopInput) {
      openLoopInput = armInput;
    }
  }

  public void setClosedLoopInput(ArmInput armInput, Boolean negate) {
    if (negate) {
      armInput.setRotationInput(armInput.getRotationInput() * -1);
    }
    setClosedLoopInput(armInput);
  }

  public void setClosedLoopInput(ArmInput armInput) {
    synchronized (closedLoopInput) {
      closedLoopInput = armInput;
    }
  }

  private boolean fixOnHold = false;

  public void latchFixIt() {
    fixOnHold = true;
  }

  @Override
  public String id() {
    return TAG;
  }

  public class ArmState {

    // in degrees from vertical
    private final double rotation;
    private double rotationSetpoint;
    // in inches
    private final double extension;
    private double extensionSetpoint;

    private final boolean rotationAtHome;
    private final boolean extensionAtHome;

    public ArmState(
        double rotation, double extension, boolean rotationAtHome, boolean extensionAtHome) {
      this.rotation = rotation;
      this.extension = extension;
      this.rotationAtHome = rotationAtHome;
      this.extensionAtHome = extensionAtHome;
    }

    public Double getRotationInDegrees() {
      return rotation / ROTATIONS_PER_DEGREE;
    }

    public Double getExtensionInInches() {
      return extension / ROTATIONS_PER_INCH;
    }

    public boolean isRotationAtHome() {
      return rotationAtHome;
    }

    public boolean isExtensionAtHome() {
      return extensionAtHome;
    }

    public void setRotationDemandInDegrees(double demand) {
      rotationSetpoint = demand;
    }

    public void setExtensionDemandInInches(double extensionSetpoint) {
      this.extensionSetpoint = extensionSetpoint;
    }

    public double getRotationDemandInDegrees() {
      return rotationSetpoint;
    }

    public double getExtensionDemandInInches() {
      return extensionSetpoint;
    }

    public double getExtensionDemandInRotations() {
      return extensionSetpoint * ROTATIONS_PER_INCH;
    }

    public double getRotationDemandInRotations() {
      return rotationSetpoint * ROTATIONS_PER_DEGREE;
    }
  }

  public ArmState getArmState() {
    return armState;
  }

  public void setAbsolutePosition(double selectedSensorPosition) {
    double absolutePosition = selectedSensorPosition / 1024d * 90d * -1d;
    //		LOG.info("abs: {}", absolutePosition);
    if (rotationHomed
        && Math.abs(
                (absolutePosition + Robot.ROBOT_CONFIGURATION.getArmHomePosition())
                    - armState.getRotationInDegrees())
            > 5) {
      LOG.warn(
          String.format(
              "rejecting large change in absolute position: goal - %s\tcurr - %s",
              absolutePosition, armState.getRotationInDegrees()));
      // Drivetrain.getInstance().fixRotationEncoder();
    } else {
      rotationMotorMaster.setEncPosition(
          (absolutePosition + Robot.ROBOT_CONFIGURATION.getArmHomePosition())
              * ROTATIONS_PER_DEGREE);
    }

    if (!rotationHomed) {
      rotationHomed = true;
      LED.getInstance().setRotationHomed(true);
    }
  }
}
