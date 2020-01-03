package com.swervedrivespecialties.exampleswerve.drivers;

import com.revrobotics.CANEncoder;
import com.revrobotics.CANPIDController;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.ControlType;
import com.swervedrivespecialties.exampleswerve.config.RobotConfig.SwerveModuleConfig;
import com.swervedrivespecialties.exampleswerve.config.RobotMap.SwerveModuleMap;
import edu.wpi.first.wpilibj.Notifier;
import org.frcteam2910.common.drivers.SwerveModule;
import org.frcteam2910.common.math.Vector2;

public class BeeSwerveModule extends SwerveModule {
  /** The default drive encoder rotations per unit. */
  public static final double DEFAULT_DRIVE_ROTATIONS_PER_UNIT =
      (1.0 / (4.0 * Math.PI)) * (60.0 / 15.0) * (18.0 / 26.0) * (42.0 / 14.0);

  private static final double CAN_UPDATE_RATE = 50.0;

  private CANSparkMax steeringMotor;
  private CANEncoder angleEncoder;
  private CANSparkMax driveMotor;
  private CANEncoder driveEncoder;
  private PWMEncoder angleOrigin;

  CANPIDController pidController;

  private SwerveModuleConfig moduleConfig;

  private final Object canLock = new Object();
  private double driveDistance = 0.0;
  private double drivePercentOutput = 0.0;
  private double driveVelocity = 0.0;
  private double driveCurrent = 0.0;
  private double goalAngle = 0.0;
  private double currentAngle = 0.0;

  private double driveEncoderRotationsPerUnit = DEFAULT_DRIVE_ROTATIONS_PER_UNIT;

  /** All CAN operations are done in a separate thread to reduce latency on the control thread */
  private Notifier canUpdateNotifier =
      new Notifier(
          () -> {
            double driveRotations = driveEncoder.getPosition();
            synchronized (canLock) {
              driveDistance = driveRotations * (1.0 / driveEncoderRotationsPerUnit);
            }

            double driveRpm = driveEncoder.getVelocity();
            synchronized (canLock) {
              driveVelocity = driveRpm * (1.0 / 60.0) * (1.0 / driveEncoderRotationsPerUnit);
            }

            double localDriveCurrent = driveMotor.getOutputCurrent();
            synchronized (canLock) {
              driveCurrent = localDriveCurrent;
            }

            double localCurrentAngle = angleEncoder.getPosition();
            synchronized (canLock) {
              currentAngle = localCurrentAngle - moduleConfig.ANGLE_OFFSET;
            }

            double localDrivePercentOutput;
            synchronized (canLock) {
              localDrivePercentOutput = drivePercentOutput;
            }
            driveMotor.set(localDrivePercentOutput);

            double localGoalAngle;
            synchronized (canLock) {
              localGoalAngle = goalAngle;
            }
            steeringMotor.getPIDController().setReference(localGoalAngle, ControlType.kPosition);
          });

  /**
   * @param modulePosition The module's offset from the center of the robot's center of rotation
   * @param angleOffset An angle in radians that is used to offset the angle encoder
   * @param angleMotor The motor that controls the module's angle
   * @param driveMotor The motor that drives the module's wheel
   * @param angleEncoder The analog input for the angle encoder
   * @param angleConstants The PID constants for the steering motor
   */
  public BeeSwerveModule(
      Vector2 modulePosition, SwerveModuleConfig moduleConfig, SwerveModuleMap moduleMap) {
    super(modulePosition);
    this.moduleConfig = moduleConfig;

    steeringMotor = new CANSparkMax(moduleMap.ANGLE_MOTOR, MotorType.kBrushless);
    steeringMotor.restoreFactoryDefaults();
    angleEncoder = new CANEncoder(steeringMotor);

    driveMotor = new CANSparkMax(moduleMap.DRIVE_MOTOR, MotorType.kBrushless);
    driveMotor.restoreFactoryDefaults();
    driveMotor.setIdleMode(IdleMode.kCoast);
    driveEncoder = new CANEncoder(driveMotor);

    angleOrigin = new PWMEncoder(moduleMap.ANGLE_ENCODER);

    angleEncoder.setPositionConversionFactor(2.0 * Math.PI / 16.0);

    driveMotor.setSmartCurrentLimit(60);
    driveEncoder.setPosition(0);
    driveMotor.setInverted(moduleMap.DRIVE_INVERT);

    pidController = this.steeringMotor.getPIDController();

    configurePID();

    canUpdateNotifier.startPeriodic(1.0 / CAN_UPDATE_RATE);
  }

  public void zero() {
    angleEncoder.setPosition((1 - angleOrigin.duty()) * 2 * Math.PI);
  }

  public void configurePID() {
    pidController.setP(moduleConfig.PID_CONSTANTS.P);
    pidController.setI(moduleConfig.PID_CONSTANTS.I);
    pidController.setD(moduleConfig.PID_CONSTANTS.D);
    pidController.setFF(moduleConfig.PID_CONSTANTS.FF);
    pidController.setDFilter(moduleConfig.PID_CONSTANTS.DF);
    pidController.setOutputRange(-1.0, 1.0);
    // ctrlr.setSmartMotionAllowedClosedLoopError(1.0, 0);

    // ctrlr.setSmartMotionAllowedClosedLoopError(0.1, 0);
    // ctrlr.setSmartMotionMaxAccel(10000, 0);
    // ctrlr.setSmartMotionMaxVelocity(5700, 0);
  }

  @Override
  protected double readAngle() {
    double angle = currentAngle;
    angle %= 2.0 * Math.PI;
    if (angle < 0.0) {
      angle += 2.0 * Math.PI;
    }

    return angle;
  }

  @Override
  protected double readDistance() {
    synchronized (canLock) {
      return driveDistance;
    }
  }

  protected double readVelocity() {
    synchronized (canLock) {
      return driveVelocity;
    }
  }

  protected double readDriveCurrent() {
    double localDriveCurrent;
    synchronized (canLock) {
      localDriveCurrent = driveCurrent;
    }

    return localDriveCurrent;
  }

  @Override
  public double getCurrentVelocity() {
    return readVelocity();
  }

  @Override
  public double getDriveCurrent() {
    return readDriveCurrent();
  }

  @Override
  protected void setTargetAngle(double angle) {
    //  SmartDashboard.putNumber(
    //      String.format("%s PWM angle", getName()), (1 - angleOrigin.duty()) * 360);
    //  SmartDashboard.putNumber(
    //      String.format("%s PWM rads", getName()), (1 - angleOrigin.duty()) * 2 * Math.PI);

    goalAngle = placeInAppropriate0To2PIScope(currentAngle, angle) + moduleConfig.ANGLE_OFFSET;
  }

  public static double placeInAppropriate0To2PIScope(double scopeReference, double newAngle) {
    double lowerBound;
    double upperBound;
    double lowerOffset = scopeReference % (2 * Math.PI);
    if (lowerOffset >= 0) {
      lowerBound = scopeReference - lowerOffset;
      upperBound = scopeReference + (2 * Math.PI - lowerOffset);
    } else {
      upperBound = scopeReference - lowerOffset;
      lowerBound = scopeReference - (2 * Math.PI + lowerOffset);
    }
    while (newAngle < lowerBound) {
      newAngle += 2 * Math.PI;
    }
    while (newAngle > upperBound) {
      newAngle -= 2 * Math.PI;
    }
    if (newAngle - scopeReference > Math.PI) {
      newAngle -= 2 * Math.PI;
    } else if (newAngle - scopeReference < -Math.PI) {
      newAngle += 2 * Math.PI;
    }
    return newAngle;
  }

  @Override
  protected void setDriveOutput(double output) {
    synchronized (canLock) {
      this.drivePercentOutput = output;
    }
  }

  public void setDriveEncoderRotationsPerUnit(double driveEncoderRotationsPerUnit) {
    synchronized (canLock) {
      this.driveEncoderRotationsPerUnit = driveEncoderRotationsPerUnit;
    }
  }
}
