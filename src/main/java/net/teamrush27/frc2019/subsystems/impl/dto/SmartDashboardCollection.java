package net.teamrush27.frc2019.subsystems.impl.dto;

public class SmartDashboardCollection {

  // ARM
  private Double ArmRotation;
  private Boolean ArmRotationReset;
  private Double ArmExtension;
  private Boolean ArmExtensionReset;
  private Double ArmRotationOutput;
  private Double ArmExtensionOutput;
  private Double ArmExtensionSpeed;
  private Double ArmExtensionCurrent;

  // DRIVETRAIN
  private Integer ArmAbsoluteRotation;
  private Integer DrivetrainLeftPosition;
  private Integer DrivetrainRightPosition;

  // GRIPPER
  private Double DetectiveVoltage;
  private String GripperState;

  // LED

  // LIMELIGHTS
  private String LimelightMode;

  // ROBOTSTATEESTIMATOR

  // SPIDERLEGS
  private Integer SpiderlegsFrontPosition;
  private Integer SpiderlegsRearPosition;
  private Boolean SpiderlegsFrontHome;
  private Boolean SpiderlegsRearHome;

  // SUPERSTRUCTURE
  private Double SuperstructureOffset;

  // WRIST
  private Integer WristEncoderTicks;
  private Double WristEncoderDegrees;
  private Double WristPWMValue;
  private Double WristPWMDegrees;

  public Double getArmRotation() {
    return ArmRotation;
  }

  public void setArmRotation(Double armRotation) {
    ArmRotation = armRotation;
  }

  public Boolean getArmRotationReset() {
    return ArmRotationReset;
  }

  public void setArmRotationReset(Boolean armRotationReset) {
    ArmRotationReset = armRotationReset;
  }

  public Double getArmExtension() {
    return ArmExtension;
  }

  public void setArmExtension(Double armExtension) {
    ArmExtension = armExtension;
  }

  public Boolean getArmExtensionReset() {
    return ArmExtensionReset;
  }

  public void setArmExtensionReset(Boolean armExtensionReset) {
    ArmExtensionReset = armExtensionReset;
  }

  public Double getArmRotationOutput() {
    return ArmRotationOutput;
  }

  public void setArmRotationOutput(Double armRotationOutput) {
    ArmRotationOutput = armRotationOutput;
  }

  public Double getArmExtensionOutput() {
    return ArmExtensionOutput;
  }

  public void setArmExtensionOutput(Double armExtensionOutput) {
    ArmExtensionOutput = armExtensionOutput;
  }

  public Double getArmExtensionSpeed() {
    return ArmExtensionSpeed;
  }

  public void setArmExtensionSpeed(Double armExtensionSpeed) {
    ArmExtensionSpeed = armExtensionSpeed;
  }

  public Double getArmExtensionCurrent() {
    return ArmExtensionCurrent;
  }

  public void setArmExtensionCurrent(Double armExtensionCurrent) {
    ArmExtensionCurrent = armExtensionCurrent;
  }

  public Integer getArmAbsoluteRotation() {
    return ArmAbsoluteRotation;
  }

  public void setArmAbsoluteRotation(Integer armAbsoluteRotation) {
    ArmAbsoluteRotation = armAbsoluteRotation;
  }

  public Integer getDrivetrainLeftPosition() {
    return DrivetrainLeftPosition;
  }

  public void setDrivetrainLeftPosition(Integer drivetrainLeftPosition) {
    DrivetrainLeftPosition = drivetrainLeftPosition;
  }

  public Integer getDrivetrainRightPosition() {
    return DrivetrainRightPosition;
  }

  public void setDrivetrainRightPosition(Integer drivetrainRightPosition) {
    DrivetrainRightPosition = drivetrainRightPosition;
  }

  public Double getDetectiveVoltage() {
    return DetectiveVoltage;
  }

  public void setDetectiveVoltage(Double detectiveVoltage) {
    DetectiveVoltage = detectiveVoltage;
  }

  public String getGripperState() {
    return GripperState;
  }

  public void setGripperState(String gripperState) {
    GripperState = gripperState;
  }

  public String getLimelightMode() {
    return LimelightMode;
  }

  public void setLimelightMode(String limelightMode) {
    LimelightMode = limelightMode;
  }

  public Integer getSpiderlegsFrontPosition() {
    return SpiderlegsFrontPosition;
  }

  public void setSpiderlegsFrontPosition(Integer spiderlegsFrontPosition) {
    SpiderlegsFrontPosition = spiderlegsFrontPosition;
  }

  public Integer getSpiderlegsRearPosition() {
    return SpiderlegsRearPosition;
  }

  public void setSpiderlegsRearPosition(Integer spiderlegsRearPosition) {
    SpiderlegsRearPosition = spiderlegsRearPosition;
  }

  public Boolean getSpiderlegsFrontHome() {
    return SpiderlegsFrontHome;
  }

  public void setSpiderlegsFrontHome(Boolean spiderlegsFrontHome) {
    SpiderlegsFrontHome = spiderlegsFrontHome;
  }

  public Boolean getSpiderlegsRearHome() {
    return SpiderlegsRearHome;
  }

  public void setSpiderlegsRearHome(Boolean spiderlegsRearHome) {
    SpiderlegsRearHome = spiderlegsRearHome;
  }

  public Double getSuperstructureOffset() {
    return SuperstructureOffset;
  }

  public void setSuperstructureOffset(Double superstructureOffset) {
    SuperstructureOffset = superstructureOffset;
  }

  public Integer getWristEncoderTicks() {
    return WristEncoderTicks;
  }

  public void setWristEncoderTicks(Integer wristEncoderTicks) {
    WristEncoderTicks = wristEncoderTicks;
  }

  public Double getWristEncoderDegrees() {
    return WristEncoderDegrees;
  }

  public void setWristEncoderDegrees(Double wristEncoderDegrees) {
    WristEncoderDegrees = wristEncoderDegrees;
  }

  public Double getWristPWMValue() {
    return WristPWMValue;
  }

  public void setWristPWMValue(Double wristPWMValue) {
    WristPWMValue = wristPWMValue;
  }

  public Double getWristPWMDegrees() {
    return WristPWMDegrees;
  }

  public void setWristPWMDegrees(Double wristPWMDegrees) {
    WristPWMDegrees = wristPWMDegrees;
  }
}
