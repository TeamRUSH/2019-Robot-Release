package net.teamrush27.frc2019.constants;

public class PracticeBot implements RobotConfiguration {

  @Override
  public double getArmHomePosition() {
    return 212.6953125;
  }

  @Override
  public Integer getJawIntakePosition() {
    return -57;
  }

  @Override
  public Integer getJawExhaustPosition() {
    return -11;
  }

  @Override
  public Integer getJawRetractPosition() {
    return -360;
  }

  @Override
  public Integer getWristHomePosition() {
    return 2209;
  }

  @Override
  public double getArmMaxExtension() {
    return 49d;
  }

  @Override
  public double getArmLevel3CargoExtension() {
    return 49d;
  }

  @Override
  public double getArmLevel3HatchExtension() {
    return 46d;
  }

  @Override
  public double getLimelightDriveForwardPercent() {
    return .40;
  }

  @Override
  public double getDriveKv() {
    return 0.14350156089390304;
  }

  @Override
  public double getDriveKa() {
    return 0.023570670895268462;
  }

  @Override
  public double getDriveVIntercept() {
    return 1.1356961958921263;
  }

  @Override
  public double getScrubFactor() {
    return 1.1525606838366464;
  }

  @Override
  public boolean isPracticeBot() {
    return true;
  }
}
