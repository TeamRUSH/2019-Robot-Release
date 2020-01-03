package net.teamrush27.frc2019.constants;

public class CompBot implements RobotConfiguration {

  // 1940
  @Override
  public double getArmHomePosition() {
    return 171.474609375;
  }

  @Override
  public Integer getJawIntakePosition() {
    return 41;
  }

  @Override
  public Integer getJawExhaustPosition() {
    return 81;
  }

  @Override
  public Integer getJawRetractPosition() {
    return -281;
  }

  @Override
  public Integer getWristHomePosition() {
    return 2100;
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
    return .3;
  }

  @Override
  public double getDriveKv() {
    return 0.15919001960377988;
  }

  @Override
  public double getDriveKa() {
    return 0.02429265696741975;
  }

  @Override
  public double getDriveVIntercept() {
    return 0.6225547106218224;
  }

  @Override
  public double getScrubFactor() {
    return 1.1580863831479828;
  }

  @Override
  public boolean isPracticeBot() {
    return false;
  }
}
