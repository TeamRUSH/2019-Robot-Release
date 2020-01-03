package net.teamrush27.frc2019.constants;

public interface RobotConfiguration {

  double getArmHomePosition();

  Integer getJawIntakePosition();

  Integer getJawExhaustPosition();

  Integer getJawRetractPosition();

  Integer getWristHomePosition();

  double getArmMaxExtension();

  double getArmLevel3CargoExtension();

  double getArmLevel3HatchExtension();

  double getLimelightDriveForwardPercent();

  double getDriveKv();

  double getDriveKa();

  double getDriveVIntercept();

  double getScrubFactor();

  boolean isPracticeBot();
}
