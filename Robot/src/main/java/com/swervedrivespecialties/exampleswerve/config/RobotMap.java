package com.swervedrivespecialties.exampleswerve.config;

import frcconf.Configurable;

@Configurable
public class RobotMap {
  public SwerveModuleMap FRONT_LEFT = new SwerveModuleMap();
  public SwerveModuleMap FRONT_RIGHT = new SwerveModuleMap();
  public SwerveModuleMap BACK_LEFT = new SwerveModuleMap();
  public SwerveModuleMap BACK_RIGHT = new SwerveModuleMap();

  public static class SwerveModuleMap {
    public int ANGLE_MOTOR;
    public int ANGLE_ENCODER;
    public int DRIVE_MOTOR;
    public boolean DRIVE_INVERT;
  }
}
