package com.swervedrivespecialties.exampleswerve.config;

import frcconf.Configurable;

@Configurable
public class RobotConfig {
  public SwerveModuleConfig FRONT_LEFT = new SwerveModuleConfig();
  public SwerveModuleConfig FRONT_RIGHT = new SwerveModuleConfig();
  public SwerveModuleConfig BACK_LEFT = new SwerveModuleConfig();
  public SwerveModuleConfig BACK_RIGHT = new SwerveModuleConfig();

  public static class SwerveModuleConfig {
    public double ANGLE_OFFSET;
    public NeoPidConstants PID_CONSTANTS = new NeoPidConstants();

    public static class NeoPidConstants {
      public double P;
      public double I;
      public double D;
      public double DF;
      public double FF;
    }
  }
}
