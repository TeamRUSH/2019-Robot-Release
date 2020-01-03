package com.swervedrivespecialties.exampleswerve;

import com.swervedrivespecialties.exampleswerve.config.ConfigLoader;
import com.swervedrivespecialties.exampleswerve.config.RobotConfig;
import com.swervedrivespecialties.exampleswerve.config.RobotMap;
import com.swervedrivespecialties.exampleswerve.drivers.BeeSwerveModule;
import com.swervedrivespecialties.exampleswerve.subsystems.DrivetrainSubsystem;
import edu.wpi.first.networktables.PersistentException;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.command.Scheduler;
import edu.wpi.first.wpilibj.livewindow.LiveWindow;
import org.frcteam2910.common.drivers.SwerveModule;
import org.frcteam2910.common.robot.subsystems.SubsystemManager;

public class Robot extends TimedRobot {
  /**
   * How often the control thread should run in seconds. By default it runs every 5 milliseconds.
   */
  private static final double UPDATE_DT = 5.0e-3;

  private ConfigLoader loader;
  private RobotMap map;
  private RobotConfig config;

  private static OI oi;
  private static DrivetrainSubsystem drivetrain;
  private static SubsystemManager subsystemManager;

  public static OI getOi() {
    return oi;
  }

  // 20 00 32 00 01 00 01 00 03

  @Override
  public void robotInit() {
    loader = new ConfigLoader();

    try {
      config = loader.loadRobotConfig();
      map = loader.loadRobotMap();
    } catch (PersistentException e) {
      e.printStackTrace();
      int d = 1 / 0;
    }

    drivetrain = new DrivetrainSubsystem(map, config);
    oi = new OI(drivetrain);
    subsystemManager = new SubsystemManager(drivetrain);

    LiveWindow.disableAllTelemetry();
    LiveWindow.setEnabled(false);
    subsystemManager.enableKinematicLoop(UPDATE_DT);
  }

  boolean once = true;

  @Override
  public void robotPeriodic() {
    subsystemManager.outputToSmartDashboard();
  }

  @Override
  public void teleopPeriodic() {
    Scheduler.getInstance().run();
  }

  private double disabledInitTime;
  private boolean zeroed;

  @Override
  public void disabledInit() {
    disabledInitTime = Timer.getFPGATimestamp();
    zeroed = false;
  }

  @Override
  public void disabledPeriodic() {
    if (Timer.getFPGATimestamp() - disabledInitTime > 0.1 && !zeroed) {
      for (SwerveModule module : drivetrain.getSwerveModules()) {
        ((BeeSwerveModule) module).zero();
      }

      System.err.println("Zeroed!");
      zeroed = true;
    }
  }
}
