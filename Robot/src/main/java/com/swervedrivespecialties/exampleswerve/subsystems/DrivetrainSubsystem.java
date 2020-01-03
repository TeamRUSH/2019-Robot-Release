package com.swervedrivespecialties.exampleswerve.subsystems;

import com.swervedrivespecialties.exampleswerve.commands.DriveCommand;
import com.swervedrivespecialties.exampleswerve.config.RobotConfig;
import com.swervedrivespecialties.exampleswerve.config.RobotMap;
import com.swervedrivespecialties.exampleswerve.drivers.BeeSwerveModule;
import edu.wpi.first.wpilibj.SPI;
import org.frcteam2910.common.drivers.Gyroscope;
import org.frcteam2910.common.drivers.SwerveModule;
import org.frcteam2910.common.math.Vector2;
import org.frcteam2910.common.robot.drivers.NavX;
import org.frcteam2910.common.robot.subsystems.SwerveDrivetrain;

public class DrivetrainSubsystem extends SwerveDrivetrain {
  private static final double TRACKWIDTH = 24.0;
  private static final double WHEELBASE = 24.0;

  private final SwerveModule[] swerveModules;

  private final Gyroscope gyroscope = new NavX(SPI.Port.kMXP);

  public DrivetrainSubsystem(RobotMap robotMap, RobotConfig robotConfig) {
    gyroscope.calibrate();
    gyroscope.setInverted(true); // You might not need to invert the gyro

    BeeSwerveModule frontLeftModule =
        new BeeSwerveModule(
            new Vector2(-TRACKWIDTH / 2.0, WHEELBASE / 2.0),
            robotConfig.FRONT_LEFT,
            robotMap.FRONT_LEFT);
    frontLeftModule.setName("Front Left");

    BeeSwerveModule frontRightModule =
        new BeeSwerveModule(
            new Vector2(TRACKWIDTH / 2.0, WHEELBASE / 2.0),
            robotConfig.FRONT_RIGHT,
            robotMap.FRONT_RIGHT);
    frontRightModule.setName("Front Right");

    BeeSwerveModule backLeftModule =
        new BeeSwerveModule(
            new Vector2(-TRACKWIDTH / 2.0, -WHEELBASE / 2.0),
            robotConfig.BACK_LEFT,
            robotMap.BACK_LEFT);
    backLeftModule.setName("Back Left");

    BeeSwerveModule backRightModule =
        new BeeSwerveModule(
            new Vector2(TRACKWIDTH / 2.0, -WHEELBASE / 2.0),
            robotConfig.BACK_RIGHT,
            robotMap.BACK_RIGHT);
    backRightModule.setName("Back Right");

    swerveModules =
        new SwerveModule[] {
          frontLeftModule, frontRightModule, backLeftModule, backRightModule,
        };
  }

  @Override
  public SwerveModule[] getSwerveModules() {
    return swerveModules;
  }

  @Override
  public Gyroscope getGyroscope() {
    return gyroscope;
  }

  @Override
  public double getMaximumVelocity() {
    return 0;
  }

  @Override
  public double getMaximumAcceleration() {
    return 0;
  }

  @Override
  protected void initDefaultCommand() {
    setDefaultCommand(new DriveCommand(this));
  }
}
