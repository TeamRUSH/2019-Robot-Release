package net.teamrush27.frc2019.constants;

import edu.wpi.first.wpilibj.DriverStation;
import java.io.File;

public class RobotConfigurationFactory {

  public static RobotConfiguration getRobotConfiguration() {
    File practiceFile = new File("/home/lvuser/THIS_IS_THE_PRACTICE_BOT");
    File compFile = new File("/home/lvuser/THIS_IS_THE_COMP_BOT");
    if (compFile.exists()) {
      DriverStation.reportError("THIS IS THE COMP BOT", false);
      return new CompBot();
    }

    if (practiceFile.exists()) {
      DriverStation.reportError("THIS IS THE PRACTICE BOT", false);
      return new PracticeBot();
    }

    DriverStation.reportError("ERROR: DID NOT FIND EITHER PRACTICE OR COMP FILES", false);

    return new CompBot();
  }
}
