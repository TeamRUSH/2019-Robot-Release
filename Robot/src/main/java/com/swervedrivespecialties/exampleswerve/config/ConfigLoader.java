package com.swervedrivespecialties.exampleswerve.config;

import edu.wpi.first.networktables.PersistentException;
import java.io.File;

public class ConfigLoader {
  private RobotMap robotMapInstance;
  private RobotConfig robotConfigInstance;

  private Bot type;

  public RobotMap loadRobotMap() throws PersistentException {
    if (robotMapInstance == null) {
      robotMapInstance = new ConfiguredRobotMap("/Map", getMapFile());
    }

    return robotMapInstance;
  }

  public RobotConfig loadRobotConfig() throws PersistentException {
    if (robotConfigInstance == null) {
      robotConfigInstance = new ConfiguredRobotConfig("/Config", getConfigFile());
    }

    return robotConfigInstance;
  }

  private String getConfigFile() {
    switch (determineBot()) {
      case Practice:
        return type.root + "practiceConfig.ini";
      case Laptop:
      case Competition:
      default:
        return type.root + "compConfig.ini";
    }
  }

  private String getMapFile() {
    switch (determineBot()) {
      case Practice:
        return type.root + "practiceMap.ini";
      case Laptop:
      case Competition:
      default:
        return type.root + "compMap.ini";
    }
  }

  private Bot determineBot() {
    if (type != null) return type;
    type = Bot.Laptop;

    File compBot = new File("/home/lvuser/THIS_IS_THE_COMP_BOT");
    if (compBot.exists()) {
      type = Bot.Competition;
    }
    File practiceBot = new File("/home/lvuser/THIS_IS_THE_PRACTICE_BOT");
    if (practiceBot.exists()) {
      type = Bot.Practice;
    }

    System.out.println(String.format("DETERMINED BOT: %s", type));

    return type;
  }

  private enum Bot {
    Competition("/home/lvuser/deploy/"),
    Practice("/home/lvuser/deploy/"),
    Laptop("Robot/src/main/deploy/");

    String root;

    Bot(String root) {
      this.root = root;
    }
  }
}
