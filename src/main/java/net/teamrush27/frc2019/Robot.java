/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package net.teamrush27.frc2019;

import com.google.gson.Gson;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.PowerDistributionPanel;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.livewindow.LiveWindow;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import java.io.IOException;
import net.teamrush27.frc2019.auto.AutoModeExecutor;
import net.teamrush27.frc2019.auto.creators.AutoModeSelector;
import net.teamrush27.frc2019.base.JoysticksAndGamepadInterface;
import net.teamrush27.frc2019.base.OperatorInterface;
import net.teamrush27.frc2019.constants.RobotConfiguration;
import net.teamrush27.frc2019.constants.RobotConfigurationFactory;
import net.teamrush27.frc2019.loops.Looper;
import net.teamrush27.frc2019.managers.SuperstructureManager;
import net.teamrush27.frc2019.managers.SuperstructureManager.WantedState;
import net.teamrush27.frc2019.subsystems.SubsystemManager;
import net.teamrush27.frc2019.subsystems.impl.Arm;
import net.teamrush27.frc2019.subsystems.impl.Drivetrain;
import net.teamrush27.frc2019.subsystems.impl.Gripper;
import net.teamrush27.frc2019.subsystems.impl.LED;
import net.teamrush27.frc2019.subsystems.impl.Limelights;
import net.teamrush27.frc2019.subsystems.impl.RobotStateEstimator;
import net.teamrush27.frc2019.subsystems.impl.SpiderLegs;
import net.teamrush27.frc2019.subsystems.impl.Wrist;
import net.teamrush27.frc2019.subsystems.impl.dto.DriveCommand;
import net.teamrush27.frc2019.subsystems.impl.dto.SmartDashboardCollection;
import net.teamrush27.frc2019.util.TelemetryUtil;
import net.teamrush27.frc2019.util.crash.CrashTracker;
import net.teamrush27.frc2019.util.trajectory.TrajectoryGenerator;
import org.apache.logging.log4j.LogManager;
import org.apache.logging.log4j.Logger;

public class Robot extends TimedRobot {

  public static PowerDistributionPanel pdp = new PowerDistributionPanel(0);

  public static final RobotConfiguration ROBOT_CONFIGURATION =
      RobotConfigurationFactory.getRobotConfiguration();

  private RobotStateEstimator robotStateEstimator = RobotStateEstimator.getInstance();
  private Drivetrain drivetrain = Drivetrain.getInstance();
  private Arm arm = Arm.getInstance();
  private Gripper gripper = Gripper.getInstance();
  private Wrist wrist = Wrist.getInstance();
  private SpiderLegs spiderLegs = SpiderLegs.getInstance();
  private OperatorInterface operatorInterface = JoysticksAndGamepadInterface.getInstance();
  private LED led = LED.getInstance();
  private Limelights limelights = Limelights.getInstance();
  private final SuperstructureManager superman = SuperstructureManager.getInstance();
  private final SubsystemManager subsystemManager =
      new SubsystemManager(
          drivetrain,
          gripper,
          spiderLegs,
          wrist,
          arm,
          led,
          limelights,
          superman,
          robotStateEstimator);

  private final Looper enabledLooper = new Looper();
  private final Looper disabledLooper = new Looper();

  private final Logger LOG = LogManager.getLogger(Robot.class);

  private AutoModeExecutor autoModeExecutor;

  boolean autoRan = false;

  private Gson serializer = new Gson();

  boolean cheesyDrive = false;

  @Override
  public void robotInit() {
    subsystemManager.registerEnabledLoops(enabledLooper);
    subsystemManager.registerDisabledLoops(disabledLooper);
    TrajectoryGenerator.getInstance().generateTrajectories();
    superman.zeroSensors();
    drivetrain.zeroSensors();

    limelights.defaultState();

    LiveWindow.disableAllTelemetry();
    autoModeExecutor = new AutoModeExecutor();
    AutoModeSelector.initAutoModeSelector();

    LOG.info("Defaulting to Tank Drive Control");
  }

  @Override
  public void robotPeriodic() {
    SmartDashboardCollection collection = new SmartDashboardCollection();
    //limelights.outputToSmartDashboard();
    subsystemManager.outputToSmartDashboard(collection);
    SmartDashboard.putString("robot.state", serializer.toJson(collection));
    // enabledLooper.outputToSmartDashboard();
    // LOG.info("rot: {} ext: {} wrist: {}", arm.getArmState().getRotationInDegrees(),
    //	arm.getArmState().getExtensionInInches(), wrist.getPWMAngle());
    // wrist.outputToSmartDashboard();

    // gripper.outputToSmartDashboard();
    SmartDashboard.putNumber("triggers", operatorInterface.getArmInput().getExtensionInput());
  }

  @Override
  public void autonomousInit() {
    DriverStation.reportError(
        String.format("I HATE THIS ROBOT %s", operatorInterface.getWantManipulateHatch()), false);
    led.setWantedState(LED.WantedState.ENABLED);
    disabledLooper.stop();
    // drivetrain.startLogging();
    //		subsystemManager.startLogging();
    // robotStateEstimator.startLogging();
    enabledLooper.start();

    // drivetrain.startLogging();
    //autoModeExecutor.setAutoMode(AutoModeSelector.getSelectedAutoMode());
    //autoModeExecutor.start();

    arm.setWantedState(Arm.WantedState.CLOSED_LOOP);
    spiderLegs.setWantedState(SpiderLegs.WantedState.OFF);
    gripper.setWantedState(Gripper.WantedState.INTAKE_HATCH);
    wrist.setWantedState(Wrist.WantedState.CLOSED_LOOP);

    superman.setWantedState(WantedState.STOW, true, false);
    superman.mustRecompute();

    autoRan = true;
  }

  @Override
  public void autonomousPeriodic() {
    /*
    if (operatorInterface.wantsAutoStop()) {
      autoModeExecutor.stop();

      drivetrain.stopLogging();

      drivetrain.shift(true);
      limelights.setTrackingEnabled(false);
      drivetrain.setLimelightSteering(SystemState.DRIVE);
      drivetrain.setOpenLoop(DriveCommand.defaultCommand());

      operatorInterface.clear();
    }

    if (!autoModeExecutor.isActive()) {
      driverControl();
    }

     */
    driverControl();
  }

  @Override
  public void teleopInit() {
    led.setWantedState(LED.WantedState.ENABLED);
    disabledLooper.stop();
    enabledLooper.start();

    arm.setWantedState(Arm.WantedState.CLOSED_LOOP);
    spiderLegs.setWantedState(SpiderLegs.WantedState.OFF);
    wrist.setWantedState(Wrist.WantedState.CLOSED_LOOP);
    limelights.setTrackingEnabled(false);
    drivetrain.setLimelightSteering(limelights.getSystemState());
    drivetrain.setBrakeMode(false);

    if (!autoRan) {
      //      gripper.setWantedState(Gripper.WantedState.OFF);
      superman.setWantedState(WantedState.STOW, true, false);
    }

    superman.mustRecompute();

    drivetrain.shift(true);
    drivetrain.setOpenLoop(DriveCommand.defaultCommand());

    //		subsystemManager.startLogging();

    autoModeExecutor = new AutoModeExecutor();
  }

  @Override
  public void teleopPeriodic() {
   /* if (autoModeExecutor.isActive() && operatorInterface.wantsAutoStop()) {
      double time = Timer.getFPGATimestamp();
      LOG.info("STOPPING AUTONOMOUS");
      autoModeExecutor.stop();

      drivetrain.shift(true);
      limelights.setTrackingEnabled(false);
      drivetrain.setLimelightSteering(SystemState.DRIVE);
      drivetrain.setOpenLoop(DriveCommand.defaultCommand());
      drivetrain.setBrakeMode(false);

      operatorInterface.clear();

      drivetrain.stopLogging();

      LOG.info("STOPPED AUTONOMOUS: {}", Timer.getFPGATimestamp() - time);

    } else if (!autoModeExecutor.isActive() && operatorInterface.getWantStartAuton()) {
      // drivetrain.startLogging();
      autoModeExecutor.setAutoMode(AutoModeSelector.getSelectedAutoMode());
      autoModeExecutor.start();
    } else if (!autoModeExecutor.isActive()) {
      drivetrain.setBrakeMode(false);
      driverControl();
    }*/

   driverControl();
  }

  @Override
  public void testInit() {
    enabledLooper.stop();
    disabledLooper.stop();
  }

  @Override
  public void testPeriodic() {}

  @Override
  public void disabledInit() {

    if (!autoRan) {
      // arm.reset();
      led.setWantedState(LED.WantedState.DISABLED);
    }
    drivetrain.stopLogging();
    //    gripper.zeroSensors();
    SmartDashboard.putString("Match Cycle", "DISABLED");
    try {
      TelemetryUtil.getInstance().writeToFile("/media/sda1/logs/telemetry.csv");
    } catch (IOException e) {
      LOG.error("could not write telemetry", e);
    }

    if (autoModeExecutor != null) {
      autoModeExecutor.stop();
    }

    try {
      CrashTracker.logDisabledInit();
      enabledLooper.stop();

      // drivetrain.startLogging();

      //			subsystemManager.stopLogging();
      drivetrain.setBrakeMode(false);
      drivetrain.stopLogging();
      // robotStateEstimator.stopLogging();

      //			Drivetrain.getInstance().zeroSensors();
      //			RobotState.getInstance().reset(Timer.getFPGATimestamp(), Pose2d.identity());

      disabledLooper.start();

    } catch (Throwable t) {
      CrashTracker.logThrowableCrash(t);
      throw t;
    }
  }

  @Override
  public void disabledPeriodic() {
    if (!autoRan) {
      spiderLegs.zeroSensors();
      arm.zeroSensors();
      wrist.zeroSensors();
    }

    if (operatorInterface.wantsToggleLimelightSteering()) {
      limelights.cycleDisabled();
      drivetrain.setLimelightSteering(limelights.getSystemState());
    }

    if (operatorInterface.toggleDriveStyle()) {
      cheesyDrive = !cheesyDrive;
      if (cheesyDrive) {
				LOG.info("Switching to Cheesy Drive Control");
				DriverStation.reportError("Switching to Cheesy Drive Control", false);
			} else {
				LOG.info("Switching to Tank Drive Control");
				DriverStation.reportError("Switching to Tank Drive Control", false);
			}
    }

    AutoModeSelector.update();
  }

  private void driverControl() {
    // bail everything if we're climbing
    if (operatorInterface.wantsPreClimb() && !operatorInterface.wantsClimb()) {
      spiderLegs.setWantedState(SpiderLegs.WantedState.PENDING_CLIMB);
      superman.setWantedState(SuperstructureManager.WantedState.CLIMB, true, false);
      drivetrain.setBrakeMode(true);
      drivetrain.shift(false);
    } else if (operatorInterface.wantsClimb()) {
      superman.setWantedState(SuperstructureManager.WantedState.CLIMB, true, false);
      if (operatorInterface.getWantsInvert()) {
        spiderLegs.setWantedState(SpiderLegs.WantedState.CLIMB);
      } else {
        spiderLegs.setWantedState(SpiderLegs.WantedState.CLIMB_L2);
      }
    } else {
      if (operatorInterface.wantsArmReset()) {
        drivetrain.fixArm();
        wrist.zeroSensors();
      } else if (operatorInterface.wantsStow()) {
        superman.setWantedState(
            WantedState.STOW, operatorInterface.getWantsInvert(), gripper.hasHatch());
      } else if (operatorInterface.wantsLevel1HumanLoad() && gripper.hasGamepiece()) {
        superman.setWantedState(
            WantedState.ROCKET_LEVEL_1, operatorInterface.getWantsInvert(), gripper.hasHatch());
      } else if (operatorInterface.wantsLevel1HumanLoad()) {
        superman.setWantedState(WantedState.HUMAN_LOAD, operatorInterface.getWantsInvert(), true);
      } else if (operatorInterface.wantsGroundPickup()) {
        superman.setWantedState(
            WantedState.CARGO_GROUND_PICKUP, operatorInterface.getWantsInvert(), false);
      } else if (operatorInterface.getWantsCargoShip()) {
        superman.setWantedState(
            WantedState.CARGO_SHIP, operatorInterface.getWantsInvert(), gripper.hasHatch());
      } else if (operatorInterface.wantsLevel2() && gripper.hasGamepiece()) {
        superman.setWantedState(
            WantedState.ROCKET_LEVEL_2, operatorInterface.getWantsInvert(), gripper.hasHatch());
      } else if (operatorInterface.wantsLevel2()) {
        superman.setWantedState(WantedState.HUMAN_LOAD, operatorInterface.getWantsInvert(), false);
      } else if (operatorInterface.wantsLevel3()) {
        superman.setWantedState(
            WantedState.ROCKET_LEVEL_3, operatorInterface.getWantsInvert(), gripper.hasHatch());
      }

      if (operatorInterface.getShift()) {
        drivetrain.shift();
      }

      if (WantedState.STOW.equals(superman.getWantedState()) && operatorInterface.getWantUnjam()) {
        gripper.unjam();
      }

      if ((superman.getHasHatch() || gripper.hasHatch())
          && operatorInterface.getWantManipulateHatch()) {
        gripper.transitionHatch();
      } else if (operatorInterface.getWantManipulateHatch()) {
        gripper.transitionCargo();
      }

      if (operatorInterface.wantsToggleLimelightSteering()) {
        limelights.cycleEnabled();
        drivetrain.setLimelightSteering(limelights.getSystemState());
      }

      double extensionInput = operatorInterface.getArmInput().getExtensionInput();

      if (Math.abs(extensionInput) > 0.05) {
        superman.addOffset(extensionInput * 0.2);
      }
    }

    if (spiderLegs.shouldDrive()) {
      if (spiderLegs.shouldHoldPosition()) {
        superman.setWantedState(WantedState.STOW, false, false);
        drivetrain.setOpenLoop(DriveCommand.BRAKE);
      } else {
        drivetrain.setOpenLoop(new DriveCommand(.3, .3, true));
      }
    } else {
      if (cheesyDrive) { //ROBOT_CONFIGURATION.isPracticeBot()) {
        drivetrain.setOpenLoop(operatorInterface.getCheesyDrive());
      } else {
        drivetrain.setOpenLoop(operatorInterface.getTankCommand());
      }
    }
  }
}
