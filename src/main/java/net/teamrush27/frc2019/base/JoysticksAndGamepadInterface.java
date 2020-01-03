package net.teamrush27.frc2019.base;

import edu.wpi.first.wpilibj.GenericHID.Hand;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import net.teamrush27.frc2019.subsystems.impl.dto.ArmInput;
import net.teamrush27.frc2019.subsystems.impl.dto.DriveCommand;
import net.teamrush27.frc2019.util.math.KinematicsUtils;
import net.teamrush27.frc2019.util.math.Twist2d;
import net.teamrush27.frc2019.util.math.KinematicsUtils.DriveVelocity;
import net.teamrush27.frc2019.wrappers.APEMJoystick;
import net.teamrush27.frc2019.wrappers.PS4Controller;
import net.teamrush27.frc2019.wrappers.XboxController;

public class JoysticksAndGamepadInterface implements OperatorInterface {

  private static OperatorInterface INSTANCE = null;

  private final CheesyDriveHelper cheesyDriveHelper = new CheesyDriveHelper();

  public static OperatorInterface getInstance() {
    if (INSTANCE == null) {
      INSTANCE = new JoysticksAndGamepadInterface();
    }
    return INSTANCE;
  }

  private final APEMJoystick driverLeftJoystick;
  private final APEMJoystick driverRightJoystick;
  private final PS4Controller gamePad;
  private final XboxController cheesyController;

  public JoysticksAndGamepadInterface() {
    driverLeftJoystick = new APEMJoystick(0);
    driverRightJoystick = new APEMJoystick(1);
    gamePad = new PS4Controller(2);
    cheesyController = new XboxController(3);
  }

  @Override
  public DriveCommand getTankCommand() {
    double left = driverLeftJoystick.getY();
    double right = driverRightJoystick.getY();

    left = Math.abs(left) < 0.01 ? 0 : left;
    right = Math.abs(right) < 0.01 ? 0 : right;

    return new DriveCommand(left, right);
  }

  private boolean shiftLatch = false;

  @Override
  public boolean getShift() {
    if (!shiftLatch && driverRightJoystick.getZ() > 0.75) {
      shiftLatch = true;
      return true;
    } else if (driverRightJoystick.getZ() < 0.75) {
      shiftLatch = false;
    }
    return false;
  }

  private double deadband(double value, double min) {
    if (Math.abs(value) < Math.abs(min)) {
      return 0;
    }

    return value;
  }

  @Override
  public DriveCommand getCheesyDrive() {
    double throttle = -deadband(cheesyController.getY(Hand.kLeft), 0.1);
    double wheel = -deadband(cheesyController.getX(Hand.kRight), 0.1);
    boolean quickTurn = cheesyController.getBumper(Hand.kRight);

    final double kWheelGain = 0.05;
    final double kWheelNonlinearity = 0.05;
    final double denominator = Math.sin(Math.PI / 2.0 * kWheelNonlinearity);
    // Apply a sin function that's scaled to make it feel better.
    if (!quickTurn) {
        wheel = Math.sin(Math.PI / 2.0 * kWheelNonlinearity * wheel);
        wheel = Math.sin(Math.PI / 2.0 * kWheelNonlinearity * wheel);
        wheel = wheel / (denominator * denominator) * Math.abs(throttle);
    }

    wheel *= kWheelGain;
    DriveVelocity signal = KinematicsUtils.inverseKinematics(new Twist2d(throttle, 0.0, wheel));
    double scaling_factor = Math.max(1.0, Math.max(Math.abs(signal.leftVelocity), Math.abs(signal.rightVelocity)));
    return new DriveCommand(signal.leftVelocity / scaling_factor, signal.rightVelocity / scaling_factor);

    /*return cheesyDriveHelper.cheesyDrive(
        -cheesyController.getY(Hand.kLeft),
        cheesyController.getX(Hand.kRight),
        cheesyController.getBumper(Hand.kRight), false);*/
  }

  @Override
  public ArmInput getArmInput() {
    return new ArmInput(gamePad.getTriggerAxis(Hand.kLeft) - gamePad.getTriggerAxis(Hand.kRight), 0d);
  }

  @Override
  public Boolean getWantManipulateHatch() {
    return gamePad.getXButtonPressed();
  }

  @Override
  public Boolean getWantManipulateCargo() {
    return gamePad.getTriangleButtonPressed();
  }

  @Override
  public Boolean wantsStow() {
    return gamePad.getPadButton();
  }

  @Override
  public Boolean wantsGroundPickup() {
    return gamePad.getPOV() > 0 && Math.abs(gamePad.getPOV() - 270) <= 10;
  }

  @Override
  public Boolean getWantsCargoShip() {
    return gamePad.getBumper(Hand.kLeft);
  }

  @Override
  public Boolean wantsLevel1HumanLoad() {
    return gamePad.getPOV() > 0 && Math.abs(gamePad.getPOV() - 180) <= 10;
  }

  @Override
  public Boolean wantsLevel2() {
    return gamePad.getPOV() > 0 && Math.abs(gamePad.getPOV() - 90) <= 10;
  }

  @Override
  public Boolean wantsLevel3() {
    return gamePad.getPOV() == 0;
  }

  @Override
  public Boolean getWantsInvert() {
    return !gamePad.getBumper(Hand.kRight);
  }

  @Override
  public Boolean wantsArmReset() {
    return gamePad.getMiddleButton();
  }

  @Override
  public Double getWristInput() {
    return gamePad.getY(Hand.kRight);
  }

  @Override
  public Boolean wantsPreClimb() {
    return gamePad.getShareButton();
  }

  @Override
  public Boolean wantsClimb() {
    return gamePad.getOptionsButton() && gamePad.getShareButton();
  }

  @Override
  public Boolean wantsSwitchPipeline() {
    return false;
  }

  @Override
  public Boolean wantsIncreaseOffset() {
    return false;
  }

  @Override
  public Boolean wantsDecreaseOffset() {
    return false;
  }

  @Override
  public Boolean wantsToggleLimelightSteering() {
    return driverRightJoystick.getLeftButtonPressed() || cheesyController.getBumperPressed(Hand.kLeft);
  }

  @Override
  public void setRumble(double frac) {
    gamePad.setRumble(RumbleType.kLeftRumble, frac);
    gamePad.setRumble(RumbleType.kRightRumble, frac);
  }

  @Override
  public Boolean wantsAutoStop() {
    return driverLeftJoystick.getLeftButtonPressed() || driverLeftJoystick.getRightButtonPressed();
  }

  @Override
  public Boolean getWantStartAuton() {
    return false;
  }

  @Override
  public Boolean clear() {
    return driverLeftJoystick.getLeftButtonPressed()
        ^ driverLeftJoystick.getRightButtonPressed()
        ^ driverRightJoystick.getLeftButtonPressed()
        ^ driverRightJoystick.getRightButtonPressed()
        ^ gamePad.getXButtonPressed()
        ^ gamePad.getTriangleButtonPressed()
        ^ gamePad.getTriggerButtonPressed(Hand.kLeft)
        ^ gamePad.getTriggerButtonPressed(Hand.kRight)
        ^ gamePad.getPadButtonPressed()
        ^ gamePad.getMiddleButtonPressed()
        ^ gamePad.getBumperPressed(Hand.kLeft)
        ^ gamePad.getBumperPressed(Hand.kRight)
        ^ gamePad.getCircleButtonPressed()
        ^ gamePad.getSquareButtonPressed()
        ^ gamePad.getShareButtonPressed()
        ^ gamePad.getOptionsButtonPressed()
        ^ cheesyController.getBumperPressed(Hand.kLeft) 
        ^ cheesyController.getBumperPressed(Hand.kRight)
        ^ cheesyController.getStickButtonPressed(Hand.kLeft) 
        ^ cheesyController.getStickButtonPressed(Hand.kRight)
        ^ cheesyController.getAButtonPressed() 
        ^ cheesyController.getBButtonPressed() 
        ^ cheesyController.getXButtonPressed()
        ^ cheesyController.getYButtonPressed() 
        ^ cheesyController.getSelectButtonPressed() 
        ^ cheesyController.getStartButtonPressed();
  }

  @Override
  public Boolean getWantUnjam() {
    return gamePad.getCircleButton();
  }

  @Override
  public Boolean toggleDriveStyle() {
    return gamePad.getStickButtonPressed(Hand.kLeft);
  }
}