package net.teamrush27.frc2019.base;

import net.teamrush27.frc2019.subsystems.impl.dto.ArmInput;
import net.teamrush27.frc2019.subsystems.impl.dto.DriveCommand;

public interface OperatorInterface {
  // DRIVER CONTROLS
  DriveCommand getTankCommand();

  boolean getShift();

  // ignore plz

  DriveCommand getCheesyDrive();

  // OPERATOR CONTROLS

  // gripper
  Boolean getWantManipulateHatch();

  Boolean getWantManipulateCargo();

  // arm
  Boolean wantsStow();

  Boolean wantsGroundPickup();

  Boolean getWantsCargoShip();

  Boolean wantsLevel1HumanLoad();

  Boolean wantsLevel2();

  Boolean wantsLevel3();

  Boolean getWantsInvert();

  Boolean wantsArmReset();

  // climb
  Boolean wantsPreClimb();

  Boolean wantsClimb();

  // open loop
  Double getWristInput();

  ArmInput getArmInput();

  Boolean wantsSwitchPipeline();

  Boolean wantsIncreaseOffset();

  Boolean wantsDecreaseOffset();

  Boolean wantsToggleLimelightSteering();

  void setRumble(double frac);

  Boolean wantsAutoStop();

  Boolean getWantStartAuton();

  Boolean clear();

  Boolean getWantUnjam();

  Boolean toggleDriveStyle();
}
