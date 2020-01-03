package net.teamrush27.frc2019.auto.actions.impl;

import edu.wpi.first.wpilibj.Timer;
import net.teamrush27.frc2019.auto.actions.Action;
import net.teamrush27.frc2019.subsystems.impl.Drivetrain;
import net.teamrush27.frc2019.subsystems.impl.dto.DriveCommand;

public class ShiftHighGearAction implements Action {
  private static final double kTime = 2.0;
  private static final double kPower = 0.5;
  private static final Drivetrain mDrive = Drivetrain.getInstance();

  private final boolean mReverse;

  private double mStartTime = 0.0;

  public ShiftHighGearAction(boolean reverse) {
    mReverse = reverse;
  }

  @Override
  public void start() {
    mDrive.shift(true);
    mDrive.setOpenLoop(
        new DriveCommand((mReverse ? -1.0 : 1.0) * kPower, (mReverse ? -1.0 : 1.0) * kPower));
    mStartTime = Timer.getFPGATimestamp();
  }

  @Override
  public void update() {}

  @Override
  public boolean isFinished() {
    return Timer.getFPGATimestamp() - mStartTime > kTime;
  }

  @Override
  public void done() {
    mDrive.setOpenLoop(DriveCommand.BRAKE);
  }
}
