package net.teamrush27.frc2019.auto.actions.impl;

import edu.wpi.first.wpilibj.Timer;
import net.teamrush27.frc2019.auto.actions.Action;
import net.teamrush27.frc2019.subsystems.impl.Drivetrain;
import net.teamrush27.frc2019.subsystems.impl.dto.DriveCommand;

public class OpenLoopDrive implements Action {
  private static final Drivetrain drivetrain = Drivetrain.getInstance();

  private double mStartTime;
  private final double mDuration, mLeft, mRight;
  private final boolean mFinishWhenSeesCube;

  public OpenLoopDrive(double left, double right, double duration, boolean finishWhenSeesCube) {
    mDuration = duration;
    mLeft = left;
    mRight = right;
    mFinishWhenSeesCube = finishWhenSeesCube;
  }

  @Override
  public boolean isFinished() {
    return Timer.getFPGATimestamp() - mStartTime > mDuration || mFinishWhenSeesCube;
  }

  @Override
  public void update() {
    System.out.println((Timer.getFPGATimestamp() - mStartTime) + " > " + mDuration);
  }

  @Override
  public void done() {
    drivetrain.setOpenLoop(new DriveCommand(0.0, 0.0));
  }

  @Override
  public void start() {
    drivetrain.setOpenLoop(new DriveCommand(mLeft, mRight));
    mStartTime = Timer.getFPGATimestamp();
  }
}
