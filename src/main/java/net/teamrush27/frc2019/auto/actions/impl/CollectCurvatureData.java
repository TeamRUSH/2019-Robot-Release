package net.teamrush27.frc2019.auto.actions.impl;

import edu.wpi.first.wpilibj.Timer;
import java.util.List;
import net.teamrush27.frc2019.auto.actions.Action;
import net.teamrush27.frc2019.base.RobotState;
import net.teamrush27.frc2019.subsystems.impl.Drivetrain;
import net.teamrush27.frc2019.subsystems.impl.dto.DriveCommand;
import net.teamrush27.frc2019.util.ReflectingCSVWriter;
import net.teamrush27.frc2019.util.physics.DriveCharacterization;
import net.teamrush27.frc2019.util.physics.DriveCharacterization.CurvatureDataPoint;

public class CollectCurvatureData implements Action {
  private static final double kMaxPower = 0.4;
  private static final double kStartPower = 0.2;
  private static final double kStartTime = 0.25;
  private static final double kRampRate = 0.02;
  private static final Drivetrain mDrive = Drivetrain.getInstance();
  private static final RobotState mRobotState = RobotState.getInstance();

  private final ReflectingCSVWriter<CurvatureDataPoint> mCSVWriter;
  private final List<CurvatureDataPoint> mCurvatureData;
  private final boolean mReverse;

  private boolean isFinished = false;
  private double mStartTime = 0.0;

  /**
   * @param data reference to the list where data points should be stored
   * @param reverse if true drive in reverse, if false drive normally
   */
  public CollectCurvatureData(
      List<DriveCharacterization.CurvatureDataPoint> data, boolean reverse) {
    mCurvatureData = data;
    mReverse = reverse;
    mCSVWriter =
        new ReflectingCSVWriter<>(
            "/home/lvuser/CURVATURE_DATA.csv", DriveCharacterization.CurvatureDataPoint.class);
  }

  @Override
  public void start() {
    mDrive.setOpenLoop(new DriveCommand(kStartPower, kStartPower));
    mStartTime = Timer.getFPGATimestamp();
  }

  @Override
  public void update() {
    double t = Timer.getFPGATimestamp() - mStartTime;
    if (t < kStartTime) { // give the robot some time to accelerate before recording data
      return;
    }
    double rightPower = kStartPower + (t - kStartTime) * kRampRate;
    if (rightPower > kMaxPower) {
      isFinished = true;
      return;
    }
    mDrive.setOpenLoop(
        new DriveCommand(
            (mReverse ? -1.0 : 1.0) * kStartPower, (mReverse ? -1.0 : 1.0) * rightPower));
    mCurvatureData.add(
        new DriveCharacterization.CurvatureDataPoint(
            mRobotState.getPredictedVelocity().deltaX,
            mRobotState.getPredictedVelocity().deltaTheta,
            kStartPower,
            rightPower));
    mCSVWriter.add(mCurvatureData.get(mCurvatureData.size() - 1));
  }

  @Override
  public boolean isFinished() {
    return isFinished;
  }

  @Override
  public void done() {
    mDrive.setOpenLoop(DriveCommand.BRAKE);
    mCSVWriter.flush();
  }
}
