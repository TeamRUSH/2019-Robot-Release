package net.teamrush27.frc2019.auto.actions.impl;

import edu.wpi.first.wpilibj.Timer;
import java.util.List;
import net.teamrush27.frc2019.auto.actions.Action;
import net.teamrush27.frc2019.subsystems.impl.Drivetrain;
import net.teamrush27.frc2019.subsystems.impl.dto.DriveCommand;
import net.teamrush27.frc2019.util.ReflectingCSVWriter;
import net.teamrush27.frc2019.util.physics.DriveCharacterization;
import net.teamrush27.frc2019.util.physics.DriveCharacterization.VelocityDataPoint;

public class CollectVelocityData implements Action {
  private static final double kMaxPower = 0.25;
  private static final double kRampRate = 0.02;
  private static final Drivetrain mDrive = Drivetrain.getInstance();

  private final ReflectingCSVWriter<DriveCharacterization.VelocityDataPoint> mCSVWriter;
  private final List<DriveCharacterization.VelocityDataPoint> mVelocityData;
  private final boolean mTurn;
  private final boolean mReverse;

  private boolean isFinished = false;
  private double mStartTime = 0.0;

  /**
   * @param data reference to the list where data points should be stored
   * @param reverse if true drive in reverse, if false drive normally
   * @param turn if true turn, if false drive straight
   */
  public CollectVelocityData(List<VelocityDataPoint> data, boolean reverse, boolean turn) {
    mVelocityData = data;
    mReverse = reverse;
    mTurn = turn;
    mCSVWriter =
        new ReflectingCSVWriter<>(
            "/home/lvuser/VELOCITY_DATA.csv", DriveCharacterization.VelocityDataPoint.class);
  }

  @Override
  public void start() {
    mStartTime = Timer.getFPGATimestamp();
  }

  @Override
  public void update() {
    double percentPower = kRampRate * (Timer.getFPGATimestamp() - mStartTime);
    if (percentPower > kMaxPower) {
      isFinished = true;
      return;
    }
    mDrive.setOpenLoop(
        new DriveCommand(
            (mReverse ? -1.0 : 1.0) * percentPower,
            (mReverse ? -1.0 : 1.0) * (mTurn ? -1.0 : 1.0) * percentPower));
    mVelocityData.add(
        new DriveCharacterization.VelocityDataPoint(
            (Math.abs(mDrive.getLeftVelocityNativeUnits())
                    + Math.abs(mDrive.getRightVelocityNativeUnits()))
                / 4096.0
                * Math.PI
                * 10, // convert velocity to radians per second
            percentPower * 12.0 // convert to volts
            ));
    mCSVWriter.add(mVelocityData.get(mVelocityData.size() - 1));
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
