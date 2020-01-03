package net.teamrush27.frc2019.auto.actions.impl;

import edu.wpi.first.wpilibj.Timer;
import net.teamrush27.frc2019.auto.actions.Action;
import net.teamrush27.frc2019.managers.SuperstructureManager;
import net.teamrush27.frc2019.subsystems.impl.Arm;
import net.teamrush27.frc2019.subsystems.impl.Arm.ArmState;
import net.teamrush27.frc2019.util.math.MathUtils;

public class AutoSuperstructurePosition implements Action {
  private static final SuperstructureManager superman = SuperstructureManager.getInstance();
  private static final double kHeightEpsilon = 2.0;
  private static final double kAngleEpsilon = 5.0;
  private static final double kTimeout = 4.0;

  private final SuperstructureManager.WantedState wantedState;
  private final boolean invertedRotation;
  private final boolean hasHatch;
  private final boolean waitForCompletion;
  private final double angle;
  private final double extension;
  private double mStartTime;

  public AutoSuperstructurePosition(
      SuperstructureManager.WantedState wantedState, boolean invertedRotation, boolean hasHatch) {
    this(wantedState, invertedRotation, hasHatch, false, 0, 0);
  }

  public AutoSuperstructurePosition(
      SuperstructureManager.WantedState wantedState,
      boolean invertedRotation,
      boolean hasHatch,
      boolean waitForCompletion,
      double angle,
      double extension) {
    this.wantedState = wantedState;
    this.invertedRotation = invertedRotation;
    this.hasHatch = hasHatch;
    this.waitForCompletion = waitForCompletion;
    this.angle = angle;
    this.extension = extension;
  }

  @Override
  public void start() {
    superman.setWantedState(wantedState, invertedRotation, hasHatch);
    mStartTime = Timer.getFPGATimestamp();
  }

  @Override
  public boolean isFinished() {
    if (Timer.getFPGATimestamp() - mStartTime > kTimeout) {
      System.out.println("Auto Superstructure Position timed out!!!");
      return true;
    }
    if (waitForCompletion) {
      ArmState armState = Arm.getInstance().getArmState();
      return MathUtils.epsilonEquals(armState.getExtensionInInches(), extension, kHeightEpsilon)
          && MathUtils.epsilonEquals(armState.getRotationInDegrees(), angle, kAngleEpsilon);
    } else {
      return true;
    }
  }

  @Override
  public void update() {}

  @Override
  public void done() {
    System.out.println("Auto Superstructure Position action finished");
  }
}
