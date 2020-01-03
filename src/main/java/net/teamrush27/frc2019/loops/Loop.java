package net.teamrush27.frc2019.loops;

/**
 * Interface for loops (Routines that run periodically in the robot code)
 *
 * @author team254
 */
public interface Loop {
  public void onStart(double timestamp);

  public void onLoop(double timestamp);

  public void onStop(double timestamp);

  public String id();
}
