package net.teamrush27.frc2019.constants;

public class DriveConstants {

  public static double PID_P = 0.0; // 1.0; //1.0;
  public static double PID_I = 0.0; // 0.002;
  public static double PID_D = 0.0; // 0.0; // 100.0;
  public static double PID_F = 0.0; // .3908; // 1023 / 2617.26
  public static int PID_I_ZONE = 700;
  public static double PID_RAMP_RATE = .100; // V/s: Seconds to full voltage

  public static double NOMINAL_OUTPUT = 0.5; // V

  // 13 f/s in in/s
  public static double MAX_VELOCITY = 13.0 * 12.0;

  // 5 f/s/s in in/s/s
  public static double MAX_ACCELERATION = 5.0 * 12.0;

  public static int MAX_CONTINUOUS_CURRENT = 18;

  public static int MAX_PEAK_CURRENT = 37;
  public static int PEAK_CURRENT_DURATION = 20;
}
