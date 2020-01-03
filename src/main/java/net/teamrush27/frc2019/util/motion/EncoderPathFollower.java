package net.teamrush27.frc2019.util.motion;

// import jaci.pathfinder.Trajectory;

/**
 * The EncoderFollower is an object designed to follow a trajectory based on encoder input. This
 * class can be used for Tank or Swerve drive implementations.
 *
 * @author Jaci
 */
public class EncoderPathFollower {

  int encoderOffset, encoderTickCount;
  double wheelCircumference;

  double p, i, d, v, a;
  double lastError, heading;

  int segment;
  // Trajectory trajectory;

  /*public EncoderPathFollower(Trajectory traj) {
  	this.trajectory = traj;
  }*/

  public EncoderPathFollower() {}

  /** Set a new trajectory to follow, and reset the cumulative errors and segment counts */
  /*public void setTrajectory(Trajectory traj) {
  	this.trajectory = traj;
  	reset();
  }*/

  /**
   * Configure the PID/VA Variables for the Follower
   *
   * @param p The proportional term. This is usually quite high (0.8 - 1.0 are common values)
   * @param i The integral term. Currently unused.
   * @param d The derivative term. Adjust this if you are unhappy with the tracking of the follower.
   *     0.0 is the default
   * @param v The velocity ratio. This should be 1 over your maximum velocity @ 100% throttle. This
   *     converts m/s given by the algorithm to a scale of -1..1 to be used by your motor
   *     controllers
   * @param a The acceleration term. Adjust this if you want to reach higher or lower speeds faster.
   *     0.0 is the default
   */
  public void configurePIDVA(double p, double i, double d, double v, double a) {
    this.p = p;
    this.i = i;
    this.d = d;
    this.v = v;
    this.a = a;
  }

  /**
   * Configure the Encoders being used in the follower.
   *
   * @param initialPosition The initial 'offset' of your encoder. This should be set to the encoder
   *     value just before you start to track
   * @param ticksPerRevolution How many ticks per revolution the encoder has
   * @param wheelDiameter The diameter of your wheels (or pulleys for track systems) in meters
   */
  public void configureEncoder(int initialPosition, int ticksPerRevolution, double wheelDiameter) {
    encoderOffset = initialPosition;
    encoderTickCount = ticksPerRevolution;
    wheelCircumference = Math.PI * wheelDiameter;
  }

  /** Reset the follower to start again. Encoders must be reconfigured. */
  public void reset() {
    lastError = 0;
    segment = 0;
  }

  /**
   * Calculate the desired output for the motors, based on the amount of ticks the encoder has gone
   * through. This does not account for heading of the robot. To account for heading, add some extra
   * terms in your control loop for realignment based on gyroscope input and the desired heading
   * given by this object.
   *
   * @param encoderTicks The amount of ticks the encoder has currently measured.
   * @return The desired output for your motor controller
   */
  public double calculate(int encoderTicks) {
    // Number of Revolutions * Wheel Circumference
    /*double displacement = ((double)(encoderTicks - encoderOffset) / encoderTickCount) * wheelCircumference;
    if (segment < trajectory.length()) {
    	Trajectory.Segment seg = trajectory.get(segment);
    	double error = seg.position - displacement;
    	double calculatedValue =
    		p * error +                                    // Proportional
    			d * ((error - lastError) / seg.dt) +          // Derivative
    			(v * seg.velocity + a * seg.acceleration);    // V and A Terms
    	lastError = error;
    	heading = seg.heading;
    	segment++;

    	return calculatedValue;
    } else */ return 0;
  }

  /** @return the desired heading of the current point in the trajectory */
  public double getHeading() {
    return heading;
  }

  /** @return the current segment being operated on */
  /*public Trajectory.Segment getSegment() {
  	return trajectory.get(segment);
  }*/

  /** @return whether we have finished tracking this trajectory or not. */
  /*public boolean isFinished() {
  	return segment >= trajectory.length();
  }

  public Trajectory getTrajectory() {
  	return trajectory;
  }

  public double getLastError() {
  	return lastError;
  }*/
}
