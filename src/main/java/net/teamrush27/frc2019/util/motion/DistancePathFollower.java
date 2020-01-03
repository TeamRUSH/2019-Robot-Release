package net.teamrush27.frc2019.util.motion;

// import jaci.pathfinder.Pathfinder;
// import jaci.pathfinder.Trajectory;
import java.io.File;
import java.io.FileWriter;
import java.io.IOException;

// import net.teamrush27.frc2019.util.interpolate.InterpolatingSegment;
// import net.teamrush27.frc2019.util.interpolate.InterpolatingTrajectory;

public class DistancePathFollower {
  private double p, i, d, v, a;
  private double lastError, heading;

  private int segment;
  // private InterpolatingTrajectory trajectory;

  private final FileWriter fileWriter;

  private boolean finished;

  private boolean within_tolerance;

  /*public DistancePathFollower(InterpolatingTrajectory traj) {
  	this.trajectory = traj;
  	try {
  		fileWriter = new FileWriter(new File("/tmp/pathDebug.csv"));
  		fileWriter.append("t,p,d,v,a,err,err-lasterr/dt,pvel,pacc,result\n");
  	} catch (IOException e) {
  		throw new RuntimeException("could not open file");
  	}

  }*/

  public DistancePathFollower(boolean thing) {
    try {
      fileWriter = new FileWriter(new File("/tmp/pathDebug" + (thing ? "Left" : "Right") + ".csv"));
      fileWriter.append("t,p,d,v,a,err,err-lasterr/dt,pvel,pacc,result\n");
    } catch (IOException e) {
      throw new RuntimeException("could not open file");
    }
  }

  /** Set a new trajectory to follow, and reset the cumulative errors and segment counts */
  /*public void setTrajectory(InterpolatingTrajectory traj) {
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

  /** Reset the follower to start again. Encoders must be reconfigured. */
  public void reset() {
    lastError = 0;
    segment = 0;
    within_tolerance = false;
  }

  private int getLatestSegment(double distanceCovered) {
    /*Trajectory.Segment seg;

    //new segment between current segment and final segment
    for(int i = segment + 1; i < trajectory.length(); i++) {
    	seg = trajectory.get(i);
    	if (seg.position > distanceCovered) {
    		return Math.min(i, trajectory.length() - 1);
    	}
    }
    return trajectory.length();*/
    return 0;
  }

  /**
   * Calculate the desired output for the motors, based on the distance the robot has covered. This
   * does not account for heading of the robot. To account for heading, add some extra terms in your
   * control loop for realignment based on gyroscope input and the desired heading given by this
   * object.
   *
   * @param distance The time passed in seconds
   * @return The desired output for your motor controller
   */
  public double calculate(final double distance, final double velocity) {
    /*final int lastSegment = segment;
          segment = getLatestSegment(distance);

          int last = trajectory.length() - 1;
          within_tolerance = trajectory.get(last).position - distance < TrackingConstants.GOAL_POSITION_TOLERANCE;

    if (segment < trajectory.length()) {
    	final InterpolatingSegment seg = trajectory.getSegmentByFieldBetween(
    			InterpolatingTrajectory.Field.POSITION,
    			lastSegment,
    			segment,
    			distance);

    	heading = seg.heading;

    	double latestVelError = seg.velocity - velocity;

    	System.out.println(
    		String.format(
    			"2\t%s\t%s\t%s\t%s\t%s\t%s\t%s",
    			distance,
    			velocity,
    			latestVelError,
    			Pathfinder.r2d(heading),
    			seg.position,
    			seg.velocity,
    			seg.acceleration));

    	return FollowingConstants.P * seg.position + FollowingConstants.V * latestVelError + FollowingConstants.FEED_FORWARD_VELOCITY * seg.velocity
    		+ (Double.isNaN(seg.acceleration) ? 0.0 : FollowingConstants.FEED_FORWARD_ACCELERATION * seg.acceleration);
    } else {
    	return 0;
    }*/
    return 0;
  }

  /** @return the desired heading of the current point in the trajectory */
  public double getHeading() {
    return heading;
  }

  /** @return the current segment being operated on */
  public Object getSegment() { // Trajectory.Segment getSegment() {
    return null; // return trajectory.get(segment);
  }

  /** @return whether we have finished tracking this trajectory or not. */
  public boolean isFinished() {
    //	    System.out.println(String.format("Segment: %s, Length: %s", segment,
    // trajectory.length()));
    if (within_tolerance || segment >= 1000) { // trajectory.length()) {
      try {
        fileWriter.flush();
      } catch (IOException e) {
        e.printStackTrace();
      }
      return true;
    }
    return false;
  }

  public Object getTrajectory() {
    return null; // trajectory;
  }

  public int getSegmentNumber() {
    return segment;
  }

  public double getLastError() {
    return lastError;
  }
}
