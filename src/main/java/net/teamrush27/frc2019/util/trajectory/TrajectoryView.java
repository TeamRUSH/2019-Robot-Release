package net.teamrush27.frc2019.util.trajectory;

import net.teamrush27.frc2019.util.math.State;

public interface TrajectoryView<S extends State<S>> {

  public TrajectorySamplePoint<S> sample(final double interpolant);

  public double first_interpolant();

  public double last_interpolant();

  public Trajectory<S> trajectory();
}
