package net.teamrush27.frc2019.util.math;

import net.teamrush27.frc2019.util.CSVWritable;
import net.teamrush27.frc2019.util.interpolate.Interpolable;

public interface State<S> extends Interpolable<S>, CSVWritable {
  double distance(final S other);

  boolean equals(final Object other);

  String toString();

  String toCSV();
}
