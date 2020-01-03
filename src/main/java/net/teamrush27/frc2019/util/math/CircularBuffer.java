package net.teamrush27.frc2019.util.math;

import java.util.LinkedList;

/** Implements a simple circular buffer. */
public class CircularBuffer {
  @Override
  public String toString() {
    return "CircularBuffer [getAverage()="
        + getAverage()
        + ", getNumValues()="
        + getNumValues()
        + ", isFull()="
        + isFull()
        + "]";
  }

  int windowSize;
  LinkedList<Double> samples;
  double sum;

  public CircularBuffer(int window_size) {
    windowSize = window_size;
    samples = new LinkedList<Double>();
    sum = 0.0;
  }

  public void clear() {
    samples.clear();
    sum = 0.0;
  }

  public double getAverage() {
    if (samples.isEmpty()) return 0.0;
    return sum / samples.size();
  }

  public double getMax() {
    if (samples.isEmpty()) return 0.0;
    double max = Double.NEGATIVE_INFINITY;
    for (double sample : samples) {
      if (sample > max) {
        max = sample;
      }
    }

    return max;
  }

  public double getMin() {
    if (samples.isEmpty()) return 0.0;
    double min = Double.POSITIVE_INFINITY;
    for (double sample : samples) {
      if (sample < min) {
        min = sample;
      }
    }

    return min;
  }

  public void recomputeAverage() {
    // Reset any accumulation drift.
    sum = 0.0;
    if (samples.isEmpty()) return;
    for (Double val : samples) {
      sum += val;
    }
    sum /= windowSize;
  }

  public void addValue(double val) {
    samples.addLast(val);
    sum += val;
    if (samples.size() > windowSize) {
      sum -= samples.removeFirst();
    }
  }

  public int getNumValues() {
    return samples.size();
  }

  public boolean isFull() {
    return windowSize == samples.size();
  }
}
