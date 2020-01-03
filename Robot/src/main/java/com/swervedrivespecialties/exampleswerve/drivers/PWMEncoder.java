package com.swervedrivespecialties.exampleswerve.drivers;

import edu.wpi.first.wpilibj.Counter;
import edu.wpi.first.wpilibj.DigitalInput;

public class PWMEncoder {
  private Counter high = new Counter(Counter.Mode.kSemiperiod);
  private Counter low = new Counter(Counter.Mode.kSemiperiod);

  public PWMEncoder(int dioPort) {
    DigitalInput in = new DigitalInput(dioPort);
    high.setSemiPeriodMode(true);
    low.setSemiPeriodMode(false);

    high.setUpSource(in);
    low.setUpSource(in);
  }

  private double[] readInputs() {
    return new double[] {high.getPeriod(), low.getPeriod()};
  }

  private double sum(double[] ds) {
    return ds[0] + ds[1];
  }

  public double duty() {
    return highDuty();
  }

  public double highDuty() {
    double[] inputs = readInputs();
    return inputs[0] / sum(inputs);
  }

  public double lowDuty() {
    double[] inputs = readInputs();
    return inputs[1] / sum(inputs);
  }

  public double period() {
    double[] inputs = readInputs();
    return sum(inputs);
  }
}
