package net.teamrush27.frc2019.wrappers;

import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;

public class Limelight {

  private NetworkTableInstance networkTableInstance;

  private String table;
  private NetworkTableEntry camModeEntry = null;
  private NetworkTableEntry ledModeEntry = null;

  private NetworkTableEntry txEntry = null;
  private NetworkTableEntry taEntry = null;
  private NetworkTableEntry thorEntry = null;

  public enum CamMode {
    DRIVE_MODE(1),
    TRACKING_MODE(0);

    private int value;

    CamMode(int value) {
      this.value = value;
    }

    public int getValue() {
      return value;
    }
  }

  public enum LEDMode {
    ON(0),
    OFF(1);

    private int value;

    LEDMode(int value) {
      this.value = value;
    }

    public int getValue() {
      return value;
    }
  }

  public Limelight(String table) {
    this.table = table;
    this.networkTableInstance = NetworkTableInstance.getDefault();
  }

  public void setCamMode(CamMode camMode) {
    if (camModeEntry == null) {
      camModeEntry = networkTableInstance.getTable(table).getEntry("camMode");
    }

    camModeEntry.setNumber(camMode.getValue());
  }

  public void setLedMode(LEDMode ledMode) {
    if (ledModeEntry == null) {
      ledModeEntry = networkTableInstance.getTable(table).getEntry("ledMode");
    }

    ledModeEntry.setNumber(ledMode.getValue());
  }

  public void configure(CamMode camMode, LEDMode ledMode) {
    setCamMode(camMode);
    setLedMode(ledMode);
  }

  public Double getTX() {
    if (txEntry == null) {
      txEntry = networkTableInstance.getTable(table).getEntry("tx");
    }

    return txEntry.getDouble(0d);
  }

  public double getTA() {
    if (taEntry == null) {
      taEntry = networkTableInstance.getTable(table).getEntry("ta");
    }

    return taEntry.getDouble(0d);
  }

  public double getTHOR() {
    if (thorEntry == null) {
      thorEntry = networkTableInstance.getTable(table).getEntry("thor");
    }

    return thorEntry.getDouble(0d);
  }
}
