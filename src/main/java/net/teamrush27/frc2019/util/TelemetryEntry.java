package net.teamrush27.frc2019.util;

import java.util.Comparator;

public class TelemetryEntry implements Comparable<TelemetryEntry> {

  private double timestamp;
  private String entryName;
  private String value;

  public TelemetryEntry(double timestamp, String entryName, String value) {
    this.timestamp = timestamp;
    this.entryName = entryName;
    this.value = value;
  }

  public double getTimestamp() {
    return timestamp;
  }

  public void setTimestamp(double timestamp) {
    this.timestamp = timestamp;
  }

  public String getEntryName() {
    return entryName;
  }

  public void setEntryName(String entryName) {
    this.entryName = entryName;
  }

  public String getValue() {
    return value;
  }

  public void setValue(String value) {
    this.value = value;
  }

  @Override
  public int compareTo(TelemetryEntry o) {
    return Comparator.comparing(TelemetryEntry::getTimestamp).compare(this, o);
  }
}
