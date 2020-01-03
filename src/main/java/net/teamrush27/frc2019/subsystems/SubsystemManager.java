package net.teamrush27.frc2019.subsystems;

import edu.wpi.first.wpilibj.Timer;
import java.util.ArrayList;
import java.util.Collections;
import java.util.HashSet;
import java.util.List;
import java.util.Set;
import java.util.stream.Collectors;
import net.teamrush27.frc2019.loops.ILooper;
import net.teamrush27.frc2019.loops.Loop;
import net.teamrush27.frc2019.loops.Looper;
import net.teamrush27.frc2019.subsystems.impl.dto.SmartDashboardCollection;
import net.teamrush27.frc2019.util.ReflectingCSVWriter;

/** Used to reset, start, stop, and update all subsystems at once */
public class SubsystemManager implements ILooper {

  public static String TAG = "SUBSYSTEMMANAGER";

  private final Set<Subsystem> subsystems = new HashSet<Subsystem>();
  private List<Loop> loops = new ArrayList<>();

  private ReflectingCSVWriter CSVWriter;

  public SubsystemManager(Subsystem... subsystems) {
    Collections.addAll(this.subsystems, subsystems);
  }

  public SubsystemManager(List subsystems) {}

  public void outputToSmartDashboard(SmartDashboardCollection collection) {
    subsystems.forEach(s -> s.outputToSmartDashboard(collection));
  }

  public void stop() {
    subsystems.forEach(s -> s.stop());
  }

  public void zeroSensors() {
    subsystems.forEach(s -> s.zeroSensors());
  }

  public void registerEnabledLoops(Looper enabledLooper) {
    subsystems.forEach(s -> s.registerEnabledLoops(this));
    enabledLooper.register(new EnabledLoop());
  }

  public void registerDisabledLoops(Looper disabledLooper) {
    disabledLooper.register(new DisabledLoop());
  }

  public void test() {
    for (Subsystem subsystem : subsystems) {
      subsystem.test();
    }
  }

  @Override
  public void register(Loop loop) {
    loops.add(loop);
  }

  private class EnabledLoop implements Loop {

    @Override
    public void onStart(double timestamp) {
      List profiles =
          loops.stream()
              .map(
                  l -> {
                    double start = Timer.getFPGATimestamp();
                    l.onStart(start);
                    double end = Timer.getFPGATimestamp();

                    return new Profile(l.id(), "START", start, end - start);
                  })
              .collect(Collectors.toList());

      double end = Timer.getFPGATimestamp();
      profiles.add(new Profile(this.id(), "START", timestamp, end - timestamp));

      if (CSVWriter != null) {
        profiles.forEach(p -> CSVWriter.add(p));
      }
    }

    @Override
    public void onLoop(double timestamp) {
      List readProfiles = readSubsystemInputs(timestamp);

      double loopStart = Timer.getFPGATimestamp();

      List loopProfiles =
          loops.stream()
              .map(
                  l -> {
                    double start = Timer.getFPGATimestamp();
                    l.onLoop(start);
                    double end = Timer.getFPGATimestamp();

                    return new Profile(l.id(), "LOOP", start, end - start);
                  })
              .collect(Collectors.toList());

      double end = Timer.getFPGATimestamp();
      loopProfiles.add(new Profile(this.id(), "LOOP", loopStart, end - loopStart));

      List writeProfiles = writeSubsystemOutputs(Timer.getFPGATimestamp());

      if (CSVWriter != null) {
        readProfiles.forEach(p -> CSVWriter.add(p));
        loopProfiles.forEach(p -> CSVWriter.add(p));
        writeProfiles.forEach(p -> CSVWriter.add(p));
      }
    }

    @Override
    public void onStop(double timestamp) {

      List profiles =
          loops.stream()
              .map(
                  l -> {
                    double start = Timer.getFPGATimestamp();
                    l.onStop(start);
                    double end = Timer.getFPGATimestamp();

                    return new Profile(l.id(), "STOP", start, end - start);
                  })
              .collect(Collectors.toList());

      double end = Timer.getFPGATimestamp();
      profiles.add(new Profile(this.id(), "STOP", timestamp, end - timestamp));

      if (CSVWriter != null) {
        profiles.forEach(p -> CSVWriter.add(p));
      }
    }

    @Override
    public String id() {
      return TAG;
    }
  }

  private class DisabledLoop implements Loop {

    @Override
    public void onStart(double timestamp) {}

    @Override
    public void onLoop(double timestamp) {
      readSubsystemInputs(timestamp);
      writeSubsystemOutputs(Timer.getFPGATimestamp());
    }

    @Override
    public void onStop(double timestamp) {}

    @Override
    public String id() {
      return TAG;
    }
  }

  public List readSubsystemInputs(double timestamp) {
    List profiles =
        subsystems.stream()
            .map(
                s -> {
                  double start = Timer.getFPGATimestamp();
                  s.readPeriodicInputs();
                  double end = Timer.getFPGATimestamp();

                  return new Profile(s.id(), "READ_INPUT", start, end - start);
                })
            .collect(Collectors.toList());

    double end = Timer.getFPGATimestamp();
    profiles.add(new Profile(this.id(), "READ_INPUT", timestamp, end - timestamp));

    return profiles;
  }

  public List writeSubsystemOutputs(double timestamp) {
    List profiles =
        subsystems.stream()
            .map(
                s -> {
                  double start = Timer.getFPGATimestamp();
                  s.writePeriodicOutputs();
                  double end = Timer.getFPGATimestamp();

                  return new Profile(s.id(), "WRITE_OUTPUT", start, end - start);
                })
            .collect(Collectors.toList());

    double end = Timer.getFPGATimestamp();
    profiles.add(new Profile(this.id(), "WRITE_OUTPUT", timestamp, end - timestamp));

    return profiles;
  }

  public void startLogging() {
    if (CSVWriter == null) {
      CSVWriter = new ReflectingCSVWriter<>("/home/lvuser/PROFILE-LOGS.csv", Profile.class);
    }
  }

  public void stopLogging() {
    if (CSVWriter != null) {
      CSVWriter.flush();
      CSVWriter = null;
    }
  }

  public String id() {
    return TAG;
  }

  public static class Profile {

    public String id;
    public String action;
    public double timestamp;
    public double duration;

    public Profile(String id, String action, double timestamp, double duration) {
      this.id = id;
      this.action = action;
      this.timestamp = timestamp;
      this.duration = duration;
    }

    @Override
    public String toString() {
      return id + ", " + action + ", " + timestamp + ", " + duration;
    }
  }
}
