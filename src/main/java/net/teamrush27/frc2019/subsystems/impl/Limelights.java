package net.teamrush27.frc2019.subsystems.impl;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import net.teamrush27.frc2019.loops.ILooper;
import net.teamrush27.frc2019.loops.Loop;
import net.teamrush27.frc2019.managers.SuperstructureManager;
import net.teamrush27.frc2019.subsystems.Subsystem;
import net.teamrush27.frc2019.subsystems.impl.dto.SmartDashboardCollection;
import net.teamrush27.frc2019.wrappers.Limelight;
import net.teamrush27.frc2019.wrappers.Limelight.CamMode;
import net.teamrush27.frc2019.wrappers.Limelight.LEDMode;
import org.apache.logging.log4j.LogManager;
import org.apache.logging.log4j.Logger;

public class Limelights extends Subsystem {

  private static final Logger LOG = LogManager.getLogger(Limelights.class);
  private static final String TAG = "LIMELIGHTS";

  private static Limelights INSTANCE = null;

  private SuperstructureManager superman = SuperstructureManager.getInstance();

  private Limelight front = new Limelight("limelight-front");
  private Limelight rear = new Limelight("limelight-rear");

  private PeriodicIO periodicIO = new PeriodicIO();

  public static Limelights getInstance() {
    if (INSTANCE == null) {
      INSTANCE = new Limelights();
    }

    return INSTANCE;
  }

  public enum SystemState {
    DRIVE,
    BOTH_TRACKING,
    REAR_TRACKING,
    FRONT_TRACKING
  }

  private SystemState systemState = SystemState.DRIVE;
  private boolean stateChanged = false;

  private final Loop loop =
      new Loop() {
        private SystemState lastState = null;

        @Override
        public void onStart(double timestamp) {}

        @Override
        public void onLoop(double timestamp) {
          systemState = defaultStateTransfer(superman.overBack());
          if (lastState != systemState) {
            LOG.info("LIMELIGHT state " + lastState + " to " + systemState);
            lastState = systemState;
            stateChanged = true;
          }
        }

        @Override
        public void onStop(double timestamp) {
          stop();
        }

        @Override
        public String id() {
          return TAG;
        }
      };

  private SystemState defaultStateTransfer(boolean rear) {
    switch (systemState) {
      case FRONT_TRACKING:
      case REAR_TRACKING:
        if (rear) {
          return SystemState.REAR_TRACKING;
        }
        return SystemState.FRONT_TRACKING;
      default:
        return systemState;
    }
  }

  public void defaultState() {
    synchronized (this) {
      systemState = SystemState.DRIVE;
      stateChanged = true;
    }
  }

  public void cycleDisabled() {
    synchronized (this) {
      switch (systemState) {
        case DRIVE:
          systemState = SystemState.BOTH_TRACKING;
          break;
        case BOTH_TRACKING:
        default:
          systemState = SystemState.DRIVE;
      }

      stateChanged = true;
    }
  }

  public void setTrackingEnabled(boolean trackingEnabled) {
    synchronized (systemState) {
      if (trackingEnabled) {
        systemState = SystemState.FRONT_TRACKING;
      } else {
        systemState = SystemState.DRIVE;
      }
    }
  }

  public void cycleEnabled() {
    synchronized (this) {
      switch (systemState) {
        case DRIVE:
          systemState = SystemState.FRONT_TRACKING;
          break;
        default:
          systemState = SystemState.DRIVE;
      }

      stateChanged = true;
    }
  }

  public void applyState() {
    switch (systemState) {
      case BOTH_TRACKING:
        front.configure(CamMode.TRACKING_MODE, LEDMode.ON);
        rear.configure(CamMode.TRACKING_MODE, LEDMode.ON);
        break;
      case REAR_TRACKING:
        front.configure(CamMode.DRIVE_MODE, LEDMode.OFF);
        rear.configure(CamMode.TRACKING_MODE, LEDMode.ON);
        break;
      case FRONT_TRACKING:
        front.configure(CamMode.TRACKING_MODE, LEDMode.ON);
        rear.configure(CamMode.DRIVE_MODE, LEDMode.OFF);
        break;
      case DRIVE:
      default:
        front.configure(CamMode.DRIVE_MODE, LEDMode.OFF);
        rear.configure(CamMode.DRIVE_MODE, LEDMode.OFF);
        break;
    }
  }

  public Double getOffset(boolean front) {
    return front ? periodicIO.front_tx : -periodicIO.rear_tx;
  }

  public Double getTargetArea(boolean front) {
    return front ? periodicIO.front_ta : periodicIO.rear_ta;
  }

  public Double getWidth(boolean front) {
    return front ? periodicIO.front_thor : periodicIO.rear_thor;
  }

  public SystemState getSystemState() {
    return systemState;
  }

  @Override
  public void readPeriodicInputs() {
    switch (systemState) {
      case FRONT_TRACKING:
        periodicIO.front_tx = front.getTX();
        periodicIO.front_ta = front.getTA();
        periodicIO.front_thor = front.getTHOR();
        periodicIO.rear_tx = 0d;
        periodicIO.rear_ta = 0d;
        periodicIO.rear_thor = 0d;
        break;
      case REAR_TRACKING:
        periodicIO.front_tx = 0d;
        periodicIO.front_ta = 0d;
        periodicIO.front_thor = 0d;
        periodicIO.rear_tx = rear.getTX();
        periodicIO.rear_ta = rear.getTA();
        periodicIO.rear_thor = rear.getTHOR();
        break;
      case BOTH_TRACKING:
        periodicIO.front_tx = front.getTX();
        periodicIO.front_ta = front.getTA();
        periodicIO.front_thor = front.getTHOR();
        periodicIO.rear_tx = rear.getTX();
        periodicIO.rear_ta = rear.getTA();
        periodicIO.rear_thor = rear.getTHOR();
        break;
      default:
        periodicIO.front_tx = 0d;
        periodicIO.front_ta = 0d;
        periodicIO.front_thor = 0d;
        periodicIO.rear_tx = 0d;
        periodicIO.rear_ta = 0d;
        periodicIO.rear_thor = 0d;
    }
  }

  @Override
  public void writePeriodicOutputs() {
    if (stateChanged) {
      applyState();
      stateChanged = false;
    }
  }

  @Override
  public void outputToSmartDashboard(SmartDashboardCollection collection) {
    // collection.setLimelightMode(systemState.toString());
    SmartDashboard.putString("LIMELIGHT_MODE", systemState.toString());
  }

  @Override
  public void stop() {}

  @Override
  public void zeroSensors() {}

  @Override
  public void registerEnabledLoops(ILooper enabledLooper) {
    enabledLooper.register(loop);
  }

  @Override
  public void test() {}

  @Override
  public String id() {
    return TAG;
  }

  public static class PeriodicIO {
    public Double front_tx;
    public Double rear_tx;

    public Double front_ta;
    public Double rear_ta;

    public Double front_thor;
    public Double rear_thor;
  }
}
