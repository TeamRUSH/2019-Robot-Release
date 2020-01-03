package net.teamrush27.frc2019.auto;

import net.teamrush27.frc2019.util.crash.CrashTrackingRunnable;

/** This class selects, runs, and stops (if necessary) a specified autonomous mode. */
public class AutoModeExecutor {
  private AutoModeBase m_auto_mode;
  private Thread m_thread = null;

  private boolean isActive = false;

  public void setAutoMode(AutoModeBase new_auto_mode) {
    m_auto_mode = new_auto_mode;
    m_thread =
        new Thread(
            new CrashTrackingRunnable() {
              @Override
              public void runCrashTracked() {
                if (m_auto_mode != null) {
                  m_auto_mode.run();
                }
              }
            });
  }

  public void start() {
    isActive = true;
    if (m_thread != null) {
      m_thread.start();
    }
  }

  public void stop() {
    isActive = false;
    if (m_auto_mode != null) {
      m_auto_mode.stop();
    }

    m_thread = null;
  }

  public AutoModeBase getAutoMode() {
    return m_auto_mode;
  }

  public boolean isActive() {
    return isActive && m_auto_mode.isActive();
  }
}
