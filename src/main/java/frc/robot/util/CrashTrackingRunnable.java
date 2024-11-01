package frc.robot.util;

public abstract class CrashTrackingRunnable implements Runnable {
  
  @Override
  public final void run() {
    try {
      runCrashTracked();
    } catch (Throwable t) {
      throw t;
    }
  }

  public abstract void runCrashTracked();
}