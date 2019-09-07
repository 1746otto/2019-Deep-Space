package frc.robot.loops;

import edu.wpi.first.wpilibj.Notifier;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants;
import frc.robot.util.CrashTrackingRunnable;

import java.util.ArrayList;
import java.util.List;

public class Looper implements ILooper {
  public final double kPeriod = Constants.kPeriod;

  private boolean running;

  private final Notifier notifier;
  private final List<Loop> loops;
  private final Object taskRunningLock = new Object();
  private double timestamp = 0;
  private double dt = 0.0;

  public double dt() {
    return dt;
  } 

  private final CrashTrackingRunnable runnable = new CrashTrackingRunnable() {
    @Override
    public void runCrashTracked() {
      synchronized (taskRunningLock) {
        if (running) {
          double now = Timer.getFPGATimestamp();

          for (Loop loop : loops) {
            loop.onLoop(now);
          }

          dt = now - timestamp;
          timestamp = now;
        }
      }
    }
  };

  public Looper() {
    notifier = new Notifier(runnable);
    running = false;
    loops = new ArrayList<>();
  }

  @Override
  public void register(Loop loop) {
    synchronized (taskRunningLock) {
      loops.add(loop);
    }
  }

  public synchronized void start() {
    if (!running) {
      System.out.println("Starting Loops...");
      synchronized (taskRunningLock) {
        timestamp = Timer.getFPGATimestamp();
        for (Loop loop : loops) {
          loop.onStart(timestamp);
        }
        running = true;
      }
      notifier.startPeriodic(kPeriod);
    }
  }

  public synchronized void stop() {
    if (running) {
      System.out.println("Stopping Loops...");
      notifier.stop();
      synchronized (taskRunningLock) {
        running = false;
        timestamp = Timer.getFPGATimestamp();
        for (Loop loop : loops) {
          loop.onStop(timestamp);
          System.out.println("Succesfully Stopped: " + loop);
        }
      }
    }
  }

  public void outputToSmartDashboard() {
    SmartDashboard.putNumber("looperDt", dt());
  }
  
}