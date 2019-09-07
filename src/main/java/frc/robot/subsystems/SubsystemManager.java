package frc.robot.subsystems;

import frc.robot.loops.ILooper;
import frc.robot.loops.Loop;
import java.util.ArrayList;
import java.util.List;

/**
 * If all of the subsystems of the robot needs to be updated at once. Use this class to send out a
 * mass update (e.g. start, stop reset)
 */
public class SubsystemManager implements ILooper {

  private final List<Subsystem> mAllSubsystems;
  private final List<Loop> mLoops = new ArrayList<>();

  public List<Subsystem> getSubsystem() {
    return mAllSubsystems;
  }

  public SubsystemManager(List<Subsystem> subsystems) {
    mAllSubsystems = subsystems;
  }

  public void outputToSmartDashboard() {
    mAllSubsystems.forEach(s -> s.outputTelemetery());
  }

  public void stop() {
    mAllSubsystems.forEach(s -> s.stop());
  }

  /**
   * Iterates through every declared subsystem whether there is an emergency.
   * 
   * @return Emergency State
   */

  public boolean haveEmergency() {
    boolean emergency = false;
    for (Subsystem s : mAllSubsystems) {
      emergency |= s.hasEmergency;
    }
    return emergency;
  }

  private class enabledLoop implements Loop {

    @Override
    public void onStart(double timestamp) {
      for (Loop l : mLoops) {
        l.onStart(timestamp);
      }
    }

    @Override
    public void onLoop(double timestamp) {
      for (Subsystem s : mAllSubsystems) {
        s.readPeriodicInputs();
      }
      for (Loop l : mLoops) {
        l.onLoop(timestamp);
      }
      for (Subsystem s : mAllSubsystems) {
        s.writePeriodicOutputs();
      }
    }

    @Override
    public void onStop(double timestamp) {
      for (Loop l : mLoops) {
        l.onStop(timestamp);
      }
    }

  }

  private class disabledLoop implements Loop {

    @Override
    public void onStart(double timestamp) {

    }

    @Override
    public void onLoop(double timestamp) {
      for (Subsystem s : mAllSubsystems) {
        s.readPeriodicInputs();
      }
      for (Subsystem s : mAllSubsystems) {
        s.writePeriodicOutputs();
      }
    }

    @Override
    public void onStop(double timestamp) {

    }

  }

  public void registerEnabledLoops(ILooper looper) {
    mAllSubsystems.forEach(s -> s.registerEnabledLooper(this));
    looper.register(new enabledLoop());
  }

  public void registerDisabledLooper(ILooper looper) {
    looper.register(new disabledLoop());
  }

  @Override
  public void register(Loop loop) {
    mLoops.add(loop);
  }
}
