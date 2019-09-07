package frc.robot.subsystems;

import frc.robot.loops.ILooper;

/**
 * The purpose of this class is to be able to define the basic properties of a Subsystem. As seen
 * below with the abstract class, a Subsystem should be able to do the following: Write to a Log for
 * further Analysis/Error Debugging, Have a Stop Routine after every match, Zeroing all Sensors
 * 
 * <p>
 * The Subsystem should only have one instance and should instantiate all of its member classes. All
 * Subsystems should work as a state machine constantly within two states of the actual state and
 * desired state.
 */
public abstract class Subsystem {
  public void writeToLog() {
  }

  public void writePeriodicOutputs() {
  }

  public void readPeriodicInputs() {
  }

  public void zeroSensors() {
  }

  public abstract void stop();

  public abstract void outputTelemetery();

  public void registerEnabledLooper(ILooper looper) {
  }

  public boolean hasEmergency = false;
}
