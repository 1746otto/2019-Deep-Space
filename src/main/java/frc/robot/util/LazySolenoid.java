package frc.robot.util;

import edu.wpi.first.wpilibj.Solenoid;

public class LazySolenoid extends Solenoid {
  protected boolean previousState = false;

  public LazySolenoid(int channel) {
    super(channel);
  }

  public boolean getLastState() {
    return previousState;
  }

  @Override
  public void set(boolean on) {
    if (on != previousState) {
      previousState = on;
      super.set(on);
    }
  }
  
}