package frc.robot.util;

import edu.wpi.first.wpilibj.DoubleSolenoid;

public class LazyDoubleSolenoid extends DoubleSolenoid {
  protected Value previousValue = null;

  public LazyDoubleSolenoid(int forwardChannel, int reverseChannel) {
    super(forwardChannel, reverseChannel);
  }

  public Value getLastSet() {
    return previousValue;
  }
  
  @Override
  public void set(Value value) {
    if (value != previousValue) {
      previousValue = value;
      super.set(value);
    }
  }
  
}