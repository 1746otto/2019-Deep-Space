package frc.robot.util;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

public class LazyTalonSRX extends TalonSRX {
  protected ControlMode lastControlMode = null;
  protected double lastSet = Double.NaN;

  public LazyTalonSRX(int deviceNumber) {
    super(deviceNumber);
  }

  public double getLastSet() {
    return lastSet;
  }

  @Override
  public void set(ControlMode mode, double value) {
    if (lastControlMode != mode || lastSet != value) {
      lastControlMode = mode;
      lastSet = value;
      super.set(mode, value);
    }
  }
}