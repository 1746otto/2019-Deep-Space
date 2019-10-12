package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Ports;
import frc.robot.util.LazyTalonSRX;
import frc.robot.util.LazyVictorSPX;

public class Climber extends Subsystem {
  private static Climber instance;

  private LazyTalonSRX rightVacc;
  private LazyVictorSPX leftVacc;

  private LazyTalonSRX rightLift;
  private LazyVictorSPX leftLift;

  private ControlState state = ControlState.VACCOFF;

  private enum ControlState {
    VACCOFF, VACCON, SUCCED 
  }

  private ControlState getState() {
    return state;
  }

  private void setState(ControlState newState) {
    state = newState;
  }

  PeriodicIO periodicIO = new PeriodicIO();

  public static Climber getInstance() {
    if (instance == null) {
      instance = new Climber();
    }
    return instance;
  }

  private Climber() {
    rightVacc = new LazyTalonSRX(Ports.SUCK_RIGHT);
    leftVacc = new LazyVictorSPX(Ports.SUCK_LEFT);

    rightLift = new LazyTalonSRX(Ports.LIFT_RIGHT);
    leftLift = new LazyVictorSPX(Ports.LIFT_LEFT);

    leftVacc.follow(rightVacc);
    leftLift.follow(rightLift);
  }

  public void setOpenLoop(double climbInput) {
    periodicIO.climbControl = ControlMode.PercentOutput;
    periodicIO.climbInput = climbInput;
  }

  public void toggleVaccum() {
    if (getState() == ControlState.VACCOFF) {
      setState(ControlState.VACCON);
      rightVacc.set(ControlMode.PercentOutput, 1);
    } else {
      setState(ControlState.VACCOFF);
      rightVacc.set(ControlMode.PercentOutput, 0);
    }
  }

  @Override
  public void stop() {
    periodicIO.climbInput = 0;
  }

  @Override
  public void outputTelemetery() {
    SmartDashboard.putString("Vaccum State", getState().toString());    
  }


  @Override
  public void writePeriodicOutputs() {
    rightLift.set(periodicIO.climbControl, periodicIO.climbInput);
  }

  public static class PeriodicIO {
    public double climbInput;
    public ControlMode climbControl = ControlMode.PercentOutput;
  }
  
}