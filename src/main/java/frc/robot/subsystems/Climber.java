package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Ports;
import frc.robot.util.LazyTalonSRX;
import frc.robot.util.LazyVictorSPX;

public class Climber extends Subsystem {
  private static Climber instance;

  private LazyVictorSPX rightVacc;
  private LazyTalonSRX leftVacc;

  private LazyTalonSRX rightLift;
  private LazyVictorSPX leftLift;

  private enum ControlState {
    VACCOFF, VACCON, SUCCED 
  }

  private ControlState state = ControlState.VACCOFF;


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
    rightVacc = new LazyVictorSPX(Ports.SUCK_RIGHT);
    leftVacc = new LazyTalonSRX(Ports.SUCK_LEFT);

    rightLift = new LazyTalonSRX(Ports.LIFT_RIGHT);
    leftLift = new LazyVictorSPX(Ports.LIFT_LEFT);

    rightVacc.follow(leftVacc);
    leftLift.follow(rightLift);
  }

  public void setOpenLoop(double climbInput) {
    periodicIO.climbControl = ControlMode.PercentOutput;
    periodicIO.climbInput = climbInput;
  }

  public void toggleVaccum() {
    if (getState() == ControlState.VACCOFF) {
      setState(ControlState.VACCON);
      leftVacc.set(ControlMode.PercentOutput, 1);
    } else {
      setState(ControlState.VACCOFF);
      leftVacc.set(ControlMode.PercentOutput, 0);
    }
  }

  @Override
  public void stop() {
    rightLift.set(ControlMode.PercentOutput, 0);
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