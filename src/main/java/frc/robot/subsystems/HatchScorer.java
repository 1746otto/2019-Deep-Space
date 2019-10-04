package frc.robot.subsystems;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants;
import frc.robot.Ports;
import frc.robot.subsystems.requests.Request;
import frc.robot.util.LazyDoubleSolenoid;

public class HatchScorer extends Subsystem {
  private static HatchScorer instance = null;

  private LazyDoubleSolenoid stowSolenoid;
  private LazyDoubleSolenoid intakeSolenoid;

  private DigitalInput hatchSensor;

  public static HatchScorer getInstance() {
    if (instance == null) {
      instance = new HatchScorer();
    }
    return instance;
  }

  public static enum GrabState {
    HOLDING, INTAKING
  }

  public static enum StowState {
    STOWED, UNSTOWED
  }

  public void setStowState(StowState state) {
    stowState = state;
  }

  public void setGrabState(GrabState state) {
    grabState = state;
  }

  public StowState getStowState() {
    return stowState;
  }

  public GrabState getGrabState() {
    return grabState;
  }

  private GrabState grabState = GrabState.HOLDING;
  private StowState stowState = StowState.STOWED;

  private HatchScorer() {
    hatchSensor = new DigitalInput(Ports.HATCH1);

    stowSolenoid = new LazyDoubleSolenoid(Ports.HATCHENOID11, Ports.HATCHENOID12);
    intakeSolenoid = new LazyDoubleSolenoid(Ports.HATCHENOID21, Ports.HATCHENOID22);

    stowSolenoid.set(Value.kOff);
    intakeSolenoid.set(Value.kOff);
    
  }

  public Request stowRequest(StowState state) {
    return new Request() {
    
      @Override
      public void act() {
        setStowState(state);
      }
    };
  }

  public Request intakeRequest(GrabState state) {
    return new Request() {
    
      @Override
      public void act() {
        setGrabState(state);
      }
    };
  }

  public void toggleIntakeState() {
    if (getGrabState() == GrabState.HOLDING) {
      setGrabState(GrabState.INTAKING);
    } else if (getGrabState() == GrabState.INTAKING) {
      setGrabState(GrabState.HOLDING);
    }
  }

  public void toggleStowState() {
    if (getStowState() == StowState.STOWED) {
      setStowState(StowState.UNSTOWED);
    } else if (getStowState() == StowState.UNSTOWED) {
      setStowState(StowState.STOWED);
    }
  }
  
 
  public boolean getHatchSensor() {
    return hatchSensor.get();
  }


  @Override
  public void writePeriodicOutputs() {
    switch (grabState) {
      case HOLDING:
        intakeSolenoid.set(Value.kForward);
        break;
      case INTAKING:
        intakeSolenoid.set(Value.kReverse);
        break;
      default:
        hasEmergency = true;
    }
    switch (stowState) {
      case STOWED:
        stowSolenoid.set(Value.kForward);
        break;
      case UNSTOWED:
        stowSolenoid.set(Value.kReverse);
        break;
      default:
        hasEmergency = true;
    }
  }


  @Override
  public void stop() {
    // TODO Auto-generated method stub

  }

  @Override
  public void outputTelemetery() {
    if (Constants.kDebuggingOutput) {
      SmartDashboard.putString("Hatch Intake State", getGrabState().toString());
      SmartDashboard.putString("Four Bar State", getStowState().toString());
    }
  }
  
}