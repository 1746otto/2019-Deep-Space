package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import frc.robot.Constants;
import frc.robot.Ports;
import frc.robot.loops.ILooper;
import frc.robot.loops.Loop;
import frc.robot.subsystems.requests.Request;
import frc.robot.util.LazyTalonSRX;
import frc.robot.util.LazyVictorSPX;
import frc.robot.util.Util;

import java.util.Arrays;
import java.util.List;

public class CargoIntake extends Subsystem {
  private CargoIntake instance = null;

  private DigitalInput cargoSensor;

  private LazyVictorSPX intakeRight;
  private LazyVictorSPX intakeLeft;

  private LazyTalonSRX overBumper;
  private Solenoid overBumperSolenoid;

  List<LazyTalonSRX> masters;
  List<LazyVictorSPX> slaves;

  private Lift lift;

  public CargoIntake getInstance() {
    if (instance == null) {
      instance = new CargoIntake();
    } 
    return instance;
  }

  private CargoIntake() {
    lift = Lift.getInstance();

    cargoSensor = new DigitalInput(Ports.BALLS);

    overBumper = new LazyTalonSRX(Ports.OVER_BUMPER);

    intakeRight = new LazyVictorSPX(Ports.BALL_RIGHT);
    intakeLeft = new LazyVictorSPX(Ports.BALL_LEFT);

    masters = Arrays.asList(overBumper);
    slaves = Arrays.asList(intakeRight, intakeLeft);

    slaves.forEach(s -> s.set(ControlMode.Follower, Ports.OVER_BUMPER));

    overBumperSolenoid = new Solenoid(Ports.BALLENOID);
    overBumperSolenoid.set(true);
  }

  public enum IntakeState {
    NEUTRAL, INTAKING, HOLDING, EJECTING
  }

  public enum BumperState {
    RETRACTED, EXTENDED 
  }

  private IntakeState intakeState = IntakeState.NEUTRAL;
  private BumperState bumperState = BumperState.RETRACTED;

  public IntakeState getIntakeState() {
    return intakeState;
  }

  public BumperState getBumperState() {
    return bumperState;
  }



  public void setBumperState(BumperState state) {
    bumperState = state;
  }

  public boolean getCargoSensor() {
    return cargoSensor.get();
  }

  public void setIntakeState(IntakeState state) {
    intakeState = state;
  }

  public Request intakeStateRequest(double output) {
    return new Request() {
    
      @Override
      public void act() {
        setIntakeSpeed(output);
      }
    };
  }
  
  public void setCargoState(BumperState state) {
    bumperState = state;
  }

  public Request cargoStateRequest(BumperState state) {
    return new Request() {
    
      @Override
      public void act() {
        setCargoState(state);
      }
    };
  }

  private void setIntakeSpeed(double output) {
    if (Util.epsilonEquals(Math.signum(output), 1.0)) {
      setIntakeState(IntakeState.INTAKING);
    } else if (Util.epsilonEquals(Math.signum(output), -1.0)) {
      setIntakeState(IntakeState.EJECTING);
    } else {
      setIntakeState(IntakeState.NEUTRAL);
      overBumper.set(ControlMode.PercentOutput, 0.0);
    }
    overBumper.set(ControlMode.PercentOutput, output);
  }

  private final Loop loop = new Loop() {
  
    @Override
    public void onStart(double timestamp) {
      
    }
  
    @Override
    public void onLoop(double timestamp) {
      if (lift.getHeight() > Constants.kLiftCargoTolerance) {
        setBumperState(BumperState.RETRACTED);
      }
      if (cargoSensor.get()) {
        setIntakeState(IntakeState.HOLDING);
      }    
    }
  
    @Override
    public void onStop(double timestamp) {
      
    }
  };

  @Override
  public void registerEnabledLooper(ILooper looper) {
    looper.register(loop);
  }

  @Override
  public void writePeriodicOutputs() {
    switch (bumperState) {
      case RETRACTED:
        overBumperSolenoid.set(true);
        break;
      case EXTENDED:
        overBumperSolenoid.set(false);
        break;
      default:
        break;
    }
  }

  @Override
  public void stop() {
    overBumperSolenoid.set(false);
    overBumper.set(ControlMode.PercentOutput, 0.0);
  }

  @Override
  public void outputTelemetery() {
    if (Constants.kDebuggingOutput) {
      SmartDashboard.putString("Over the Bumper State", getBumperState().toString());
      SmartDashboard.putString("Cargo Intake State", getIntakeState().toString());
    }
  }

  
}