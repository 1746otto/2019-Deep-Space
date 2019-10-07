package frc.robot.subsystems;

import java.util.Arrays;
import java.util.List;
import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.StatusFrameEnhanced;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants;
import frc.robot.Ports;
import frc.robot.loops.ILooper;
import frc.robot.loops.Loop;
import frc.robot.subsystems.requests.Prerequisite;
import frc.robot.subsystems.requests.Request;
import frc.robot.util.LazyTalonSRX;
import frc.robot.util.Util;

public class Lift extends Subsystem {
  private static Lift instance = null;

  public static Lift getInstance() {
    if (instance == null) {
      instance = new Lift();
    }
    return instance;
  }

  LazyTalonSRX master;
  LazyTalonSRX slave;
  List<LazyTalonSRX> motors;
  List<LazyTalonSRX> slaves;
  
  private double targetHeight = 0.0;

  public double getTargetHeight() {
    return targetHeight;
  }

  private boolean configForAscent = true;
  private boolean limitsEnabled = false;

  public boolean limitsEnabled() {
    return limitsEnabled;
  }

  public enum ControlState {
    Neutal, Position, OpenLoop, Locked
  }

  private ControlState state = ControlState.Neutal;

  public ControlState getState() {
    return state;
  }

  public void setState(ControlState newState) {
    state = newState;
  }

  double manualSpeed = Constants.kLiftTeleopManualSpeed;

  public void setManualSpeed(double speed) {
    manualSpeed = speed;
  }

  PeriodicIO periodicIO = new PeriodicIO();

  private Lift() {
    master = new LazyTalonSRX(Ports.ELEVATOR_TALON);
    slave = new LazyTalonSRX(Ports.ELEVATOR_VICTOR);

    motors = Arrays.asList(master, slave);
    slaves = Arrays.asList(slave);
    
    slaves.forEach((s) -> s.follow(master));
    
    for (LazyTalonSRX motor : motors) {
      motor.configVoltageCompSaturation(12.0, 10);
      motor.enableVoltageCompensation(true);
      motor.setNeutralMode(NeutralMode.Brake);
    }
    
    master.configSelectedFeedbackSensor(FeedbackDevice.QuadEncoder, 0, 10);
    master.setStatusFramePeriod(StatusFrameEnhanced.Status_13_Base_PIDF0, 10);
    // master.configForwardSoftLimitEnable(true, 10);
    // master.configReverseSoftLimitEnable(true, 10);
    // enableLimits(false);

    setCurrentLimit(Constants.kLiftCurrentLimit);

    resetToAbsolutePosition();
    configForAscent();
  }

  private void configForAscent() {
    manualSpeed = Constants.kLiftTeleopManualSpeed;

    master.config_kP(0, 0.50, 10);
    master.config_kI(0, 0.0, 10);
    master.config_kD(0, 5, 10);
    master.config_kF(0, 1023.0 / Constants.kLiftMaxSpeedHighGear, 10);

    master.config_kP(1, 0.50, 10);
    master.config_kI(1, 0.0, 10);
    master.config_kD(1, 5, 10);
    master.config_kF(1, 1023.0 / Constants.kLiftMaxSpeedHighGear, 10);

    master.configMotionCruiseVelocity((int)(300 * 1.0), 10);
    master.configMotionAcceleration((int)(300 * 3.0), 10);
    master.configMotionSCurveStrength(0);

    configForAscent = true;
  }

  private void configForDescent() {
    master.configMotionSCurveStrength(4);

    configForAscent = false;
  }

  public void configForTeleopSpeed() {
    configForAscent();
  }

  public void enableLimits(boolean enable) {
    master.overrideSoftLimitsEnable(enable);
    limitsEnabled = enable;
  }

  public void setCurrentLimit(int amps) {
    for (LazyTalonSRX motor : motors) {
      motor.configContinuousCurrentLimit(amps, 10);
      motor.configPeakCurrentLimit(amps, 10);
      motor.configPeakCurrentDuration(10, 10);
      motor.enableCurrentLimit(true);
    }
  }

  public void setOpenLoop(double output) {
    setState(ControlState.OpenLoop);
    periodicIO.demand = output * manualSpeed;
  }

  public boolean isOpenLoop() {
    return getState() == ControlState.OpenLoop;
  }

  public synchronized void setTargetHeight(double heightInches) {
    setState(ControlState.Position);
    if (heightInches > Constants.kMaxElevatorHeight) {
      heightInches = Constants.kMaxElevatorHeight;
    } else if (heightInches < Constants.kMinElevatorHeight) {
      heightInches = Constants.kMinElevatorHeight;
    }
    if (isSensorConnected()) {
      if (heightInches > getHeight()) {
        master.selectProfileSlot(0, 0);
        configForAscent();
      } else if (heightInches < getHeight()) {
        master.selectProfileSlot(1, 0);
        configForDescent();
      }
      targetHeight = heightInches;
      periodicIO.demand = elevatorHeightToEncTicks(heightInches);
      onTarget = false;
      startTime = Timer.getFPGATimestamp();
    } else {
      DriverStation.reportError("Elevator Encoder Dead", false);
      stop();
    }

  }

  public synchronized void lockHeight() {
    setState(ControlState.Locked);
    if (isSensorConnected()) {
      targetHeight = getHeight();
      periodicIO.demand = periodicIO.position;
    } else {
      DriverStation.reportError("Elevator Encoder Dead", false);
      stop();
    }
  }

  public Request openLoopRequest(double output) {
    return new Request(){
    
      @Override
      public void act() {
        setOpenLoop(output);
      }
    };
  }

  public Request heightRequest(double heightInches) {
    return new Request(){
    
      @Override
      public void act() {
        setTargetHeight(heightInches);
      }

      @Override
      public boolean isFinished() {
        return hasReachedTargetHeight() || isOpenLoop();
      }
    };
  }

  public Request resetRequest() {
    return new Request() {
    
      @Override
      public void act() {
        setOpenLoop(-0.3);
      }

      @Override
      public boolean isFinished() {
        if (Util.epsilonEquals(master.getOutputCurrent(), Constants.kLiftCurrentLimit) 
            || master.getOutputCurrent() > Constants.kLiftCurrentLimit) {
              resetToAbsolutePosition();
              return true;
            }
        return false;
      }
    };
  }

  public Request lockHeightRequest() {
    return new Request(){
    
      @Override
      public void act() {
        lockHeight();
      }
    };
  }

  public Prerequisite heightRequisite(double height, boolean above) {
    return new Prerequisite(){
    
      @Override
      public boolean met() {
        return Util.epsilonEquals(Math.signum(height - getHeight()), above ? 1.0 : -1.0);
      }
    };
  }

  boolean onTarget = false;
  double startTime = 0.0;

  public boolean hasReachedTargetHeight() {
    if (master.getControlMode() == ControlMode.MotionMagic) {
      if (Math.abs(targetHeight - getHeight()) < Constants.kElevatorHeightTolerance) {
        if (!onTarget) {
          System.out.println("Elevator Reached Height in: " 
              + (Timer.getFPGATimestamp() - startTime));
          onTarget = true;
        }
        return true;
      }
    }
    return false;
  }

  private boolean getMotorsWithHighCurrents() {
    return periodicIO.current >= Constants.kLiftCurrentLimit;
  }

  private final Loop loop = new Loop(){
  
    @Override
    public void onStart(double timestamp) {
    
    }
  
    @Override
    public void onLoop(double timestamp) {
      if (getMotorsWithHighCurrents()) {
        DriverStation.reportError("Elevator Current to High", false);
      }      
    }
  
    @Override
    public void onStop(double timestamp) {
      // TODO Auto-generated method stub
      
    }
  };


  @Override
  public void stop() {
    setOpenLoop(0.0);
  }

  @Override
  public void registerEnabledLooper(ILooper looper) {
    looper.register(loop);
  }


  @Override
  public void readPeriodicInputs() {
    periodicIO.position = master.getSelectedSensorPosition(0);

    if (Constants.kDebuggingOutput) {
      periodicIO.current = master.getOutputCurrent();
      periodicIO.velocity = master.getSelectedSensorVelocity(0);
      periodicIO.voltage = master.getMotorOutputVoltage();
    }
  }

  @Override
  public void writePeriodicOutputs() {
    if (getState() == ControlState.Position || getState() == ControlState.Locked) {
      master.set(ControlMode.MotionMagic, periodicIO.demand);
    } else {
      master.set(ControlMode.PercentOutput, periodicIO.demand);
    }
  }

  @Override
  public void zeroSensors() {
    resetToAbsolutePosition();
  }

  @Override
  public void outputTelemetery() {
    SmartDashboard.putNumber("Elevator Height", getHeight());

    if (Constants.kDebuggingOutput) {
      SmartDashboard.putNumber("Elevator 1 Voltage", periodicIO.voltage);
      SmartDashboard.putNumber("Elevator 2 Voltage", slave.getMotorOutputVoltage());
      SmartDashboard.putNumber("Elevator 1 Current", periodicIO.current);
      SmartDashboard.putNumber("Elevator Pulse Width Position",
          master.getSensorCollection().getAnalogIn());
      SmartDashboard.putNumber("Elevator Encoder", periodicIO.position);
      SmartDashboard.putNumber("Elevator Velocity", periodicIO.velocity);
      SmartDashboard.putNumber("Elevator Error", master.getClosedLoopError(0));
      if (master.getControlMode() == ControlMode.MotionMagic) {
        SmartDashboard.putNumber("Elevator Desired Setpoint", targetHeight);
        SmartDashboard.putNumber("Elevator Setpoint", master.getClosedLoopTarget());
      }
    }
  }

  private double encTickToInches(double encTicks) {
    return encTicks / Constants.kEncTicksPerInch;
  }

  private int inchesToEncTicks(double inches) {
    return (int) (inches * Constants.kEncTicksPerInch);
  }

  public double getHeight() {
    return encTicksToElevatorHeight(periodicIO.position);
  }

  public double encTicksToElevatorHeight(double encTicks) {
    return (encTickToInches(encTicks) + Constants.kElevatorInitialHeight) 
      * Constants.kCascadingFactor;
  }

  public double elevatorHeightToEncTicks(double elevatorHeight) {
    return inchesToEncTicks((elevatorHeight - Constants.kElevatorInitialHeight) 
      / Constants.kCascadingFactor);
  }

  public boolean isSensorConnected() {
    boolean connected = master.getSensorCollection().getPinStateQuadA();
    if (!connected) {
      hasEmergency = true;
    }
    return true;
  }

  public void resetToAbsolutePosition() {
    master.setSelectedSensorPosition(0);
  }

  public static class PeriodicIO {
    public int position = 0;
    public double velocity = 0.0;
    public double voltage = 0.0;
    public double current = 0.0;

    public double demand;
  }
  
}