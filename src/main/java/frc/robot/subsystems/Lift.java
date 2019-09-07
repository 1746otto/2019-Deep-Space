package frc.robot.subsystems;

import java.util.Arrays;
import java.util.List;
import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.StatusFrameEnhanced;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants;
import frc.robot.Ports;
import frc.robot.util.LazyTalonSRX;

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
    
    slaves.forEach((s) -> s.set(ControlMode.Follower, Ports.ELEVATOR_TALON));
    
    for (LazyTalonSRX motor : motors) {
      motor.configVoltageCompSaturation(12.0, 10);
      motor.enableVoltageCompensation(true);
      motor.setNeutralMode(NeutralMode.Brake);
    }
    
    master.configSelectedFeedbackSensor(FeedbackDevice.QuadEncoder, 0, 10);
    master.setStatusFramePeriod(StatusFrameEnhanced.Status_13_Base_PIDF0, 10);
    master.configForwardSoftLimitEnable(true, 10);
    master.configReverseSoftLimitEnable(true, 10);
    enableLimits(true);

    setCurrentLimit(Constants.kLiftCurrentLimit);

    resetToAbsolutePosition();
    configForAscent();
  }

  private void configForAscent() {
    manualSpeed = Constants.kLiftTeleopManualSpeed;

    master.config_kP(0, 1.75, 10);
    master.config_kI(0, 0.0, 10);
    master.config_kD(0, 40.0, 10);
    master.config_kF(0, 1023.0 / Constants.kLiftMaxSpeedHighGear, 10);

    master.config_kP(1, 1.5, 10);
    master.config_kI(1, 0.0, 10);
    master.config_kD(1, 30.0, 10);
    master.config_kF(1, 1023.0 / Constants.kLiftMaxSpeedHighGear, 10);

    master.configMotionCruiseVelocity((int)(Constants.kLiftMaxSpeedHighGear * 1.0), 10);
    master.configMotionAcceleration((int)(Constants.kLiftMaxSpeedHighGear * 3.0), 10);
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

  public synchronized void setTargetHeight(double heightFeet) {
    setState(ControlState.Position);
    if (heightFeet > Constants.kMaxElevatorHeight) {
      heightFeet = Constants.kMaxElevatorHeight;
    } else if (heightFeet < Constants.kMinElevatorHeight) {
      heightFeet = Constants.kMinElevatorHeight;
    }
    if (isSensorConnected()) {
      if (heightFeet > getHeight()) {
        liftRight.selectProfileSlot(0, 0);
        configForAscent();
      } else if (heightFeet < getHeight()) {
        liftRight.selectProfileSlot(1, 0);
        configForDescent();
      }
      targetHeight = heightFeet;
      periodicIO.demand = elevatorHeightToEncTicks(heightFeet);
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

  @Override
  public void stop() {

  }

  @Override
  public void outputTelemetery() {
    SmartDashboard.putNumber("Elevator Height", getHeight());

    if (Constants.kDebuggingOutput) {
      SmartDashboard.putNumber("Elevator 1 Voltage", periodicIO.voltage);
      SmartDashboard.putNumber("Elevator 2 Voltage", slave.getMotorOutputVoltage());
      SmartDashboard.putNumber("Elevator 1 Current", periodicIO.current);
      SmartDashboard.putNumber("Elevator Pulse Width Position",
          master.getSensorCollection().getPulseWidthPosition());
      SmartDashboard.putNumber("Elevator Encoder", periodicIO.position);
      SmartDashboard.putNumber("Elevator Velocity", periodicIO.velocity);
      SmartDashboard.putNumber("Elevator Error", master.getClosedLoopError(0));
      if (master.getControlMode() == ControlMode.MotionMagic) {
        SmartDashboard.putNumber("Elevator Setpoint", master.getClosedLoopTarget());
      }
    }
  }

  public static class PeriodicIO {
    public int position = 0;
    public double velocity = 0.0;
    public double voltage = 0.0;
    public double current = 0.0;

    public double demand;
  }
  
}