package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.InvertType;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants;
import frc.robot.Ports;
import frc.robot.lib.team254.geometry.Translation2d;
import frc.robot.subsystems.requests.Request;
import frc.robot.util.LazyTalonSRX;
import frc.robot.util.LazyVictorSPX;
import frc.robot.util.Util;

import java.util.Arrays;
import java.util.List;

public class Drivetrain extends Subsystem {
  private static Drivetrain instance = null;

  private List<LazyTalonSRX> masters;
  private List<LazyVictorSPX> slaves;

  private LazyTalonSRX rightMaster;
  private LazyTalonSRX leftMaster;

  private LazyVictorSPX rightSlave1;
  private LazyVictorSPX rightSlave2;

  private LazyVictorSPX leftSlave1;
  private LazyVictorSPX leftSlave2;

  private PeriodicIO periodicIO = new PeriodicIO();

  public enum ControlState {
    NEUTRAL, OPEN_LOOP, VISION
  }

  private ControlState state = ControlState.NEUTRAL;

  private void setState(ControlState newState) {
    state = newState;
  }

  public ControlState getState() {
    return state;
  }

  private double maxSpeed = Constants.kDriveMaxSpeed;

  public void setMaxSpeed(double speed) {
    maxSpeed = speed;
  }

  public double getMaxSpeed() {
    return maxSpeed;
  }

  
  public static Drivetrain getInstance() {
    if (instance == null) {
      instance = new Drivetrain();
    } 
    return instance;
  }

  private Drivetrain() {
    rightMaster = new LazyTalonSRX(Ports.MOTOR_DRIVE_RIGHT_MASTER);
    leftMaster = new LazyTalonSRX(Ports.MOTOR_DRIVE_LEFT_MASTER);

    rightSlave1 = new LazyVictorSPX(Ports.MOTOR_DRIVE_RIGHT_FOLLOWER_A);
    rightSlave2 = new LazyVictorSPX(Ports.MOTOR_DRIVE_RIGHT_FOLLOWER_B);

    leftSlave1 = new LazyVictorSPX(Ports.MOTOR_DRIVE_LEFT_FOLLOWER_A);
    leftSlave2 = new LazyVictorSPX(Ports.MOTOR_DRIVE_LEFT_FOLLOWER_B);

    masters = Arrays.asList(rightMaster, leftMaster);

    slaves = Arrays.asList(rightSlave1, rightSlave2, leftSlave1, leftSlave2);

    // rightSlave1.set(ControlMode.Follower, Ports.MOTOR_DRIVE_RIGHT_MASTER);
    // rightSlave2.set(ControlMode.Follower, Ports.MOTOR_DRIVE_RIGHT_MASTER);

    // leftSlave1.set(ControlMode.Follower, Ports.MOTOR_DRIVE_LEFT_MASTER);
    // leftSlave2.set(ControlMode.Follower, Ports.MOTOR_DRIVE_LEFT_MASTER);

    rightSlave1.follow(rightMaster);
    rightSlave2.follow(rightMaster);

    leftSlave1.follow(leftMaster);
    leftSlave2.follow(leftMaster);

    rightMaster.setInverted(true);
    rightSlave1.setInverted(true);
    rightSlave2.setInverted(true);

    rightMaster.configOpenloopRamp(0.0);
    leftMaster.configOpenloopRamp(0.0);
    rightMaster.setNeutralMode(NeutralMode.Brake);
    leftMaster.setNeutralMode(NeutralMode.Brake);

  }

  public void setOpenLoop(Translation2d driveInput) {
    setState(ControlState.OPEN_LOOP);
    periodicIO.controlMode = ControlMode.PercentOutput;

    if (Math.abs(driveInput.y()) > Math.abs(driveInput.x())) {
      periodicIO.rightDemand = 
       (driveInput.y()/10*9) - (driveInput.x()/10*7.5);
      periodicIO.leftDemand = 
       (driveInput.y()/10*9) + (driveInput.x()/10*7);
    } else {
      periodicIO.rightDemand =
       (driveInput.y()/10*9) - (driveInput.x()/10*8.5);
      periodicIO.leftDemand =
       (driveInput.y()/10*9) + (driveInput.x()/10*8);
    }
  }

  public Request openLoopRequest(Translation2d driveInput) {
    return new Request(){
    
      @Override
      public void act() {
        if (Math.abs(driveInput.x()) > Math.abs(driveInput.y())) {
          periodicIO.rightDemand =
              Util.boundToScope(0, maxSpeed, ((driveInput.y()) - (driveInput.x())));
          periodicIO.leftDemand =
              Util.boundToScope(0, maxSpeed, ((driveInput.y()) + (driveInput.x())));
        } else {
          periodicIO.rightDemand =
              Util.boundToScope(0, maxSpeed, ((driveInput.y()) - (driveInput.x())));
          periodicIO.leftDemand =
              Util.boundToScope(0, maxSpeed, ((driveInput.y()) + (driveInput.x())));
        }
      }
    };
  }

  @Override
  public void stop() {
    setOpenLoop(new Translation2d(0, 0));
  }

  @Override
  public void writePeriodicOutputs() {
    rightMaster.set(periodicIO.controlMode, periodicIO.rightDemand);
    leftMaster.set(periodicIO.controlMode, periodicIO.leftDemand);
  }

  @Override
  public void outputTelemetery() {
    if (Constants.kDebuggingOutput) {
      SmartDashboard.putString("Drivebase State", getState().toString());
    }
  }

  public static class PeriodicIO {
    public double rightInput = 0;
    public double leftInput =  0;
    public ControlMode controlMode = ControlMode.PercentOutput;

    public double rightDemand;
    public double leftDemand;
  }

}