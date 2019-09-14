package frc.robot;

public class Constants {
  public static final boolean kDebuggingOutput = false;
  public static final int kPeriod = 20;
  public static final double kElipson = 1e-6;

  // Elevator
  public static final double kLiftMaxSpeedHighGear = 696.96 * Constants.kElevatorEncRes / 600.0;
  public static final double kElevatorEncRes = 100;
  public static final double kLiftTeleopManualSpeed = 0.7;
  public static final int kLiftCurrentLimit = 10;
  public static final double kMaxElevatorHeight = 10;
  public static final double kMinElevatorHeight = 0.0;
  public static final double kElevatorHeightTolerance = 10;
  public static final double kEncTicksPerInch = 10;
  public static final double kElevatorEncoderStartingPosition = 10;


  // Drive Constants
  public static final double kDriveMaxSpeed = 0.8;
}