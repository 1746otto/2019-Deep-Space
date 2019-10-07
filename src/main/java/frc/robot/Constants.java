package frc.robot;

public class Constants {
  public static final boolean kDebuggingOutput = true;
  public static final double kPeriod = 0.02;
  public static final double kElipson = 1e-6;

  // Elevator
  public static final double kElevatorMaxMotorRPM = 16000;
  public static final double kElevatorGearRatio = 11.11;
  public static final double kElevatorEncRes = 1024;
  public static final double kLiftMaxSpeedHighGear = (kElevatorMaxMotorRPM / kElevatorGearRatio) * kElevatorEncRes / 600.0;
  public static final double kCascadingFactor = 3;
  public static final double kEncTicksPerInch = 100;
  public static final double kElevatorInitialHeight = 4.375;

  public static final double kMinElevatorHeight = 0.0;
  public static final double kMaxElevatorHeight = 73;
  public static final double kCargoShipHeight = 36;
  public static final double kMidLevelHatchRocketHeight = 33;
  public static final double kHighLevelHatchRocketHeight = 60;
  public static final double kLowLevelCargoHeight = 24;
  public static final double kMidLevelCargoHeight = 53;
  public static final double kHighLevelCargoHeight = 70;

  public static final double kElevatorHeightTolerance = 0.1;
  public static final double kLiftCargoTolerance = 0.1;

  public static final double kLiftTeleopManualSpeed = 0.8;
  public static final int kLiftCurrentLimit = 45;

  // Drive Constants
  public static final double kDriveMaxSpeed = 0.05;
}