package frc.robot;

public class Constants {
  public static final boolean kDebuggingOutput = false;
  public static final int kPeriod = 20;
  public static final double kElipson = 1e-6;

  // Elevator
  public static final double kLiftMaxSpeedHighGear = 696.96 * Constants.kElevatorEncRes / 600.0;
  public static final double kElevatorEncRes = 100;
  public static final double kElevatorToEncoderRatio = 3;
  public static final double kEncTicksPerInch = kElevatorToEncoderRatio * kElevatorEncRes;
  public static final double kElevatorInitialHeight = 4.375;

  public static final double kMaxElevatorHeight = 90;
  public static final double kMinElevatorHeight = 0.0;
  public static final double kCargoShipHeight = 36;
  public static final double kMidLevelHatchRocketHeight = 33;
  public static final double kHighLevelHatchRocketHeight = 60;
  public static final double kLowLevelCargoHeight = 24;
  public static final double kMidLevelCargoHeight = 53;
  public static final double kHighLevelCargoHeight = 70;

  public static final double kElevatorHeightTolerance = 0.3;
  public static final double kLiftCargoTolerance = 0;

  public static final double kLiftTeleopManualSpeed = 0.4;
  public static final int kLiftCurrentLimit = 45;

  // Drive Constants
  public static final double kDriveMaxSpeed = 0.8;
}