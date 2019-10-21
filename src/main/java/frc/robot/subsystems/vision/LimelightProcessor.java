package frc.robot.subsystems.vision;

import edu.wpi.first.networktables.NetworkTableInstance;
import frc.robot.loops.Loop;

public class LimelightProcessor implements Loop {
  private static LimelightProcessor instance;

  private boolean validTarget;  // Whether the limelight has any valid targets (0 or 1)
  private double horzOffset;      // Horiszontal Offset from crosshair to target 
  private double camMode;

  private double prevTxError = 0;
  private double txError;
  private double deltaTxError;

  private double steerP = 0.06;
  private double steerD = 0.2;
  private double steerF = 0.4;

  public static LimelightProcessor getInstance() {
    if (instance == null) {
      instance = new LimelightProcessor();
    }
    return instance;
  }


  @Override
  public void onStart(double timestamp) {
    NetworkTableInstance.getDefault().getTable("limelight")
        .getEntry("camMode").setNumber(1);
  }

  @Override
  public void onLoop(double timestamp) {
    try {
      validTarget = (NetworkTableInstance.getDefault()
          .getTable("limelight").getEntry("tv").getDouble(0) == 1) ? true : false;
      horzOffset = NetworkTableInstance.getDefault()
          .getTable("limelight").getEntry("tx").getDouble(0);
      camMode = NetworkTableInstance.getDefault()
          .getTable("limelight").getEntry("camMode").getDouble(0);
    } catch (Exception ex) {
      throw ex;
    }                                 
  }

  @Override
  public void onStop(double timestamp) {

  }

  public double generateSteer() {
    txError = (horzOffset + 0.5) * steerF;
    if (txError != prevTxError) {
      deltaTxError = txError - prevTxError;
    }
    if (!validTarget) {
      prevTxError = 0; 
      return 0.0;
    } else {
      prevTxError = txError;
      return (txError * steerP) + (deltaTxError * steerD);
    }
  }

  public void toggleCamMode(boolean toggle) {
    if (toggle) {
      NetworkTableInstance.getDefault().getTable("limelight")
          .getEntry("camMode").setNumber(0);
    } else {
      NetworkTableInstance.getDefault().getTable("limelight")
          .getEntry("camMode").setNumber(1);
    }
  }

}