package frc.robot.subsystems;

import com.ctre.phoenix.sensors.PigeonIMU;
import com.ctre.phoenix.sensors.PigeonIMU.PigeonState;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Ports;
import frc.robot.util.LazyTalonSRX;

public class Pigeon {
  private static Pigeon instance = null;

  private PigeonIMU pigeon;

  /**
   * Get a singular instance of the Pigeon IMU object.
   * 
   * @return An instance of the Pigeon IMU
   */
  public static Pigeon getInstance() {
    if (instance == null) {
      instance = new Pigeon();
    }
    return instance;
  }

  private Pigeon() {
    try {
      pigeon = new PigeonIMU(new LazyTalonSRX(Ports.GYRO));
    } catch (Exception e) {
      System.out.println(e);
    }
  }

  public boolean isGood() {
    return (pigeon.getState() == PigeonState.Ready) ? true : false;
  }

  /**
   * Returns the Yaw of the Pigeon.
   * 
   * @return Yaw value in Rotation2d format
   */
 

  /**
   * Returns the Pitch of the Pigeon.
   * 
   * @return Pitch
   */
  public double getPitch() {
    double[] ypr = new double[3];
    pigeon.getYawPitchRoll(ypr);
    return ypr[1];
  }

  /**
   * Returns the Roll of the Pigeon.
   * 
   * @return Roll
   */
  public double getRoll() {
    double[] ypr = new double[3];
    pigeon.getYawPitchRoll(ypr);
    return ypr[2];
  }

  /**
   * Returns Yaw, Pitch, and Roll of the Pigeon.
   * 
   * @return An Array of all three axis values
   */
  public double[] getYPR() {
    double[] ypr = new double[3];
    pigeon.getYawPitchRoll(ypr);
    return ypr;
  }

  /**
   * Sets the angle the Pigeon should be at.
   * 
   * @param angle Angle Pigeon should be at
   */
  public void setAngle(double angle) {
    pigeon.setFusedHeading(-angle * 64.0, 10);
    pigeon.setYaw(-angle, 10);
    System.out.println("Pigeon Angle set to: " + angle);
  }

  public void outputToSmartDashboard() {
    SmartDashboard.putString("Pigeon Status", pigeon.getState().toString());
  }
}
