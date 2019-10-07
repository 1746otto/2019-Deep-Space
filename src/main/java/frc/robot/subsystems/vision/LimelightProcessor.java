package frc.robot.subsystems.vision;

import frc.robot.loops.Loop;

public class LimelightProcessor implements Loop {
  private double validTarget;  // Whether the limelight has any valid targets (0 or 1)
  private double xOffset;      // Horiszontal Offset from crosshair to target (-27 degrees to 27 degrees)
  private double yOffset;      // Vertical Offset from crosshair to target (-20.5 degrees to 20.5 degrees)
  private double targetArea;   // target area (0% of image to 100%)
  private double skew;         // skew or rotation (-90 degrees to 0 degrees)
  private double pipeLatency;  // the pipeline's latency contribution (ms) add at least 11 ms for image capture latency
  private double tshort;       // Sidelength of shortest side of the fitted bounding box (pixels)
  private double tlong;        // Sidelength of longest side of the fitted bounding box (pixels)
  private double thor;         // Horizontal sidelength of the rough bounding box (0 - 320 pixels)
  private double tvert;        // Vertical sidelength of the rough bounding box (0 - 320 pixels)
  private double getpipe;      // True active pipeline index of the camera (0 .. 9)
  private double camtran;      // Results of a 3D position solution, 6 numbers: Translation (x,y,y) Rotation(pitch,yaw,roll)


  @Override
  public void onStart(double timestamp) {
      
  }

  @Override
  public void onLoop(double timestamp) {
      
  }

  @Override
  public void onStop(double timestamp) {
      
  }

  public double generateSteer() {
    return 0.1;
  }

}