package frc.robot.subsystems;

public class CargoIntake extends Subsystem {
  private CargoIntake instance = null;

  public CargoIntake getInstance() {
    if (instance == null) {
      instance = new CargoIntake();
    } 
    return instance;
  }


  @Override
  public void stop() {
    // TODO Auto-generated method stub

  }

  @Override
  public void outputTelemetery() {
    // TODO Auto-generated method stub

  }
  
}