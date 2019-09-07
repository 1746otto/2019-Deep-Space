package frc.robot.subsystems.requests;

public class EmptyRequest extends Request {

  @Override
  public void act() {

  }
  
  @Override
  public boolean isFinished() {
    return true;
  }
}