package frc.robot.lib.team254.geometry;

public interface ICurvature<S> extends State<S> {
  double getCurvature();

  double getDCurvatureDs();
}
