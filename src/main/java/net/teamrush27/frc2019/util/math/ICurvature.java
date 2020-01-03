package net.teamrush27.frc2019.util.math;

public interface ICurvature<S> extends State<S> {
  double getCurvature();

  double getDCurvatureDs();
}
