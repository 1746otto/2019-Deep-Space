package frc.robot.util;

import frc.robot.Constants;

public class Util {

  private Util() {

  }

  public static boolean epsilonEquals(double a, double b, double kElipson) {
    return (a - kElipson <= b) && (a + kElipson >= b);
  }

  public static boolean epsilonEquals(double a, double b) {
    return epsilonEquals(a, b, Constants.kElipson);
  }

  public static double boundToScope(double lowerScope, double upperScope, double argument) {
    double stepSize = upperScope - lowerScope;
    while (argument >= upperScope) {
      argument -= stepSize;
    }
    while (argument < lowerScope) {
      argument += stepSize;
    }
    return argument;
  }

  public static double deadBand(double val, double deadband) {
    return (Math.abs(val) > Math.abs(deadband)) ? val : 0.0;
  }
}