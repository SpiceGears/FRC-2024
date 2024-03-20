// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

/** Add your docs here. */
public class AimBot {
  // (metry, kat ramienia, speed_shootera)
  static final double error = 0.0;
  static double[][] data = {
    // {101, 13.5, 4200},
    // {143, 18 - 2, 4200}


    {102, 23.5 -10, 4200},
    {170, 31 -10, 4200},
    {246, 37 -10, 4200},
    {311, 41 -10, 4200}
  };

  public static double[] interpolate(double distance) {
    // Znajdz dwa najblizsze punkty
    double[] low = data[0];
    double[] high = data[data.length - 1];

    for (int i = 0; i < data.length - 1; i++) {
      if (distance >= data[i][0] && distance <= data[i + 1][0]) {
        low = data[i];
        high = data[i + 1];
        break;
      }
    }

    // Interpolacja liniowa
    double distRatio = (distance - low[0]) / (high[0] - low[0]);
    double angle = low[1] + (high[1] - low[1]) * distRatio;
    double speed = low[2] + (high[2] - low[2]) * distRatio;

    return new double[] {angle, speed};
  }
}
