// Copyright 2021-2024 FRC 6328
// http://github.com/Mechanical-Advantage
//
// This program is free software; you can redistribute it and/or
// modify it under the terms of the GNU General Public License
// version 3 as published by the Free Software Foundation or
// available in the root directory of this project.
//
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
// GNU General Public License for more details.

// https://www.kauailabs.com/public_files/navx-mxp/apidocs/java/com/kauailabs/navx/frc/AHRS.html

package frc.robot.subsystems.drive;

import com.kauailabs.navx.frc.AHRS;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/** IO implementation for Navx */
public class GyroIONavx implements GyroIO {
  private final AHRS gyro = new AHRS(SPI.Port.kMXP);
  private final Float yaw = gyro.getYaw();
  //private final Float yawVelocity = gyro.getVelocityZ();
  private final Float yawVelocity = gyro.getQuaternionZ();

  public GyroIONavx() {
    gyro.reset();
  }

  @Override
  public void updateInputs(GyroIOInputs inputs) {
    inputs.connected = gyro.isConnected();
    inputs.yawPosition = Rotation2d.fromDegrees((double) yaw);
    inputs.yawVelocityRadPerSec = Units.degreesToRadians((double) yawVelocity);
    SmartDashboard.putNumber("Gyro/Yaw", yaw);
    SmartDashboard.putNumber("Gyro/YawVelocity", yawVelocity);
    SmartDashboard.putNumber("Gyro/Angle", gyro.getAngle());
    SmartDashboard.putNumber("Gyro/VelocityZ", gyro.getVelocityZ());
    SmartDashboard.putNumber("Gyro/QuaternionZ", gyro.getQuaternionZ());
  }
}
