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

package frc.robot.commands.drive;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.subsystems.drive.Drive;
import java.util.function.DoubleSupplier;

public class DriveCommands {
  private static final double DEADBAND = 0.1;
  private static final double LIMELIGHT_DEADBAND = 0.15;
  private static final double rateLimit = 1.25;
  private static SlewRateLimiter xLimiter = new SlewRateLimiter(rateLimit);
  private static SlewRateLimiter yLimiter = new SlewRateLimiter(rateLimit);
  private static SlewRateLimiter omegaLimiter = new SlewRateLimiter(rateLimit);

  /**
   * Field relative drive command using two joysticks (controlling linear and angular velocities).
   */
  public static Command joystickDrive(
      Drive drive,
      DoubleSupplier speedSupplier,
      DoubleSupplier xSupplier,
      DoubleSupplier ySupplier,
      DoubleSupplier omegaSupplier) {
    return Commands.run(
        () -> {
          double linearMagnitude =
              MathUtil.applyDeadband(
                  Math.hypot(xLimiter.calculate(xSupplier.getAsDouble()), yLimiter.calculate(ySupplier.getAsDouble())), DEADBAND);
          Rotation2d linearDirection =
              new Rotation2d(xLimiter.calculate(xSupplier.getAsDouble()), yLimiter.calculate(ySupplier.getAsDouble()));

          double omega = MathUtil.applyDeadband(omegaLimiter.calculate(omegaSupplier.getAsDouble()), DEADBAND) * 0.69;

          // Square values
          linearMagnitude = linearMagnitude * linearMagnitude;
          omega = Math.copySign(omega * omega, omega);

          // Calcaulate new linear velocity
          Translation2d linearVelocity =
              new Pose2d(new Translation2d(), linearDirection)
                  .transformBy(new Transform2d(linearMagnitude, 0.0, new Rotation2d()))
                  .getTranslation();

          // ! adjust joystick axis [-1 to 1] value to usable modifier [0-1]
          double speedModifier = (speedSupplier.getAsDouble() + 1) / 2;
          double speedModifierMinimum = 0.2;
          if (speedModifier < speedModifierMinimum) {
            speedModifier = speedModifierMinimum;
          }

          // Convert to field relative speeds & send command
          drive.runVelocity(
              ChassisSpeeds.fromFieldRelativeSpeeds(
                  linearVelocity.getX() * drive.getMaxLinearSpeedMetersPerSec() * speedModifier,
                  linearVelocity.getY() * drive.getMaxLinearSpeedMetersPerSec() * speedModifier,
                  omega * drive.getMaxAngularSpeedRadPerSec() * speedModifier,
                  drive.getRotation()));
        },
        drive);
  }

  public static InstantCommand stopDrive(Drive drive) {
    return new InstantCommand(
        () -> {
          drive.runVelocity(new ChassisSpeeds());
        },
        drive);
    // () -> {
    //   drive.stop();
    // }
    // );
  }
}
