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
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.limelight.LimelightSubsystem;
import java.util.function.DoubleSupplier;

public class DriveCommands {
  private static final double DEADBAND = 0.1;
  private static final double LIMELIGHT_DEADBAND = 0.15;

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
          // Apply deadband
          double linearMagnitude =
              MathUtil.applyDeadband(
                  Math.hypot(xSupplier.getAsDouble(), ySupplier.getAsDouble()), DEADBAND);
          Rotation2d linearDirection =
              new Rotation2d(xSupplier.getAsDouble(), ySupplier.getAsDouble());
          double omega = MathUtil.applyDeadband(omegaSupplier.getAsDouble(), DEADBAND);

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

  /**
   * @param drive Drive drive
   * @param angleSupplier -1 to 1 value of how much to the side to aim
   * @param tv check if limelight can even see any apriltag
   */
  public static Command angleRotate(
      Drive drive,
      DoubleSupplier speedSupplier,
      DoubleSupplier xSupplier,
      DoubleSupplier ySupplier,
      LimelightSubsystem limelightSubsystem, // ! limelightSubsystem.getTxDouble()
      int tv) {
    return Commands.run(
        () -> {

          // Apply deadband
          double linearMagnitude =
              MathUtil.applyDeadband(
                  Math.hypot(xSupplier.getAsDouble(), ySupplier.getAsDouble()), DEADBAND);
          Rotation2d linearDirection =
              new Rotation2d(xSupplier.getAsDouble(), ySupplier.getAsDouble());
          double tx = limelightSubsystem.getTxDouble();
          double omega = -MathUtil.applyDeadband(tx, DEADBAND);

          // Square values
          linearMagnitude = linearMagnitude * linearMagnitude;

          // ! MODYFY LIMELIGHT ERROR HERE
          // ! INPUT IS ANGLES(-30 to 30) OUTPUT IS RAD/SECOND
          // omega = Math.copySign(omega * omega, omega);

          double xGamepad = ySupplier.getAsDouble();

          // Calcaulate new linear velocity
          Translation2d linearVelocity =
              new Pose2d(new Translation2d(), linearDirection)
                  .transformBy(new Transform2d(linearMagnitude, 0.0, new Rotation2d()))
                  .getTranslation();

          // omega -30 to 30 degrees
          // error -1 to 1
          double error =
              (omega + xGamepad * 4) / 30; // should be max error = 1 or -1 and center is 0

          if (Math.abs(error) < LIMELIGHT_DEADBAND) {
            drive.onTarget = true;
          } else {
            drive.onTarget = false;
          }

          // Convert to field relative speeds & send command

          double finalRotation = error * 2.5;

          finalRotation = MathUtil.applyDeadband(finalRotation, 0.02);

          // ! adjust joystick axis [-1 to 1] value to usable modifier [0-1]
          double speedModifier = (speedSupplier.getAsDouble() + 1) / 2;
          double speedModifierMinimum = 0.2;
          if (speedModifier < speedModifierMinimum) {
            speedModifier = speedModifierMinimum;
          }

          if (tv == 1) { // IF LIMELIGHT SEE TARGET

            drive.runVelocity(
                ChassisSpeeds.fromFieldRelativeSpeeds(
                    linearVelocity.getX() * drive.getMaxLinearSpeedMetersPerSec() * speedModifier,
                    linearVelocity.getY() * drive.getMaxLinearSpeedMetersPerSec() * speedModifier,
                    finalRotation, // rad/second
                    drive.getRotation()));

          } else {
            drive.runVelocity(
                ChassisSpeeds.fromFieldRelativeSpeeds(
                    linearVelocity.getX() * drive.getMaxLinearSpeedMetersPerSec() * speedModifier,
                    linearVelocity.getY() * drive.getMaxLinearSpeedMetersPerSec() * speedModifier,
                    0, // DOESNT ROTATE WITHOUT TARGET
                    drive.getRotation()));
          }
        },
        drive);
  }
}
