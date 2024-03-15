// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.drive;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.limelight.LimelightDriver;
import java.util.function.DoubleSupplier;

public class DriveLL extends Command {
  /** Creates a new DriveLL. */
  Drive drive;

  DoubleSupplier speedSupplier;
  DoubleSupplier xSupplier;
  DoubleSupplier ySupplier;
  LimelightDriver limelightDriver;

  public DriveLL(
      Drive drive,
      DoubleSupplier speedSupplier,
      DoubleSupplier xSupplier,
      DoubleSupplier ySupplier,
      LimelightDriver limelightDriver) {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(drive);
    this.drive = drive;
    this.speedSupplier = speedSupplier;
    this.xSupplier = xSupplier;
    this.ySupplier = ySupplier;
    this.limelightDriver = limelightDriver;
  }
  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    drive.isDriveByLL = true;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    // Apply deadband
    double linearMagnitude =
        MathUtil.applyDeadband(Math.hypot(xSupplier.getAsDouble(), ySupplier.getAsDouble()), 0.1);
    Rotation2d linearDirection = new Rotation2d(xSupplier.getAsDouble(), ySupplier.getAsDouble());
    double tx = limelightDriver.getTxDouble();
    double omega = -MathUtil.applyDeadband(tx, 0.1);

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
        ((omega - (xGamepad * 5.883)) / 30); // should be max error = 1 or -1 and center is 0
    SmartDashboard.putNumber("LL/error", error);

    if (Math.abs(error) < 0.1) {
      drive.onTarget = true;
    } else {
      drive.onTarget = false;
    }

    // Convert to field relative speeds & send command

    double finalRotation = error * 1.5;

    finalRotation = MathUtil.applyDeadband(finalRotation, 0.02);

    // ! adjust joystick axis [-1 to 1] value to usable modifier [0-1]
    double speedModifier = speedSupplier.getAsDouble();

    if (limelightDriver.getTvInt() == 1) { // IF LIMELIGHT SEE TARGET

      // //System.out.println("finalRotation=" + finalRotation + "error=" + error + "");

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
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    drive.runVelocity(
        ChassisSpeeds.fromFieldRelativeSpeeds(
            0,
            0,
            0, // DOESNT ROTATE WITHOUT TARGET
            drive.getRotation()));
    drive.isDriveByLL = false;
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
