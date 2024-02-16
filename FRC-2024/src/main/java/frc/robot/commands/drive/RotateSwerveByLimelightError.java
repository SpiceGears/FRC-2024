// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.drive;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.limelight.LimelightSubsystem;

public class RotateSwerveByLimelightError extends Command {
  /** Creates a new RotateSwerveByError. */
  Drive drive;

  LimelightSubsystem limelightSubsystem;

  double tx; // -30 to +30 angle in degrees [crosshair to target]
  double steeringAdjustment; // how much to adjust drivetrain

  double kP = 0.1;
  double adjustmentDeadband = 5;

  public RotateSwerveByLimelightError(Drive drive, LimelightSubsystem limelightSubsystem) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.drive = drive;
    this.limelightSubsystem = limelightSubsystem;
    addRequirements(drive, limelightSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    tx = limelightSubsystem.getTxDouble();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    tx = limelightSubsystem.getTxDouble(); // -30 to +30 angle in degrees [crosshair to target]
    steeringAdjustment = kP * tx;

    if (tx > adjustmentDeadband || tx < -adjustmentDeadband) {
      // rotate swerve with power of steeringAdjustment
    } else {
      end(false); // end command when in deadband
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    // TODO stop swerve drive from moving
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if (tx < adjustmentDeadband || tx > -adjustmentDeadband) {
      return true;
    } else {
      return false;
    }
  }
}
