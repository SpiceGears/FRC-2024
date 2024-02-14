// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.drive;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.drive.Drive;

public class RotateSwerveByAngle extends Command {
  /** Creates a new RotateSwerveByAngle. */
  Drive drive;

  public RotateSwerveByAngle(Drive drive) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.drive = drive;
    addRequirements(drive);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    // drive.runVelocity(
    // ChassisSpeeds.fromFieldRelativeSpeeds(
    //     linearVelocity.getX() * drive.getMaxLinearSpeedMetersPerSec(),
    //     linearVelocity.getY() * drive.getMaxLinearSpeedMetersPerSec(),
    //     omega * drive.getMaxAngularSpeedRadPerSec(),
    //     drive.getRotation()));
  }
  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
