// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.shooter;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.commands.intake.PassNoteToShooter;
import frc.robot.subsystems.intake.IntakeSubsystem;
import frc.robot.subsystems.shooter.ShooterSubsystem;

public class PassAndShootNote extends Command {

  IntakeSubsystem intakeSubsystem;
  ShooterSubsystem shooterSubsystem;
  double startTime;
  double passTime;
  double endTime;
  boolean isNotePassed;

  /** Speed up the shooter, then pass a note and end. */
  public PassAndShootNote(ShooterSubsystem shooterSubsystem, IntakeSubsystem intakeSubsystem) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.shooterSubsystem = shooterSubsystem;
    this.intakeSubsystem = intakeSubsystem;
    addRequirements(this.shooterSubsystem, this.intakeSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    isNotePassed = false;
    shooterSubsystem.setShooterPower(Constants.Shooter.shootingPower);
    startTime = Timer.getFPGATimestamp();
    passTime = startTime + Constants.Shooter.shooterSpeedupDelay;
    endTime = passTime + Constants.Intake.passingTime;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (!isNotePassed) {
      if (Timer.getFPGATimestamp() > passTime) {
        new PassNoteToShooter(intakeSubsystem);
        isNotePassed = true;
      }
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    shooterSubsystem.stopShooter();
    intakeSubsystem.stopIntake();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if (Timer.getFPGATimestamp() > endTime) {
      return true;
    } else {
      return false;
    }
  }
}
