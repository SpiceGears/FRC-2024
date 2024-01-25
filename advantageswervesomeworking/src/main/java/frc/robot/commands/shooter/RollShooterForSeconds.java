// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.shooter;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.shooter.ShooterSubsystem;

public class RollShooterForSeconds extends Command {
  /** Creates a new RollShooterForSeconds. */
  ShooterSubsystem shooterSubsystem;

  double seconds;
  double startTime;
  double endTime;

  public RollShooterForSeconds(ShooterSubsystem shooterSubsystem, double seconds) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.shooterSubsystem = shooterSubsystem;
    addRequirements(this.shooterSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    startTime = Timer.getFPGATimestamp();
    endTime = startTime + seconds;
    shooterSubsystem.setShooterPower(frc.robot.Constants.Shooter.SHOOTING_POWER);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    shooterSubsystem.stopShooter();
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
