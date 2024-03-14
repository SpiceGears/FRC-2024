// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.shooter;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.shooter.ShooterSubsystem;

public class StartShooter extends Command {
  /** Creates a new SetShooterTrapezoid. */
  ShooterSubsystem shooterSubsystem;

  double speedRPM;
  double startTime;
  double timeGoal;

  /**
   * Roll shooter up to speed in given time
   *
   * @param shooterSubsystem
   * @param speedRPM
   */
  public StartShooter(ShooterSubsystem shooterSubsystem, double speedRPM) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.shooterSubsystem = shooterSubsystem;
    this.speedRPM = speedRPM;
    addRequirements(shooterSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    this.shooterSubsystem.resetPIDController();
    this.startTime = Timer.getFPGATimestamp();
    timeGoal = 0.254; // in seconds
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double finalOutput = 0;
    double currentTime = Timer.getFPGATimestamp();
    finalOutput = Math.min((currentTime - this.startTime) / this.timeGoal, 1) * this.speedRPM;
    if ((currentTime - this.startTime) > timeGoal) {
      ShooterSubsystem.isShooterReady = true;
    } else {
      ShooterSubsystem.isShooterReady = false;
    }
    shooterSubsystem.setShooterPIDSetpoint(finalOutput);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
