// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.intake;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.intake.IntakeSubsystem;
import frc.robot.subsystems.intake.IntakeSubsystem.IntakeState;

public class PassNoteToShooter extends Command {

  private final IntakeSubsystem intakeSubsystem;
  private double startTime;
  private double endTime;

  /** Runs intake for few seconds to pass a note to shooter */
  public PassNoteToShooter(IntakeSubsystem intakeSubsystem) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.intakeSubsystem = intakeSubsystem;
    addRequirements(this.intakeSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    intakeSubsystem.setIntakeVolts(Constants.Intake.PASSING_VOLTS);
    startTime = Timer.getMatchTime();
    endTime = startTime + Constants.Intake.PASSING_TIME;
    intakeSubsystem.setIntakeState(IntakeState.EMPTY);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    intakeSubsystem.setIntakeVolts(Constants.Intake.PASSING_VOLTS);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    intakeSubsystem.setIntakeState(IntakeState.EMPTY);
    intakeSubsystem.stopIntake();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if (Timer.getMatchTime() - startTime > endTime) {
      return true;
    } else {
      return false;
    }
  }
}
