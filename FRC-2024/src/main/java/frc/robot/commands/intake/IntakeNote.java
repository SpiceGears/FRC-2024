// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.intake;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.intake.IntakeSubsystem;

public class IntakeNote extends Command {

  private final IntakeSubsystem intakeSubsystem;

  private enum IntakeState {
    INTAKING,
    BACKING,
    READY
  }

  private IntakeState intakeState;

  /** Intake notes until note is collected */
  public IntakeNote(IntakeSubsystem intakeSubsystem) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.intakeSubsystem = intakeSubsystem;
    addRequirements(this.intakeSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    intakeState = IntakeState.INTAKING;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    switch (intakeState) {
      case INTAKING:
        intakeSubsystem.setIntakeVolts(6);
        if (intakeSubsystem.checkForNoteInside()) {
          intakeState = IntakeState.BACKING;
        }
        break;

      case BACKING:
        if (intakeSubsystem.checkForNoteInside()) {
          intakeSubsystem.setIntakeVolts(-3.6);
        } else {
          intakeState = IntakeState.READY;
        }

        break;

      case READY:
        // intakeSubsystem.setIntakeVolts(0);
        break;
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    intakeSubsystem.setIntakeVolts(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return intakeState == IntakeState.READY;
  }
}
