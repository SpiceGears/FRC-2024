// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.intake;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.intake.IntakeSubsystem;
import frc.robot.subsystems.intake.IntakeSubsystem.IntakeState;

public class IntakeNote extends Command {

  private final IntakeSubsystem intakeSubsystem;

  /** Intake notes until note is collected */
  public IntakeNote(IntakeSubsystem intakeSubsystem) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.intakeSubsystem = intakeSubsystem;
    addRequirements(this.intakeSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    IntakeSubsystem.intakeState = IntakeState.INTAKING;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    switch (IntakeSubsystem.intakeState) {
      case INTAKING:
        intakeSubsystem.setIntakeVolts(5);
        if (intakeSubsystem.checkForNoteInside()) {
          IntakeSubsystem.intakeState = IntakeState.BACKING;
        }
        break;

      case BACKING:
        if (intakeSubsystem.checkForNoteInside()) {
          intakeSubsystem.setIntakeVolts(-3.6);
        } else {
          IntakeSubsystem.intakeState = IntakeState.READY;
        }

        break;

      case READY:
        // IntakeSubsystem.setIntakeVolts(0);
        break;

      case EMPTY:
        IntakeSubsystem.intakeState = IntakeState.INTAKING;
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
    return IntakeSubsystem.intakeState == IntakeState.READY;
  }
}
