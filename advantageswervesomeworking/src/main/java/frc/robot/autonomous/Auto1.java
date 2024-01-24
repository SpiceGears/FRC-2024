// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.autonomous;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.*;
import frc.robot.commands.intake.IntakeNote;
import frc.robot.commands.intake.PassNoteToShooter;
import frc.robot.commands.shooter.StartShooter;
import frc.robot.subsystems.intake.IntakeSubsystem;
import frc.robot.subsystems.shooter.ShooterSubsystem;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class Auto1 extends SequentialCommandGroup {
  /** Creates a new Auto1. */
  ShooterSubsystem shooterSubsystem;

  IntakeSubsystem intakeSubsystem;

  public Auto1() {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
        new StartShooter(shooterSubsystem), // roll shooter up to speed

        // [ARM] aim arm to SPEAKER (limelight data)
        // [SHOOTER] adjust speed (limelight data)
        new PassNoteToShooter(intakeSubsystem), // shoots note
        new ParallelCommandGroup(
            new IntakeNote(intakeSubsystem)
            // [SWERVE] go back to NOTE2,
            ),
        // [ARM] aim arm to SPEAKER (limelight data)
        // [SHOOTER] adjust speed (limelight data)
        new PassNoteToShooter(intakeSubsystem), // shoots note
        new ParallelCommandGroup(
            new IntakeNote(intakeSubsystem)
            // [SWERVE] go to NOTE3,
            ),
        // [ARM] aim arm to SPEAKER (limelight data)
        // [SHOOTER] adjust speed (limelight data)
        new PassNoteToShooter(intakeSubsystem) // shoots note
        // [SWERVE] drive out of zone for points

        );
  }
}
