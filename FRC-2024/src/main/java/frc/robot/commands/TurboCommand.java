// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.commands.arm.SetArmLimelight;
import frc.robot.commands.drive.DriveCommands;
import frc.robot.commands.intake.PassNoteToShooter;
import frc.robot.commands.shooter.SetShooterTrapezoid;
import frc.robot.commands.shooter.StopShooter;
import frc.robot.subsystems.LedSubsystem;
import frc.robot.subsystems.arm.ArmSubsystemNew;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.intake.IntakeSubsystem;
import frc.robot.subsystems.limelight.LimelightDriver;
import frc.robot.subsystems.shooter.ShooterSubsystem;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class TurboCommand extends SequentialCommandGroup {
  /** Creates a new TurboCommand. */
  ShooterSubsystem shooterSubsystem;

  IntakeSubsystem intakeSubsystem;
  ArmSubsystemNew armSubsystemNew;
  LimelightDriver limelightSubsystem;
  LedSubsystem ledSubsystem;
  Drive drive;

  public TurboCommand(
      ShooterSubsystem shooterSubsystem,
      IntakeSubsystem intakeSubsystem,
      ArmSubsystemNew armSubsystemNew,
      LimelightDriver limelightDriver,
      LedSubsystem ledSubsystem,
      Drive drive) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
        new ParallelDeadlineGroup( // ends after aim and pass sequence
        
            new SequentialCommandGroup( // aim then pass
                new ParallelDeadlineGroup( // setup aim left/right and arm angle
                    new WaitCommand(2),
                    DriveCommands.angleRotate(drive, () -> 0.5, () -> 0, () -> 0, limelightDriver),
                    new SetArmLimelight(armSubsystemNew, limelightDriver)),
                new PassNoteToShooter(intakeSubsystem) // pass note to sped up shooter
            ),

            new SetShooterTrapezoid(shooterSubsystem, 4200) // start shooter while aiming

          ),
        new StopShooter(shooterSubsystem));
  }
}
