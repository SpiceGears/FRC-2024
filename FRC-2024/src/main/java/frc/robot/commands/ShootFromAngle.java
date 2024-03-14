// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants;
import frc.robot.commands.arm.SetArm;
import frc.robot.commands.arm.SetArmLimelight;
import frc.robot.commands.intake.PassNoteToShooter;
import frc.robot.commands.shooter.StartShooter;
import frc.robot.commands.shooter.StopShooter;
import frc.robot.subsystems.arm.ArmSubsystemNew;
import frc.robot.subsystems.intake.IntakeSubsystem;
import frc.robot.subsystems.limelight.LimelightDriver;
import frc.robot.subsystems.shooter.ShooterSubsystem;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class ShootFromAngle extends SequentialCommandGroup {
  /** Creates a new ShootFromPosition. */
  ArmSubsystemNew armSubsystemNew;
  IntakeSubsystem intakeSubsystem;
  ShooterSubsystem shooterSubsystem;
  LimelightDriver limelightDriver;
  double armAngle;

  public ShootFromAngle(
  double armAngle,
  ArmSubsystemNew armSubsystemNew,
  IntakeSubsystem intakeSubsystem,
  ShooterSubsystem shooterSubsystem,
  LimelightDriver limelightDriver) {

    this.armAngle = armAngle;
    this.armSubsystemNew = armSubsystemNew;
    this.intakeSubsystem = intakeSubsystem;
    this.shooterSubsystem = shooterSubsystem;

    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
     new ParallelDeadlineGroup(
      new WaitCommand(0.5),
        new StartShooter(shooterSubsystem, Constants.Shooter.DEFAULT_RPM),
        new SetArm(armSubsystemNew, armAngle)
     ),
     new PassNoteToShooter(intakeSubsystem),
     new StopShooter(shooterSubsystem)
      
    );
  }
}
