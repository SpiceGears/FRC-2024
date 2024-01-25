// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.intake.IntakeSubsystem;
import frc.robot.subsystems.shooter.ShooterSubsystem;

public class ShuffleBoard extends SubsystemBase {
  /** Creates a new ShuffleBoard. */
  IntakeSubsystem intakeSubsystem;
  ShooterSubsystem shooterSubsystem;
  Drive drive;
  public ShuffleBoard(
    IntakeSubsystem intakeSubsystem,
    ShooterSubsystem shooterSubsystem,
    Drive drive) {

      this.intakeSubsystem = intakeSubsystem;
      this.shooterSubsystem = shooterSubsystem;
      this.drive = drive;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putBoolean("shuffleboardsubsystem/intake/noteInside", intakeSubsystem.checkForNoteInside());
    SmartDashboard.putNumber("shuffleboardsubsystem/intake/speed", intakeSubsystem.getIntakePower());
    SmartDashboard.putNumber("shuffleboardsubsystem/shooter/speed", shooterSubsystem.getShooterPower());
    SmartDashboard.putString("shuffleboardsubsystem/drive/pose", drive.getPose().toString());
  }
}
