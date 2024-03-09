// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.arm;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.AimBot;
import frc.robot.subsystems.arm.ArmSubsystemNew;
import frc.robot.subsystems.arm.ArmSubsystemNew.ArmState;
import frc.robot.subsystems.limelight.LimelightSubsystem;

public class SetArmLimelight extends Command {
  /** Creates a new SetArmLimelight. */
  private final ArmSubsystemNew ArmSubsystemNew;

  private double position;
  private LimelightSubsystem limelightSubsystem;

  public SetArmLimelight(ArmSubsystemNew ArmSubsystemNew, LimelightSubsystem limelightSubsystem) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.position = AimBot.interpolate(limelightSubsystem.getDistance())[0];
    this.ArmSubsystemNew = ArmSubsystemNew;
    this.limelightSubsystem = limelightSubsystem;
    addRequirements(ArmSubsystemNew);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    this.position = AimBot.interpolate(limelightSubsystem.getDistance())[0];
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (limelightSubsystem.getTvInt() == 1) {
      this.position = AimBot.interpolate(limelightSubsystem.getDistance())[0];
      ArmSubsystemNew.setArmState(ArmState.ENCODER);
      ArmSubsystemNew.setSetpoint(position);
    } else {
      ArmSubsystemNew.disable();
    }
    SmartDashboard.putNumber("ARM/setpoint", position);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    ArmSubsystemNew.disable();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
