// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.arm;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.arm.ArmSubsystem;

public class ArmPwmCommand extends Command {

  ArmSubsystem armSubsystem;
  double power; // power that motors are set

  /** Creates a new ArmPwmCommand. */
  public ArmPwmCommand(ArmSubsystem armSubsystem, double power) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.armSubsystem = armSubsystem;
    this.power = power;
    addRequirements(armSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    // armSubsystem.disable();
    armSubsystem.manualPower = power;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    armSubsystem.manualPower = power;
    System.out.println(armSubsystem.manualPower);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    armSubsystem.manualPower = 0;
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
