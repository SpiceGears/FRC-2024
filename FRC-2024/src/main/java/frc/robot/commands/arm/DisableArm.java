// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.arm;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.arm.ArmSubsystem;

public class DisableArm extends Command {

  private final ArmSubsystem armSubsystem;

  /** Creates a new DisableArmCommand. */
  public DisableArm(ArmSubsystem armSubsystem) {

    // Use addRequirements() here to declare subsystem dependencies.
    this.armSubsystem = armSubsystem;
    addRequirements(armSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    System.out.println("> DisableArm()");
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    // Disables the PID control. Sets output to zero.
    armSubsystem.disable();
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return true;
  }
}
