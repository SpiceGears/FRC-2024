// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Arm;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotContainer;
import frc.robot.subsystems.ArmSubsystem;

public class DisableArm extends CommandBase {
  
  private final ArmSubsystem armSubsystem;
  
  /** Creates a new DisableArmCommand. */
  public DisableArm() {
    
    // Use addRequirements() here to declare subsystem dependencies.
    armSubsystem = RobotContainer.armSubsystem;
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