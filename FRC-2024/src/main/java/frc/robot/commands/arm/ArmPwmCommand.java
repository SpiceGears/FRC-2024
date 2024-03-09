// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.arm;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.arm.ArmSubsystemNew;

public class ArmPwmCommand extends Command {

  ArmSubsystemNew ArmSubsystemNew;
  double power; // power that motors are set

  /** Creates a new ArmPwmCommand. */
  public ArmPwmCommand(ArmSubsystemNew ArmSubsystemNew, double power) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.ArmSubsystemNew = ArmSubsystemNew;
    this.power = power;
    addRequirements(ArmSubsystemNew);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    ArmSubsystemNew.setManualPower(power);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    ArmSubsystemNew.setManualPower(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
