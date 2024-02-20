// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.arm;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.subsystems.arm.ArmSubsystem;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class SetArm extends InstantCommand {

  private final ArmSubsystem armSubsystem;
  private final double position;

  /** Creates a new SetArmCommand. */
  public SetArm(ArmSubsystem armSubsystem, double position) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.position = position;
    this.armSubsystem = armSubsystem;
    addRequirements(armSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    armSubsystem.enable();
    armSubsystem.setGoal(position);
  }
}
