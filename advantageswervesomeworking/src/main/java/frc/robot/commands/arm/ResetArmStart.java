// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.arm;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.arm.ArmSubsystem;

public class ResetArmStart extends Command {

  private final ArmSubsystem armSubsystem;
  private double initTime;
  private double timeDelta = 0;

  /** Creates a new ResetArmAtStart. */
  public ResetArmStart(ArmSubsystem armSubsystem) {

    // Use addRequirements() here to declare subsystem dependencies.
    this.armSubsystem = armSubsystem;
    addRequirements(armSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    initTime = Timer.getFPGATimestamp();
    System.out.println("> ResetArmAtStart() started!");
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    // if <.2s then set(-2V)
    // if >.2s then set( 0V)
    // if frontlimit then end
    double output = -2;
    timeDelta = Timer.getFPGATimestamp() - initTime;
    if (timeDelta <= .2) { // if runs for less than .2s
      armSubsystem.setArmVolts(output);
    } else {
      armSubsystem.setArmVolts(0); // if runs for more than .2s
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {

    System.out.println("> ResetArmAtStart() ended!");
    armSubsystem.resetEncoder();
    armSubsystem.setArmVolts(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if (armSubsystem.isFrontLimitSwitchHit()) {
      return true;
    } else {
      return false;
    }
  }
}
