// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Arm;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotContainer;
import frc.robot.subsystems.ArmSubsystem;


public class ResetArmStart extends CommandBase {



  
  private final ArmSubsystem armSubsystem;
  private double initTime;
  private double timeDelta = 0;
  
  /** Creates a new ResetArmAtStart. */

  public ResetArmStart() {


    // Use addRequirements() here to declare subsystem dependencies.
    armSubsystem = RobotContainer.armSubsystem;
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
    if(armSubsystem.isFrontLimitSwitchHit()) {
      return true;
    } else {
      return false;
    }
  }
}