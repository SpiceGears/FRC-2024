// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.arm;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.subsystems.arm.ArmSubsystemNew;
import frc.robot.subsystems.arm.ArmSubsystemNew.ArmState;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class SetArm extends InstantCommand {

  private final ArmSubsystemNew ArmSubsystemNew;
  private double position;

  /** Creates a new SetArmCommand. */
  public SetArm(ArmSubsystemNew ArmSubsystemNew, double setpointAngle) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.position = setpointAngle;
    this.ArmSubsystemNew = ArmSubsystemNew;
    addRequirements(ArmSubsystemNew);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    ArmSubsystemNew.setArmState(ArmState.ENCODER);
    ArmSubsystemNew.enable();
    ArmSubsystemNew.setSetpoint(position);
    SmartDashboard.putNumber("ARM/setpoint", position);
  }
}
