// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.arm;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.arm.ArmSubsystemNew;
import frc.robot.subsystems.arm.ArmSubsystemNew.ArmState;
import java.util.function.DoubleSupplier;

public class SetArmJoystick extends Command {
  /** Creates a new SetArmLimelight. */
  private ArmSubsystemNew ArmSubsystemNew;

  private double value;
  private DoubleSupplier xSupplier;
  private double angle;

  public SetArmJoystick(ArmSubsystemNew ArmSubsystemNew, DoubleSupplier xSupplier) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.xSupplier = xSupplier;
    this.value = xSupplier.getAsDouble();
    this.ArmSubsystemNew = ArmSubsystemNew;
    addRequirements(ArmSubsystemNew);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    this.value = xSupplier.getAsDouble();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    this.value = xSupplier.getAsDouble();

    // if value >= 0:
    // value * 45 + 45
    // else if value < 0:
    // value * 30 + 45

    if (value >= 0) {
      angle = value * 45 + 45;
    } else {
      angle = value * 30 + 45;
    }
    ArmSubsystemNew.setArmState(ArmState.ENCODER);
    ArmSubsystemNew.setSetpoint(angle);

    // if (limelightSubsystem.getTvInt() == 1) {
    //   this.position = AimBot.interpolate(limelightSubsystem.getDistance())[0];
    //   ArmSubsystemNew.setArmState(ArmState.ENCODER);
    //   ArmSubsystemNew.setSetpoint(position);
    // } else {
    //   ArmSubsystemNew.disable();
    // }
    SmartDashboard.putNumber("ARM/joystickSetpoint", value);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    // ArmSubsystemNew.disable();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
