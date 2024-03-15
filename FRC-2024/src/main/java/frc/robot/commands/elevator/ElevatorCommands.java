package frc.robot.commands.elevator;

// Copyright 2021-2024 FRC 6328
// http://github.com/Mechanical-Advantage
//
// This program is free software; you can redistribute it and/or
// modify it under the terms of the GNU General Public License
// version 3 as published by the Free Software Foundation or
// available in the root directory of this project.
//
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
// GNU General Public License for more details.

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.subsystems.elevator.ElevatorSubsystem;
import java.util.function.DoubleSupplier;

public class ElevatorCommands {
  private static final double DEADBAND = 0.1;

  /**
   * Field relative drive command using two joysticks (controlling linear and angular velocities).
   */
  public static Command elevatorControl(
      ElevatorSubsystem elevator,
      DoubleSupplier leftElevatorSupplier,
      DoubleSupplier rightElevatorSupplier) {
    return Commands.run(
        () -> {
          double leftElevatorSpeed =
              MathUtil.applyDeadband(leftElevatorSupplier.getAsDouble(), DEADBAND);
          double rightElevatorSpeed =
              MathUtil.applyDeadband(rightElevatorSupplier.getAsDouble(), DEADBAND);
          elevator.setElevatorLeftPower(leftElevatorSpeed);
          elevator.setElevatorRightPower(rightElevatorSpeed);
        },
        elevator);
  }

  public static InstantCommand stopElevator(ElevatorSubsystem elevator) {
    return new InstantCommand(
        () -> {
          elevator.setElevatorBothPower(0);
        },
        elevator);
  }
}
