// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.elevator;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.motorcontrol.Talon;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.PortMap;

public class ElevatorSubsystem extends SubsystemBase {
  /** Creates a new ElevatorSubsystem. */
  public Talon elevatorMotorLeft;

  public Talon elevatorMotorRight;
  public Encoder elevatorEncoderLeft;
  public Encoder elevatorEncoderRight;
  public DigitalInput elevatorLimitLeft;
  public DigitalInput elevatorLimitRight;

  public ElevatorSubsystem() {

    elevatorMotorLeft = new Talon(PortMap.Elevator.ELEVATOR_LEFT_PORT);
    elevatorMotorRight = new Talon(PortMap.Elevator.ELEVATOR_RIGHT_PORT);

    elevatorEncoderLeft = new Encoder(0, 1);
    elevatorEncoderLeft.setDistancePerPulse(1);
    elevatorEncoderRight = new Encoder(2, 3);
    elevatorEncoderRight.setDistancePerPulse(1);

    elevatorLimitLeft = new DigitalInput(0);
    elevatorLimitRight = new DigitalInput(1);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    elevatorEncoderLeft.get();
    elevatorEncoderRight.get();

    if (elevatorLimitLeft.get()) {
      elevatorEncoderLeft.reset();
    }
    if (elevatorLimitRight.get()) {
      elevatorEncoderRight.reset();
    }
  }

  public Encoder getEncoderLeft() {
    return elevatorEncoderLeft;
  }

  public Encoder getEncoderRight() {
    return elevatorEncoderRight;
  }

  public void setElevatorBothPower(double speed) {
    elevatorMotorLeft.set(speed);
    elevatorMotorRight.set(speed);
  }

  public void setElevatorLeftPower(double speed) {
    elevatorMotorLeft.set(speed);
  }

  public void setElevatorRightPower(double speed) {
    elevatorMotorRight.set(speed);
  }
}
