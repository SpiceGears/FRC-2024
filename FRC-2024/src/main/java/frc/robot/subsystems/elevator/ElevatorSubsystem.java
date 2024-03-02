// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.elevator;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.motorcontrol.Talon;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.PortMap;
import frc.robot.subsystems.drive.GyroIO;
import frc.robot.subsystems.drive.GyroIOInputsAutoLogged;

public class ElevatorSubsystem extends SubsystemBase {
  /** Creates a new ElevatorSubsystem. */
  public Talon elevatorMotorLeft;

  public Talon elevatorMotorRight;
  public Encoder elevatorEncoderLeft;
  public Encoder elevatorEncoderRight;
  public DigitalInput elevatorLimitLeft;
  public DigitalInput elevatorLimitRight;
  public ElevatorMode elevatorState; // Elevator state/mode
  public double elevatorManualPower; // Power that is periodically sent to elevator motors when in MANUAL mode
  private static enum ElevatorMode {
    MANUAL, PID
  }
  private GyroIO gyro;
  private GyroIOInputsAutoLogged gyroInputs = new GyroIOInputsAutoLogged();
  private double rollError;
  private double correctRollPower;


  public static PIDController elevatorPIDController;

  public ElevatorSubsystem() {

    // Need to get gyro in different way, because we dont have gyro objecty in RobotContainer
    // public ElevatorSubsystem(GyroIO gyro) {
    // this.gyro = gyro;

    elevatorMotorLeft = new Talon(PortMap.Elevator.ELEVATOR_LEFT_PORT);
    elevatorMotorRight = new Talon(PortMap.Elevator.ELEVATOR_RIGHT_PORT);

    elevatorEncoderLeft = new Encoder(0, 1);
    elevatorEncoderLeft.setDistancePerPulse(1); //TODO find correct values for our rope roll
    elevatorEncoderRight = new Encoder(2, 3);
    elevatorEncoderRight.setDistancePerPulse(1); //TODO find correct values for our rope roll

    elevatorLimitLeft = new DigitalInput(0);
    elevatorLimitRight = new DigitalInput(1);

    elevatorState = ElevatorMode.MANUAL; // Starting elevator state/mode
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    elevatorEncoderLeft.get();
    elevatorEncoderRight.get();
    gyro.updateInputs(gyroInputs);

    this.rollError = gyroInputs.rollPosition.getDegrees();
    this.correctRollPower = elevatorPIDController.calculate(rollError);

    switch (elevatorState) {
      case MANUAL:
        setElevatorBothPower(elevatorManualPower);
        break;
    
      case PID:
        //TODO pid / setpoint controling

        break;
    }
    

    /* PSEUDOKOD NA WINDE

     * korekta = PID * rollError
     * 
     * leftMotor.setPower(korekta)
     * rightMotor.setPower(-korekta)
     * 
     */
    
    if (elevatorLimitLeft.get()) {
      elevatorEncoderLeft.reset();
    }
    if (elevatorLimitRight.get()) {
      elevatorEncoderRight.reset();
    }
  }

  public void ejectToSetPoint() {

  }

  public void setElevatorManual(double power) {
    elevatorState = ElevatorMode.MANUAL;
    elevatorManualPower = power;
  }

  public Encoder getEncoderLeft() {
    return elevatorEncoderLeft;
  }

  public Encoder getEncoderRight() {
    return elevatorEncoderRight;
  }

  private void setElevatorBothPower(double speed) {
    elevatorMotorLeft.set(speed);
    elevatorMotorRight.set(speed);
  }

  private void setElevatorLeftPower(double speed) {
    elevatorMotorLeft.set(speed);
  }

  private void setElevatorRightPower(double speed) {
    elevatorMotorRight.set(speed);
  }
}
