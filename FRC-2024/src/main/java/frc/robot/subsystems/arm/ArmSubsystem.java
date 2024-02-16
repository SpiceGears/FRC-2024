// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.arm;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import edu.wpi.first.wpilibj.motorcontrol.VictorSP;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.ProfiledPIDSubsystem;
import frc.robot.Constants;
import frc.robot.PortMap;

public class ArmSubsystem extends ProfiledPIDSubsystem {
  /** Creates a new ProfiledPIDSubsystem. */
  private final ArmFeedforward m_feedforward =
      new ArmFeedforward(
          Constants.Arm.kSVolts,
          Constants.Arm.kGVolts,
          Constants.Arm.kVVoltSecondPerRad,
          Constants.Arm.kAVoltSecondSquaredPerRad);

  private final VictorSP armMaster = new VictorSP(PortMap.Arm.MASTER_PORT);
  private final VictorSP armSlave = new VictorSP(PortMap.Arm.SLAVE_PORT);
  private boolean wasArmReseted;

  public MotorControllerGroup armGroup = new MotorControllerGroup(armMaster, armSlave);

  public AnalogInput armEncoder;
  public Rotation2d armPosition; // Rotation2d object that gets the value periodically
  // resolution of 1/4096*360 (~.09) degrees.
  public Rotation2d armEncoderOffset = Rotation2d.fromDegrees(Constants.Arm.ENCODER_OFFSET_DEGREES);

  public DigitalInput frontLimitSwitch = new DigitalInput(PortMap.Arm.FRONT_LIMIT_SWITCH);
  public DigitalInput backLimitSwitch = new DigitalInput(PortMap.Arm.BACK_LIMIT_SWITCH);

  public ArmSubsystem() {
    super(
        new ProfiledPIDController(
            Constants.Arm.KP,
            Constants.Arm.kI,
            Constants.Arm.kD,
            new TrapezoidProfile.Constraints(
                Constants.Arm.kMaxVelocityRadPerSecond,
                Constants.Arm.kMaxAccelerationRadPerSecSquared)),
        0);
    // wasArmReseted = false;

    armEncoder = new AnalogInput(PortMap.Arm.ENCODER_PORT); // connected to NAVX AI0 port
    updateArmPosition();
    // armPosition is Rotation2d value in radians of arm rotation

    // Start arm at rest in neutral position
    // setGoal(Constants.ARM.kArmOffsetRads);

    // System.out.println("> ArmSubsystem()");
    // SmartDashboard.putNumber("ARM/finalOutput", 0);
    // SmartDashboard.putNumber("ARM/feedforwardOutput", 0);
    // SmartDashboard.putNumber("ARM/pidOutput", 0);
    // setGoal(Constants.Arm.POSITION.VERTICAL);
    // enable();
  }

  @Override
  public void useOutput(double pidOutput, TrapezoidProfile.State setpoint) {

    updateArmPosition();
    // Calculate the feedforward from the sepoint
    double feedforwardOutput = m_feedforward.calculate(setpoint.position, setpoint.velocity);

    // Limit output voltage
    double maxVoltage = Constants.Arm.MAX_VOLTAGE_OUTPUT_UP;

    // Add the feedforward to the PID output to get the motor output
    double finalOutput = MathUtil.clamp(pidOutput + feedforwardOutput, -maxVoltage, maxVoltage);
    SmartDashboard.putNumber("ARM/finalOutput", finalOutput);
    SmartDashboard.putNumber("ARM/feedforwardOutput", feedforwardOutput);
    SmartDashboard.putNumber("ARM/pidOutput", pidOutput);

    // if(setpoint.position == Constants.Arm.POSITION.INTAKE)
    // {
    //   if(isFrontLimitSwitchHit()) {
    //     finalOutput = 0;
    //   } else if (setpoint.position > Constants.Arm.POSITION.HORIZONTAL) {
    //     finalOutput = finalOutput;
    //   } else {
    //     // TODO - Add start time and if sensor isnt change to true after 4s set value to 0
    //     finalOutput = -0.20;
    //   }
    // } else {
    //   // if(isFrontLimitSwitchHit() && finalOutput <= 0){
    //   //   stopArm();
    //   // } else{
    //     armGroup.setVoltage(finalOutput);
    //   // }
    // }
    // armGroup.setVoltage(finalOutput);
  }

  @Override
  // Executes periodically, use instead of periodic() because of @Override problems
  public double getMeasurement() {

    updateArmPosition();

    // if(frontLimitSwitch.get()) {
    //   resetEncoder();
    //   wasArmReseted = true;
    // }
    double measurement = 0;

    // if(wasArmReseted){
    //   measurement = armEncoder.getDistance() + Constants.Arm.kArmOffsetRads;
    // } else {
    //   measurement = armEncoder.getDistance() + Constants.Arm.POSITION.VERTICAL;
    // }

    // SmartDashboard.putNumber("ARM/getMeasurement()", measurement);
    // SmartDashboard.putBoolean("CHECK/armNotReseted", !wasArmReseted);
    logArm();

    return measurement;
  }

  public void setArmVolts(double volts) {
    armGroup.setVoltage(volts);
  }

  public void setArmPower(double power) {
    armGroup.set(power);
  }

  public boolean isFrontLimitSwitchHit() {
    return frontLimitSwitch.get();
  }

  public boolean isBackLimitSwitchHit() {
    return backLimitSwitch.get();
  }

  public void stopArm() {
    armGroup.set(0);
  }

  public void resetEncoder() {
    // armEncoder.reset();
    // System.out.println("> resetEncoder() [arm]");
  }

  private void updateArmPosition() {
    armPosition =
      new Rotation2d(armEncoder.getVoltage() / RobotController.getVoltage5V() * 2.0 * Math.PI)
          .minus(armEncoderOffset);
  }
  
  public double getEncoderPositionDegrees() {
    return armPosition.getDegrees();
  }

  public Rotation2d getEncoderPositionRotation2d() {
  return armPosition;
  }

  public void logArm() {

    SmartDashboard.putNumber("arm/encoder position in degrees", getEncoderPositionDegrees());

    // SmartDashboard.putNumber("ARM/getDistance()", armEncoder.getDistance());
    // SmartDashboard.putBoolean("isFrontLimitSwitchHit", isFrontLimitSwitchHit());
    // SmartDashboard.putNumber("ARM/set_position");

  }

}
