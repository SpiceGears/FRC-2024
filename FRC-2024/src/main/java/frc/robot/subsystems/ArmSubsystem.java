// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Encoder;
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
        Constants.ARM.kSVolts, Constants.ARM.kGVolts,
        Constants.ARM.kVVoltSecondPerRad, Constants.ARM.kAVoltSecondSquaredPerRad);
  
  private final VictorSP rightArmMaster = new VictorSP(PortMap.ARM.RIGHT_MASTER_PORT);
  private final VictorSP rightArmSlave = new VictorSP(PortMap.ARM.RIGHT_SLAVE_PORT);
  private boolean wasArmReseted;

  public MotorControllerGroup armGroup = new MotorControllerGroup(
     rightArmMaster,
     rightArmSlave
  );

  public Encoder armEncoder;

  public DigitalInput frontLimitSwitch = new DigitalInput(PortMap.ARM.FRONT_LIMIT);
  public DigitalInput backLimitSwitch = new DigitalInput(PortMap.ARM.BACK_LIMIT);

    public ArmSubsystem() {
      super(
        new ProfiledPIDController(
        Constants.ARM.KP,
        Constants.ARM.kI,
        Constants.ARM.kD,
        new TrapezoidProfile.Constraints(
          Constants.ARM.kMaxVelocityRadPerSecond,
          Constants.ARM.kMaxAccelerationRadPerSecSquared)),
          0);
        wasArmReseted = false;
        armEncoder = new Encoder(PortMap.ARM.ENCODER_PORT_A, PortMap.ARM.ENCODER_PORT_B);
        // armEncoder.setMaxPeriod(Constants.ARM.ENCODER_MIN_RATE); //TODO check if it works without it
        armEncoder.setReverseDirection(Constants.ARM.ENCODER_REVERSE);
        armEncoder.setSamplesToAverage(Constants.ARM.ENCODER_SAMPLES_TO_AVERAGE);
        armEncoder.setDistancePerPulse(Constants.ARM.kEncoderDistancePerPulse);

        // Start arm at rest in neutral position
        // setGoal(Constants.ARM.kArmOffsetRads);

        System.out.println("> ArmSubsystem()");
        SmartDashboard.putNumber("ARM/finalOutput", 0);
        SmartDashboard.putNumber("ARM/feedforwardOutput", 0);
        SmartDashboard.putNumber("ARM/pidOutput", 0);
        setGoal(Constants.ARM.POSITION.VERTICAL);
        enable();
        }
        
  @Override
  public void useOutput(double pidOutput, TrapezoidProfile.State setpoint) {

    // Calculate the feedforward from the sepoint
    double feedforwardOutput = m_feedforward.calculate(setpoint.position, setpoint.velocity);

    // Limit output voltage
    double maxVoltage = Constants.ARM.MAX_VOLTAGE_OUTPUT_UP;

    // Add the feedforward to the PID output to get the motor output
    double finalOutput = MathUtil.clamp(pidOutput + feedforwardOutput, -maxVoltage, maxVoltage);
    SmartDashboard.putNumber("ARM/finalOutput", finalOutput);
    SmartDashboard.putNumber("ARM/feedforwardOutput", feedforwardOutput);
    SmartDashboard.putNumber("ARM/pidOutput", pidOutput);
    
    if(setpoint.position == Constants.ARM.POSITION.INTAKE)
    { 
      if(isFrontLimitSwitchHit()) {
        finalOutput = 0;
      } else if (setpoint.position > Constants.ARM.POSITION.HORIZONTAL) {
        finalOutput = finalOutput;
      } else {
        // TODO - Add start time and if sensor isnt change to true after 4s set value to 0
        finalOutput = -0.20;
      }
    } else {
      // if(isFrontLimitSwitchHit() && finalOutput <= 0){
      //   stopArm();
      // } else{
        armGroup.setVoltage(finalOutput);
      // }
    }
    armGroup.setVoltage(finalOutput);
  }

  @Override
  // Executes periodically, use instead of periodic() because of @Override problems
  public double getMeasurement() {

    if(frontLimitSwitch.get()) {
      resetEncoder();
      wasArmReseted = true;
    }
    double measurement;

    if(wasArmReseted){
      measurement = armEncoder.getDistance() + Constants.ARM.kArmOffsetRads;
    } else {
      measurement = armEncoder.getDistance() + Constants.ARM.POSITION.VERTICAL;
    }

    SmartDashboard.putNumber("ARM/getMeasurement()", measurement);
    SmartDashboard.putBoolean("CHECK/armNotReseted", !wasArmReseted);
    logArm();

    return measurement;
  }

  public void setArmVolts(double volts) {
    armGroup.setVoltage(volts);
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
    armEncoder.reset();
    // System.out.println("> resetEncoder() [arm]");
  }

  public void logArm() {

    SmartDashboard.putNumber("ARM/getDistance()", armEncoder.getDistance());
    // SmartDashboard.putBoolean("isFrontLimitSwitchHit", isFrontLimitSwitchHit());
    // SmartDashboard.putNumber("ARM/set_position");

  }


}