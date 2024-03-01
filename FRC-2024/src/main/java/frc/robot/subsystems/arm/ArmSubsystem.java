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
import edu.wpi.first.wpilibj.motorcontrol.VictorSP;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.ProfiledPIDSubsystem;
import frc.robot.Constants;
import frc.robot.PortMap;

public class ArmSubsystem extends ProfiledPIDSubsystem {
  /** Creates a new ProfiledPIDSubsystem. */
  private static final ArmFeedforward m_feedforward =
      new ArmFeedforward(
          Constants.Arm.kSVolts,
          Constants.Arm.kGVolts,
          Constants.Arm.kVVoltSecondPerRad,
          Constants.Arm.kAVoltSecondSquaredPerRad);

  private static final VictorSP armMasterMotor = new VictorSP(PortMap.Arm.MASTER_PORT);
  private static final VictorSP armSlaveMotor = new VictorSP(PortMap.Arm.SLAVE_PORT);

  public static enum ArmState {
    ENCODER,
    MANUAL
  }

  public static ArmState armState = ArmState.MANUAL;

  public static AnalogInput armEncoder =
      new AnalogInput(PortMap.Arm.ENCODER_PORT); // connected to NAVX AI0 port
  public static Rotation2d armEncoderOffset =
      Rotation2d.fromDegrees(Constants.Arm.ENCODER_OFFSET_DEGREES);
  public static Rotation2d armPosition =
      new Rotation2d(armEncoder.getVoltage() / RobotController.getVoltage5V() * 2.0 * Math.PI)
          .minus(
              armEncoderOffset); // Rotation2d object that gets the value periodically, resolution
  // of 1/4096*360 (~.09) degrees.

  public static DigitalInput frontLimitSwitch = new DigitalInput(PortMap.Arm.FRONT_LIMIT_SWITCH);
  public static DigitalInput backLimitSwitch = new DigitalInput(PortMap.Arm.BACK_LIMIT_SWITCH);

  public static boolean frontLimitSwitchState;
  public static boolean backLimitSwitchState;

  public static double manualPower = 0;

  public ArmSubsystem() {
    super(
        new ProfiledPIDController(
            Constants.Arm.KP,
            Constants.Arm.kI,
            Constants.Arm.kD,
            new TrapezoidProfile.Constraints(
                Constants.Arm.kMaxVelocityRadPerSecond,
                Constants.Arm.kMaxAccelerationRadPerSecSquared)),
        getArmPosition().getDegrees());

    updateArmPosition();
    enable();
  }

  // Runs when subsystem is enabled();
  @Override
  public void useOutput(double pidOutput, TrapezoidProfile.State setpoint) {

    // update values
    updateArmPosition();
    // Calculate the feedforward from the sepoint
    double feedforwardOutput = m_feedforward.calculate(setpoint.position, setpoint.velocity);
    // Limit output voltage
    double maxVoltage = Constants.Arm.MAX_VOLTAGE_OUTPUT_UP;
    // Add the feedforward to the PID output to get the motor output
    double encoderStateOutput =
        MathUtil.clamp(pidOutput + feedforwardOutput, -maxVoltage, maxVoltage);

    switch (armState) {
      case ENCODER:
        setArmVolts(encoderStateOutput);
        break;

      case MANUAL:
        setArmPower(manualPower);
        break;
    }

    SmartDashboard.putNumber("ARM/encoderStateOutput", encoderStateOutput);
    SmartDashboard.putNumber("ARM/feedforwardOutput", feedforwardOutput);
    SmartDashboard.putNumber("ARM/pidOutput", pidOutput);
    SmartDashboard.putNumber("arm/armpowermaster", armMasterMotor.get());
    SmartDashboard.putNumber("arm/armopowerslave", armSlaveMotor.get());
  }

  // Executes periodically, use instead of periodic() because of @Override problems
  @Override
  public double getMeasurement() {

    frontLimitSwitchState = getFrontLimitSwitchHit();
    backLimitSwitchState = getBackLimitSwitchHit();

    double measurement = getArmPosition().getDegrees();

    logArmValues();
    return measurement;
  }

  public void setManualPower(double power) {
    armState = ArmState.MANUAL;
    manualPower = power;
    System.out.println("manual power set to " + power); // TODO
  }

  public void setArmState(ArmState armState) {
    ArmSubsystem.armState = armState;
  }

  public static boolean getFrontLimitSwitchHit() {
    return frontLimitSwitch.get();
  }

  public static boolean getBackLimitSwitchHit() {
    return backLimitSwitch.get();
  }

  public static Rotation2d getArmPosition() {
    updateArmPosition();
    return armPosition;
  }

  private static void setArmVolts(double volts) {
    armMasterMotor.setVoltage(volts);
    armSlaveMotor.setVoltage(volts);
  }

  private static void setArmPower(double power) {
    armMasterMotor.set(power);
    armSlaveMotor.set(power);
  }

  /** Get latest arm position reading and set it for armPosition */
  private static void updateArmPosition() {
    armPosition =
        new Rotation2d(-armEncoder.getVoltage() / RobotController.getVoltage5V() * 2.0 * Math.PI)
            .minus(armEncoderOffset);
  }

  public void logArmValues() {

    SmartDashboard.putBoolean("arm/frontLimitSwitch", getFrontLimitSwitchHit());
    SmartDashboard.putBoolean("arm/backLimitSwitch", getBackLimitSwitchHit());
    SmartDashboard.putNumber("arm/armpowermaster", armMasterMotor.get());
    SmartDashboard.putNumber("arm/armopowerslave", armSlaveMotor.get());
    SmartDashboard.putNumber("arm/manualPower", manualPower);
    SmartDashboard.putNumber("arm/encoderoutput [degrees]", getArmPosition().getDegrees());
    SmartDashboard.putNumber("arm/encoder raw voltage", armEncoder.getVoltage());
  }
}
