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
    // ! TODO enable();
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

    setArmVolts(finalOutput);

    SmartDashboard.putNumber("ARM/finalOutput", finalOutput);
    SmartDashboard.putNumber("ARM/feedforwardOutput", feedforwardOutput);
    SmartDashboard.putNumber("ARM/pidOutput", pidOutput);
  }

  @Override
  // Executes periodically, use instead of periodic() because of @Override problems
  public double getMeasurement() {

    updateArmPosition();
    frontLimitSwitchState = isFrontLimitSwitchHit();
    backLimitSwitchState = isBackLimitSwitchHit();
    logArmValues();

    double measurement = 0;

    // ! TODO CODE TO UNCOMMENT AFTER CALIBRATING ARM ENCODER OFFSET
    // ! TODO armPosition.getDegrees should be 0 when paralell to ground.

    measurement = armPosition.getDegrees();

    // if(frontLimitSwitch.get()) {
    //   resetEncoder();
    //   wasArmReseted = true;
    // }

    // if(wasArmReseted){
    //   measurement = armEncoder.getDistance() + Constants.Arm.kArmOffsetRads;
    // } else {
    //   measurement = armEncoder.getDistance() + Constants.Arm.POSITION.VERTICAL;
    // }

    // SmartDashboard.putNumber("ARM/getMeasurement()", measurement);
    // SmartDashboard.putBoolean("CHECK/armNotReseted", !wasArmReseted);

    return measurement;
  }

  public void setArmVolts(double volts) {
    armMasterMotor.setVoltage(volts);
    armSlaveMotor.setVoltage(volts);
  }

  public void setArmPower(double power) {
    armMasterMotor.set(power);
    armSlaveMotor.set(power);
  }

  public boolean isFrontLimitSwitchHit() {
    return frontLimitSwitch.get();
  }

  public boolean isBackLimitSwitchHit() {
    return backLimitSwitch.get();
  }

  public void stopArm() {
    armMasterMotor.set(0);
    armSlaveMotor.set(0);
  }

  private static void updateArmPosition() {
    armPosition =
        new Rotation2d(armEncoder.getVoltage() / RobotController.getVoltage5V() * 2.0 * Math.PI)
            .minus(armEncoderOffset);
  }

  private static Rotation2d getArmPosition() {
    updateArmPosition();
    return armPosition;
  }

  public void logArmValues() {

    SmartDashboard.putNumber("arm/armposition [degrees]", getArmPosition().getDegrees());
    SmartDashboard.putBoolean("arm/frontLimitSwitch", frontLimitSwitchState);
    SmartDashboard.putBoolean("arm/backLimitSwitch", backLimitSwitchState);
  }
}
