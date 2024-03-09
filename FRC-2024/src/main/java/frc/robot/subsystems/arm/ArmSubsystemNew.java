// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.arm;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.motorcontrol.VictorSP;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.PIDSubsystem;
import frc.robot.Constants;
import frc.robot.PortMap;

public class ArmSubsystemNew extends PIDSubsystem {
  /** Creates a new ArmSubsystemNewNew. */
  private static final VictorSP armMasterMotor = new VictorSP(PortMap.Arm.MASTER_PORT);

  private static final VictorSP armSlaveMotor = new VictorSP(PortMap.Arm.SLAVE_PORT);

  public static enum ArmState {
    ENCODER,
    MANUAL
  }

  public static double manualPower = 0;

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

  public ArmSubsystemNew() {
    super(
        // The PIDController used by the subsystem
        new PIDController(1.92, 0.00, 0));
  }

  public static Rotation2d getArmPosition() {
    updateArmPosition();
    return armPosition;
  }

  /** Get latest arm position reading and set it for armPosition */
  private static void updateArmPosition() {
    armPosition =
        new Rotation2d(-armEncoder.getVoltage() / RobotController.getVoltage5V() * 2.0 * Math.PI)
            .minus(armEncoderOffset);
  }

  public void logArmValues() {
    SmartDashboard.putNumber("arm/armpowermaster", armMasterMotor.get());
    SmartDashboard.putNumber("arm/armopowerslave", armSlaveMotor.get());
    SmartDashboard.putNumber("arm/encoderoutput [degrees]", armPosition.getDegrees());
    SmartDashboard.putNumber("arm/encoder raw voltage", armEncoder.getVoltage());
  }

  @Override
  public void useOutput(double output, double setpoint) {

    // update values
    updateArmPosition();
    // Calculate the feedforward from the sepoint
    // Limit output voltage
    double maxVoltageUp = Constants.Arm.MAX_VOLTAGE_OUTPUT_UP;
    double maxVoltageDown = Constants.Arm.MAX_VOLTAGE_OUTPUT_DOWN;
    // Add the feedforward to the PID output to get the motor output
    double encoderStateOutput = MathUtil.clamp(-output, -maxVoltageUp, maxVoltageDown);

    switch (armState) {
      case ENCODER:
        setArmVolts(encoderStateOutput);
        break;

      case MANUAL:
        setArmPower(manualPower);
        break;
    }

    SmartDashboard.putNumber("ARM/encoderStateOutput", encoderStateOutput);
    SmartDashboard.putNumber("ARM/pidOutput", output);
    SmartDashboard.putNumber("ARM/setpoint", setpoint);
    SmartDashboard.putString("ARM/armstate", armState.name());
    SmartDashboard.putNumber("arm/armpowermaster", armMasterMotor.get());
    SmartDashboard.putNumber("arm/armopowerslave", armSlaveMotor.get());
  }

  private static void setArmVolts(double volts) {
    armMasterMotor.setVoltage(volts);
    armSlaveMotor.setVoltage(volts);
  }

  @Override
  public double getMeasurement() {
    double measurement = getArmPosition().getDegrees();

    logArmValues();
    return measurement;
  }

  public void setArmState(ArmState armState) {
    enable();
    ArmSubsystemNew.armState = armState;
  }

  public void setManualPower(double power) {
    armState = ArmState.MANUAL;
    manualPower = power;
    System.out.println("manual power set to " + power); // TODO
  }

  private static void setArmPower(double power) {
    armMasterMotor.set(power);
    armSlaveMotor.set(power);
  }
}
