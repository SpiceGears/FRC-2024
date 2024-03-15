// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.intake;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.motorcontrol.Talon;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.PortMap;

public class IntakeSubsystem extends SubsystemBase {

  public Talon intakeMaster;
  // public Talon intakeSlave;
  public DigitalInput intakeSensor; // ir digital obstacle avoidance sensor
  public boolean isNoteInside;

  public enum IntakeState {
    INTAKING,
    BACKING,
    READY,
    EMPTY // empty when shot out and before trying to intake
  }

  public IntakeState intakeState;
  public boolean isIntakeIntaking;
  public boolean isIntakePassing;

  public IntakeSubsystem() {

    intakeMaster = new Talon(PortMap.Intake.INTAKE_MASTER_PORT);
    intakeSensor = new DigitalInput(PortMap.Intake.INTAKE_SENSOR_PORT);

    intakeMaster.setInverted(false);

    intakeState = IntakeState.EMPTY;
    isIntakeIntaking = false;
    isIntakePassing = false;
    isNoteInside = false;
  }

  @Override
  public void periodic() {
    logIntakeValues();
  }

  /**
   * Roll intake with desired power
   *
   * @param power power in range from -1 to 1
   */
  public void setIntakePower(double power) {
    intakeMaster.set(power);
    isIntakeIntaking = Math.abs(power) > 0.1;
    // intakeSlave.set(power);
  }

  public void stopIntake() {
    intakeMaster.stopMotor();
    // intakeSlave.stopMotor();
  }

  public double getIntakePower() {
    return intakeMaster.get();
  }

  public void setIntakeVolts(double volts) {
    intakeMaster.setVoltage(volts);
    // intakeSlave.setVoltage(volts);
  }

  public boolean checkForNoteInside() {
    isNoteInside = !intakeSensor.get();
    return isNoteInside;
  }

  private void logIntakeValues() {
    SmartDashboard.putNumber("intake/speed", getIntakePower());
    SmartDashboard.putBoolean("intake/isNoteInside", checkForNoteInside());
    SmartDashboard.putString("intake/state", intakeState.name());
    SmartDashboard.putBoolean("intake/isIntakeIntaking", isIntakeIntaking);
    SmartDashboard.putBoolean("intake/isIntakePassing", isIntakePassing);
    SmartDashboard.updateValues();
  }
}
