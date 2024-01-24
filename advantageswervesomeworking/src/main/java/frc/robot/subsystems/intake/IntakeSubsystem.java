// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.intake;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.motorcontrol.Talon;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.PortMap;

public class IntakeSubsystem extends SubsystemBase {

  public Talon intakeMaster;
  // public Talon intakeSlave;
  public DigitalInput intakeSensor; // ir digital obstacle avoidance sensor
  public boolean isNoteInside;

  public IntakeSubsystem() {

    intakeMaster = new Talon(PortMap.Intake.INTAKE_MASTER_PORT);
    // intakeSlave = new Talon(PortMap.INTAKE_SLAVE_PORT);
    intakeSensor = new DigitalInput(PortMap.Intake.INTAKE_SENSOR_PORT);

    intakeMaster.setInverted(false);
    // intakeSlave.setInverted(true);

    isNoteInside = false;
  }

  @Override
  public void periodic() {}

  // /**
  //  * Roll intake when note is not inside
  //  */
  // public void intakeNotes() {
  //   if(checkForNoteInside() == true) {
  //     stopIntake();
  //   } else {
  //     setIntakePower(Constants.Intake.intakingPower);
  //   }
  // }

  // /**
  //  * Passes note to shooter by rolling intake for x seconds.
  //  */
  // public void passNoteToShooter() {
  //   double startTime = Timer.getFPGATimestamp();
  //   double endTime = startTime + Constants.Intake.passingTime;
  //   if (Timer.getFPGATimestamp() < endTime) {
  //     setIntakePower(Constants.Intake.passingPower);
  //   } else {
  //     stopIntake();
  //   }
  // }

  /**
   * Roll intake with desired power
   *
   * @param power power in range from -1 to 1
   */
  public void setIntakePower(double power) {
    intakeMaster.set(power);
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
    isNoteInside = intakeSensor.get();
    return isNoteInside;
  }
}
