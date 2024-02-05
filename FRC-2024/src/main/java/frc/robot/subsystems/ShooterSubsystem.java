// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.motorcontrol.Talon;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.PortMap;

public class ShooterSubsystem extends SubsystemBase {

  public Talon shooterMaster;
  public Talon shooterSlave;


  public ShooterSubsystem() {

    shooterMaster = new Talon(PortMap.INTA.SHOOTER_MASTER_PORT); 
    shooterSlave = new Talon(PortMap.INTA.SHOOTER_SLAVE_PORT);
    shooterMaster.setInverted(false);
    shooterSlave.setInverted(true);

  }

  @Override
  public void periodic() {

  }


  /**
   * Roll shoter with desired power
   * @param power power in range from -1 to 1
   */
  public void setShooterPower(double power) {
    shooterMaster.set(power);
    shooterSlave.set(power);
  }
  public void stopShooter() {
    shooterMaster.stopMotor();
    shooterSlave.stopMotor();
  }
  public double getShooterPower() {
    return shooterMaster.get();
  }

  public void setShooterVolts(double volts) {
    shooterMaster.setVoltage(volts);
    shooterSlave.setVoltage(volts);
  }

}
