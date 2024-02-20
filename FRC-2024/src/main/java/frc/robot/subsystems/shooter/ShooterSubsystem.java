// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.shooter;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.PortMap;

public class ShooterSubsystem extends SubsystemBase {

  private static CANSparkMax shooterMaster;
  private static CANSparkMax shooterSlave;

  private static RelativeEncoder shooterEncoder;

  public static enum ShooterState {
    PID, MANUAL
  }
  public static ShooterState shooterState;
  public static double shooterSetpointRPM;
  public static boolean isShooterReadyToShoot;
  public static double shooterManualPower;

  public static PIDController shooterPIDController;

  public ShooterSubsystem() {

    shooterMaster = new CANSparkMax(PortMap.Shooter.SHOOTER_MASTER_PORT, MotorType.kBrushless);
    shooterMaster = new CANSparkMax(PortMap.Shooter.SHOOTER_SLAVE_PORT, MotorType.kBrushless);
    shooterMaster.setInverted(false);
    shooterSlave.setInverted(true);
    shooterSlave.follow(shooterMaster);

    shooterEncoder = shooterMaster.getEncoder();

    shooterPIDController = new PIDController(0, 0, 0);
    shooterPIDController.setTolerance(100); //? tolerance in RPM

    shooterState = ShooterState.MANUAL;
    isShooterReadyToShoot = false;
    shooterManualPower = 0;
    setShooterPIDSetpoint(0);
  }

  @Override
  public void periodic() {
  
    // puts power to shooter depending on shooterState
    switch (shooterState) {
      
      case PID:
        setShooterVolts(calculateShooterPIDOutput());
        if (shooterPIDController.atSetpoint()) {
          isShooterReadyToShoot = true;
        } else {
          isShooterReadyToShoot = false;
        }
        break;
      case MANUAL:
        setShoterManual(shooterManualPower);
        isShooterReadyToShoot = true;
        break;
    }

    logShooterValues();
  }

  /**
   * Set RPM setpoint to Shooter PID loop
   * @param setpointRPM RPM (rates per minute)
   */
  public void setShooterPIDSetpoint(double setpointRPM) {
    shooterState = ShooterState.PID;
    shooterPIDController.reset();
    shooterPIDController.setSetpoint(setpointRPM);
  }
  private double calculateShooterPIDOutput() {
    return shooterPIDController.calculate(shooterEncoder.getVelocity());
  }
  public void setShooterManual(double power) {
    shooterState = ShooterState.MANUAL;
    shooterPIDController.reset();
  }
  public boolean getReadyForShot() {
    return isShooterReadyToShoot;
  }
  
  private void setShoterManual(double power) {
    shooterMaster.set(power);
    shooterSlave.set(power);
  }

  private void setShooterVolts(double volts) {
    shooterMaster.setVoltage(volts);
    shooterSlave.setVoltage(volts);
  }

  private void logShooterValues() {
    SmartDashboard.putNumber("shooter/power", calculateShooterPIDOutput());
    SmartDashboard.updateValues();
  }
}
