// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.shooter;

import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.PortMap;

public class ShooterSubsystem extends SubsystemBase {

  private static CANSparkMax shooterMaster;
  private static CANSparkMax shooterSlave;

  private static RelativeEncoder shooterEncoder; // ! TODO FOR NEO SHOOTER

  public static enum ShooterState {
    PID,
    MANUAL
  }

  public static ShooterState shooterState;
  public static double shooterSetpointRPM;
  public static boolean isShooterReadyToShoot;
  public static double shooterManualPower;

  public static PIDController shooterPIDController;

  public ShooterSubsystem() {

    // shooterMaster.restoreFactoryDefaults(); // TODO UNCOMMENT AFTER SPARK
    // shooterMaster.setCANTimeout(250);
    // shooterMaster.setSmartCurrentLimit(40);
    // shooterMaster.enableVoltageCompensation(12.0);
    // shooterMaster.setCANTimeout(0);
    // shooterMaster.setInverted(false);
    // shooterMaster =
    //     new CANSparkMax(
    //         PortMap.Shooter.SHOOTER_MASTER_PORT, MotorType.kBrushed); // ! TODO FOR NEO SHOOTER
    // shooterMaster.burnFlash();

    // shooterSlave.restoreFactoryDefaults();
    // shooterSlave.setCANTimeout(250);
    // shooterSlave.setSmartCurrentLimit(40);
    // shooterSlave.enableVoltageCompensation(12.0);
    // shooterSlave.setCANTimeout(0);
    // shooterSlave.setInverted(true);
    // shooterSlave.follow(shooterMaster); // TODO UNCOMMENT AFTER SPARK
    shooterSlave = new CANSparkMax(PortMap.Shooter.SHOOTER_SLAVE_PORT, MotorType.kBrushed);
    // shooterSlave.burnFlash();

    // shooterEncoder.setPosition(0.0); // TODO UNCOMMENT AFTER SPARK
    // shooterEncoder.setMeasurementPeriod(10);
    // shooterEncoder.setAverageDepth(2);

    // shooterEncoder = shooterMaster.getEncoder(); // TODO UNCOMMENT AFTER SPARK

    shooterPIDController = new PIDController(0, 0, 0);
    shooterPIDController.setTolerance(100); // ? tolerance in RPM

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
        setShooterVolts(calculateShooterPIDOutput()); // ! TODO FOR NEO SHOOTER
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
   *
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
    // shooterMaster.set(power); //TODO
    shooterSlave.set(power);
  }

  private void setShooterVolts(double volts) {
    // shooterMaster.setVoltage(volts); //TODO
    shooterSlave.setVoltage(volts);
  }

  private void logShooterValues() {
    SmartDashboard.putNumber("shooter/power", calculateShooterPIDOutput());
    SmartDashboard.updateValues();
  }
}
