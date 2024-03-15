// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.shooter;

import com.revrobotics.CANSparkBase.IdleMode;
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

  private static RelativeEncoder shooterEncoder;

  public static enum ShooterMode {
    PID,
    MANUAL
  }

  public static ShooterMode shooterMode;
  public static boolean isShooterReady = false;
  public static double shooterManualPower;

  private static PIDController shooterPIDController;

  public ShooterSubsystem() {

    shooterMaster = new CANSparkMax(PortMap.Shooter.SHOOTER_MASTER_PORT, MotorType.kBrushless);
    shooterMaster.restoreFactoryDefaults();
    // shooterMaster.setCANTimeout(250);
    // shooterMaster.setSmartCurrentLimit(40);
    // shooterMaster.enableVoltageCompensation(12.0);
    // shooterMaster.setCANTimeout(0);
    shooterMaster.setInverted(false);
    shooterMaster.setIdleMode(IdleMode.kCoast);
    // shooterMaster.burnFlash();

    shooterSlave = new CANSparkMax(PortMap.Shooter.SHOOTER_SLAVE_PORT, MotorType.kBrushless);
    shooterSlave.restoreFactoryDefaults();
    // shooterSlave.setCANTimeout(250);
    // shooterSlave.setSmartCurrentLimit(40);
    // shooterSlave.enableVoltageCompensation(12.0);
    // shooterSlave.setCANTimeout(0);
    shooterSlave.setInverted(false);
    shooterSlave.setIdleMode(IdleMode.kCoast);
    // shooterSlave.follow(shooterMaster);
    // shooterSlave.burnFlash();

    shooterEncoder = shooterMaster.getEncoder();
    shooterEncoder.setPosition(0.0);
    shooterEncoder.setMeasurementPeriod(10);
    shooterEncoder.setAverageDepth(2);

    shooterPIDController = new PIDController(0.02, 0.012, 0);
    shooterPIDController.setTolerance(50); // ? tolerance in RPM

    shooterMode = ShooterMode.MANUAL;

    isShooterReady = false;
    shooterManualPower = 0;
  }

  @Override
  public void periodic() {

    // puts power to shooter depending on shooterState

    switch (shooterMode) {
      case PID:
        double output_shooter = calculateShooterPIDOutput();

        if (shooterPIDController.getSetpoint() == 0 || output_shooter < 0) {
          output_shooter = 0;
        } else {
          output_shooter += 0;
        }

        setShooterVolts(output_shooter);
        // //System.out.println(
        //     "test|pidoputput= "
        //         + String.format("%.3f%n", output_shooter)
        //         + " goal "
        //         + shooterPIDController.getSetpoint()
        //         + " velocity "
        //         + String.format("%.3f%n", shooterEncoder.getVelocity()));

        break;
      case MANUAL:
        setShooterPower(shooterManualPower);
        isShooterReady = true;
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
    shooterMode = ShooterMode.PID;
    shooterPIDController.setSetpoint(setpointRPM);
  }

  public void setShooterManual(double power) {
    shooterMode = ShooterMode.MANUAL;
    shooterPIDController.reset();
    shooterManualPower = power;
  }

  public void resetPIDController() {
    shooterPIDController.reset();
  }

  private double calculateShooterPIDOutput() {
    return shooterPIDController.calculate(shooterEncoder.getVelocity());
  }

  private void setShooterPower(double power) {
    shooterMaster.set(power);
    shooterSlave.set(power);
  }

  private void setShooterVolts(double volts) {
    shooterMaster.setVoltage(volts);
    shooterSlave.setVoltage(volts);
  }

  private void logShooterValues() {
    SmartDashboard.putNumber("shooter/power", shooterMaster.getAppliedOutput());
    SmartDashboard.putNumber("shooter/encoder", shooterEncoder.getVelocity());
    SmartDashboard.putNumber("shooter/setpoint", shooterPIDController.getSetpoint());
    SmartDashboard.putString("shooter/mode", shooterMode.name());
    SmartDashboard.putNumber("test/pidoutput", calculateShooterPIDOutput());
    SmartDashboard.putBoolean("test/isShooterPIDMode", shooterMode.equals(ShooterMode.PID));
    SmartDashboard.putBoolean("shooter/isShooterReady", isShooterReady);

    SmartDashboard.updateValues();
  }
}
