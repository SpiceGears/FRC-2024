// Copyright 2021-2024 FRC 6328
// http://github.com/Mechanical-Advantage
//
// This program is free software; you can redistribute it and/or
// modify it under the terms of the GNU General Public License
// version 3 as published by the Free Software Foundation or
// available in the root directory of this project.
//
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
// GNU General Public License for more details.

package frc.robot.subsystems.drive;

import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.RobotController;
import frc.robot.Constants;
import frc.robot.PortMap;

/**
 * Module IO implementation for SparkMax drive motor controller, SparkMax turn motor controller (NEO
 * or NEO 550), and analog absolute encoder connected to the RIO
 *
 * <p>NOTE: This implementation should be used as a starting point and adapted to different hardware
 * configurations (e.g. If using a CANcoder, copy from "ModuleIOTalonFX")
 *
 * <p>To calibrate the absolute encoder offsets, point the modules straight (such that forward
 * motion on the drive motor will propel the robot forward) and copy the reported values from the
 * absolute encoders using AdvantageScope. These values are logged under
 * "/Drive/ModuleX/TurnAbsolutePositionRad"
 */
public class ModuleIOSparkMax implements ModuleIO {
  // Gear ratios for SDS MK4i L2, adjust as necessary
  private static final double DRIVE_GEAR_RATIO = Constants.Swerve.DRIVE_GEAR_RATIO;
  private static final double TURN_GEAR_RATIO = Constants.Swerve.TURN_GEAR_RATIO;

  private final CANSparkMax driveSparkMax;
  private final CANSparkMax turnSparkMax;

  private final RelativeEncoder driveEncoder;
  private final RelativeEncoder turnRelativeEncoder;
  private final AnalogInput turnAbsoluteEncoder;

  private final boolean isTurnMotorInverted = true;
  private final Rotation2d absoluteEncoderOffset;

  public ModuleIOSparkMax(int index) {
    switch (index) {
        // Bateria to front
      case 1: // FRONT RIGHT
        driveSparkMax = new CANSparkMax(PortMap.Swerve.SWERVE_DRIVE_FR, MotorType.kBrushless);
        turnSparkMax = new CANSparkMax(PortMap.Swerve.SWERVE_TURN_FR, MotorType.kBrushless);
        turnAbsoluteEncoder = new AnalogInput(PortMap.Swerve.ABSOLUTE_ENCODER_FR);
        // 180 - 3.141592
        // 90 - 3.141592 / 2
        // 1 - 0.01745
        absoluteEncoderOffset =
            new Rotation2d(Constants.Swerve.ABSOLUTE_ENCODER_OFFSET_FR); // CALIBRATED
        break;
      case 3: // BACK RIGHT
        driveSparkMax = new CANSparkMax(PortMap.Swerve.SWERVE_DRIVE_BR, MotorType.kBrushless);
        turnSparkMax = new CANSparkMax(PortMap.Swerve.SWERVE_TURN_BR, MotorType.kBrushless);
        turnAbsoluteEncoder = new AnalogInput(PortMap.Swerve.ABSOLUTE_ENCODER_BR);
        absoluteEncoderOffset =
            new Rotation2d(Constants.Swerve.ABSOLUTE_ENCODER_OFFSET_BR); // CALIBRATED
        break;
      case 2: // BACK LEFT
        driveSparkMax = new CANSparkMax(PortMap.Swerve.SWERVE_DRIVE_BL, MotorType.kBrushless);
        turnSparkMax = new CANSparkMax(PortMap.Swerve.SWERVE_TURN_BL, MotorType.kBrushless);
        turnAbsoluteEncoder = new AnalogInput(PortMap.Swerve.ABSOLUTE_ENCODER_BL);
        absoluteEncoderOffset =
            new Rotation2d(Constants.Swerve.ABSOLUTE_ENCODER_OFFSET_BL); // CALIBRATED
        break;
      case 0: // FRONT LEFT
        driveSparkMax = new CANSparkMax(PortMap.Swerve.SWERVE_DRIVE_FL, MotorType.kBrushless);
        turnSparkMax = new CANSparkMax(PortMap.Swerve.SWERVE_TURN_FL, MotorType.kBrushless);
        turnAbsoluteEncoder = new AnalogInput(PortMap.Swerve.ABSOLUTE_ENCODER_FL);
        absoluteEncoderOffset =
            new Rotation2d(Constants.Swerve.ABSOLUTE_ENCODER_OFFSET_FL); // CALIBRATED
        break;
      default:
        throw new RuntimeException("Invalid module index");
    }

    driveSparkMax.restoreFactoryDefaults();
    turnSparkMax.restoreFactoryDefaults();

    driveSparkMax.setCANTimeout(250);
    turnSparkMax.setCANTimeout(250);

    driveEncoder = driveSparkMax.getEncoder();
    turnRelativeEncoder = turnSparkMax.getEncoder();

    turnSparkMax.setInverted(isTurnMotorInverted);
    driveSparkMax.setSmartCurrentLimit(40);
    turnSparkMax.setSmartCurrentLimit(30);
    driveSparkMax.enableVoltageCompensation(12.0);
    turnSparkMax.enableVoltageCompensation(12.0);

    driveEncoder.setPosition(0.0);
    driveEncoder.setMeasurementPeriod(10);
    driveEncoder.setAverageDepth(2);

    turnRelativeEncoder.setPosition(0.0);
    turnRelativeEncoder.setMeasurementPeriod(10);
    turnRelativeEncoder.setAverageDepth(2);

    driveSparkMax.setCANTimeout(0);
    turnSparkMax.setCANTimeout(0);

    driveSparkMax.burnFlash();
    turnSparkMax.burnFlash();
  }

  @Override
  public void updateInputs(ModuleIOInputs inputs) {
    inputs.drivePositionRad =
        Units.rotationsToRadians(driveEncoder.getPosition()) / DRIVE_GEAR_RATIO;
    inputs.driveVelocityRadPerSec =
        Units.rotationsPerMinuteToRadiansPerSecond(driveEncoder.getVelocity()) / DRIVE_GEAR_RATIO;
    inputs.driveAppliedVolts = driveSparkMax.getAppliedOutput() * driveSparkMax.getBusVoltage();
    inputs.driveCurrentAmps = new double[] {driveSparkMax.getOutputCurrent()};

    inputs.turnAbsolutePosition =
        new Rotation2d(
                turnAbsoluteEncoder.getVoltage() / RobotController.getVoltage5V() * 2.0 * Math.PI)
            .minus(absoluteEncoderOffset);
    inputs.turnPosition =
        Rotation2d.fromRotations(turnRelativeEncoder.getPosition() / TURN_GEAR_RATIO);
    inputs.turnVelocityRadPerSec =
        Units.rotationsPerMinuteToRadiansPerSecond(turnRelativeEncoder.getVelocity())
            / TURN_GEAR_RATIO;
    inputs.turnAppliedVolts = turnSparkMax.getAppliedOutput() * turnSparkMax.getBusVoltage();
    inputs.turnCurrentAmps = new double[] {turnSparkMax.getOutputCurrent()};
  }

  @Override
  public void setDriveVoltage(double volts) {
    driveSparkMax.setVoltage(volts / 2); // TODO testing if turn jerks still with low Volts
  }

  @Override
  public void setTurnVoltage(double volts) {
    turnSparkMax.setVoltage(volts / 2); // TODO testing if turn jerks still with low Volts
  }

  @Override
  public void setDriveBrakeMode(boolean enable) {
    driveSparkMax.setIdleMode(enable ? IdleMode.kBrake : IdleMode.kCoast);
  }

  @Override
  public void setTurnBrakeMode(boolean enable) {
    turnSparkMax.setIdleMode(enable ? IdleMode.kBrake : IdleMode.kCoast);
  }
}
