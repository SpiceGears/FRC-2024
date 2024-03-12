// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.PortMap;
import frc.robot.subsystems.arm.ArmSubsystemNew;
import frc.robot.subsystems.intake.IntakeSubsystem;
import frc.robot.subsystems.intake.IntakeSubsystem.IntakeState;
import frc.robot.subsystems.limelight.LimelightDriver;
import frc.robot.subsystems.shooter.ShooterSubsystem;

public class LedSubsystem extends SubsystemBase {
  /** Creates a new LedSubsystem. */
  private IntakeSubsystem intakeSubsystem;

  private ShooterSubsystem shooterSubsystem;
  private ArmSubsystemNew armSubsystemNew;
  private LimelightDriver limelightSubsystem;
  private AddressableLED m_led;
  private AddressableLEDBuffer m_ledBuffer;
  // ........

  public LedSubsystem(
      IntakeSubsystem intakeSubsystem,
      ShooterSubsystem shooterSubsystem,
      ArmSubsystemNew armSubsystem,
      LimelightDriver limelightSubsystem) {
    this.intakeSubsystem = intakeSubsystem;
    this.shooterSubsystem = shooterSubsystem;
    this.armSubsystemNew = armSubsystem;
    this.limelightSubsystem = limelightSubsystem;
    // .........

    // Must be a PWM header, not MXP or DIO
    m_led = new AddressableLED(PortMap.LED_PORT);

    // Reuse buffer
    // Default to a length of 60, start empty output
    // Length is expensive to set, so only set it once, then just update data
    m_ledBuffer = new AddressableLEDBuffer(24 + 30 + 24 + 23);
    for (var i = 0; i < m_ledBuffer.getLength(); i++) {
      // Sets the specified LED to the RGB values for red
      m_ledBuffer.setRGB(i, 25, 0, 0);
    }
    m_led.setLength(m_ledBuffer.getLength());

    // Set the data
    m_led.setData(m_ledBuffer);
    m_led.start();
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    for (var i = 0; i < m_ledBuffer.getLength(); i++) {
      m_ledBuffer.setRGB(i, 10, 10, 10); // white
    }

    if (intakeSubsystem.intakeState.equals(IntakeState.READY)) {
      for (var i = 0; i < m_ledBuffer.getLength(); i++) {
        m_ledBuffer.setRGB(i, 92, 1, 122); // purple
      }
    }
    if (intakeSubsystem.intakeState.equals(IntakeState.INTAKING)) {
      for (var i = 0; i < m_ledBuffer.getLength(); i++) {
        m_ledBuffer.setRGB(i, 122, 63, 0); // orange
      }
    }
    if (intakeSubsystem.intakeState.equals(IntakeState.READY)
        && shooterSubsystem.isShooterReady == true) {
      for (var i = 0; i < m_ledBuffer.getLength(); i++) {
        m_ledBuffer.setRGB(i, 0, 100, 0); // green
      }
    }
    // TODO LEDs based on auto path
    // TODO unicorn button

    // Set the data for this period
    m_led.setLength(m_ledBuffer.getLength());
    m_led.setData(m_ledBuffer);
    m_led.start();
  }
}
