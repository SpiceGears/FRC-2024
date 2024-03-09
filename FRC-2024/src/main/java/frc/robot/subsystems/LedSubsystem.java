// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class LedSubsystem extends SubsystemBase {
  /** Creates a new LedSubsystem. */
  AddressableLED m_led;

  AddressableLEDBuffer m_ledBuffer;

  public LedSubsystem() {

    // Must be a PWM header, not MXP or DIO
    m_led = new AddressableLED(3);
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
  }
}
