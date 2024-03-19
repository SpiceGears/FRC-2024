// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.PortMap;
import frc.robot.subsystems.arm.ArmSubsystemNew;
import frc.robot.subsystems.intake.IntakeSubsystem;
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
  private int m_rainbowFirstPixelHue = 5;
  final double ERROR_THRESHOLD_X = 0.1; // Przykładowa wartość
  final double ERROR_THRESHOLD_Y = 2.0; // Przykładowa wartość

  // Stan dla LED_mode
  enum LEDMode {
    DEFAULT,
    GREEN,
    GREEN_BLINK_FAST,
    BLUE_BINK_FAST,
    RED_BLINK,
    RED,
    BLUE,
    RAINBOW,
    POLIZE,
    WHITE
  }

  enum LEDpart {
    ARM_VERTICAL,
    INTAKE_VERTICAL,
    RIGHT,
    LEFT
  }

  private double lastBlinkTime = 0; // Czas ostatniej zmiany stanu LED dla migania
  private boolean blinkState = false; // Stan migania: false - wyłączone, true - włączone
  private final double BLINK_INTERVAL =
      0.0400; // Interwał migania w milisekundach (10 razy na sekundę)
  private double brightness = 0.2; // Jasność LEDów jako procent (1.0 = 100%)

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
    // rainbow();
    m_led.start();
  }

  @Override
  public void periodic() {
    double armEncoder = SmartDashboard.getNumber("ARM/encoderoutput [degrees]", 0);
    double armSetpoint = SmartDashboard.getNumber("ARM/setpoint", 0);
    double limelightError = SmartDashboard.getNumber("LL/error", 0);
    double tv = SmartDashboard.getNumber("limelight/tv", 0);
    boolean isIntakeIntaking = SmartDashboard.getBoolean("intake/isIntakeIntaking", false);
    boolean isIntakePassing = SmartDashboard.getBoolean("intake/isIntakeIntaking", false);
    boolean isDriveByLL = SmartDashboard.getBoolean("drive.isDriveByLL", false);
    boolean isArmByLL = SmartDashboard.getBoolean("ARM/isArmByLL", false);
    boolean isShooterReady = SmartDashboard.putBoolean("shooter", false);
    // Stałe dla progu błędu
    String intake_state = SmartDashboard.getString("intake/state", "");
    LEDMode ledMode = LEDMode.DEFAULT;
    boolean isArmInPosition = armEncoder < 20;
    double errorToTargetX = limelightError;
    double errorToTargetY = armSetpoint - armEncoder;
    boolean seesTarget = tv == 1;
    boolean is_aimed =
        (Math.abs(errorToTargetX) < ERROR_THRESHOLD_X)
            && (Math.abs(errorToTargetY) < ERROR_THRESHOLD_Y)
            && seesTarget;
    ledMode = LEDMode.DEFAULT;
    if (isArmInPosition) {
      switch (intake_state) {
        case "READY":
          ledMode = LEDMode.GREEN;
          break;
        case "BACKING":
          ledMode = LEDMode.GREEN_BLINK_FAST;
          break;
        case "INTAKING":
          if (isIntakeIntaking) {
            ledMode = LEDMode.RED_BLINK;
          } else {
            ledMode = LEDMode.RED;
          }
          break;
        default:
          ledMode = LEDMode.WHITE;
          break;
      }
    } else {
      if (isArmByLL) {
        if (!seesTarget) {
          // nie widzi targetu
          ledMode = LEDMode.RED_BLINK;
        } else if (is_aimed) {
          ledMode = LEDMode.GREEN;

        } else {
          ledMode = LEDMode.BLUE_BINK_FAST;
        }
      } else {
        if (seesTarget) {
          ledMode = LEDMode.BLUE;
        } else {
          ledMode = LEDMode.DEFAULT;
        }
      }
    }
    SmartDashboard.putString("LED/led_state", ledMode.name());
    updateLEDs(ledMode);
  }

  int m_rainbowFirstPixelHueL = 0;

  public void rainbow() {
    for (int i = 0; i < m_ledBuffer.getLength(); i++) {
      final var hue = (m_rainbowFirstPixelHueL + (i * 180 / m_ledBuffer.getLength())) % 180;
      m_ledBuffer.setHSV(i, hue, 255, (int) (128 * brightness));
    }
    m_led.setData(m_ledBuffer);
  }

  private void updateLEDs(LEDMode mode) {
    double currentTime = Timer.getFPGATimestamp();

    switch (mode) {
      case GREEN:
        setAllLEDs(0, 255, 0);
        break;
      case POLIZE:
        if (currentTime - lastBlinkTime >= BLINK_INTERVAL) {
          blinkState = !blinkState;
          lastBlinkTime = currentTime;

          if (blinkState) {
            if (mode == LEDMode.GREEN_BLINK_FAST) {
              setAllLEDs(0, 255, 0);
            } else if (mode == LEDMode.BLUE_BINK_FAST) {
              setAllLEDs(0, 0, 255);
            } else {
              setAllLEDs(255, 0, 0);
            }
          } else {
            setAllLEDs(0, 0, 0);
          }
        }
        break;
      case BLUE_BINK_FAST:
      case GREEN_BLINK_FAST:
      case RED_BLINK:
        if (currentTime - lastBlinkTime >= BLINK_INTERVAL) {
          blinkState = !blinkState;
          lastBlinkTime = currentTime;

          if (blinkState) {
            if (mode == LEDMode.GREEN_BLINK_FAST) {
              setAllLEDs(0, 255, 0);
            } else if (mode == LEDMode.BLUE_BINK_FAST) {
              setAllLEDs(0, 0, 255);
            } else {
              setAllLEDs(255, 0, 0);
            }
          } else {
            setAllLEDs(0, 0, 0);
          }
        }
        break;
      case RED:
        setAllLEDs(255, 0, 0);
        break;
      case BLUE:
        setAllLEDs(0, 0, 255);
        break;
      case WHITE:
        setAllLEDs(50, 50, 50);
        break;
      case RAINBOW:
      case DEFAULT:
        if (currentTime - lastBlinkTime > 0.002056) {
          m_rainbowFirstPixelHueL += 2;
          m_rainbowFirstPixelHueL %= 180;
          lastBlinkTime = currentTime;
        }
        rainbow();
        break;
    }
  }

  private void setLedPart(LEDpart ledPart, int red, int green, int blue) {
    red = adjustBrightness(red, brightness);
    green = adjustBrightness(green, brightness);
    blue = adjustBrightness(blue, brightness);

    switch (ledPart) {
      case RIGHT:
        for (var i = 0; i < 24; i++) {
          m_ledBuffer.setRGB(i, red, green, blue);
        }
        m_led.setData(m_ledBuffer);
        break;
      case ARM_VERTICAL:
        for (var i = 24; i < 24 + 30; i++) {
          m_ledBuffer.setRGB(i, red, green, blue);
        }
        m_led.setData(m_ledBuffer);
        break;
      case LEFT:
        for (var i = 24 + 30; i < 24 + 30 + 24; i++) {
          m_ledBuffer.setRGB(i, red, green, blue);
        }
        m_led.setData(m_ledBuffer);
        break;
      case INTAKE_VERTICAL:
        for (var i = 24 + 30 + 24; i < 24 + 30 + 24 + 23; i++) {
          m_ledBuffer.setRGB(i, red, green, blue);
        }
        m_led.setData(m_ledBuffer);
        break;
      default:
        break;
    }
  }

  private void setAllLEDs(int red, int green, int blue) {
    // Aplikowanie jasności do wartości RGB
    red = adjustBrightness(red, brightness);
    green = adjustBrightness(green, brightness);
    blue = adjustBrightness(blue, brightness);

    // setLedPart(LEDpart.ARM_VERTICAL, 100, 0, 0);
    // setLedPart(LEDpart.RIGHT, 0, 100, 0);
    // setLedPart(LEDpart.LEFT, 0, 0, 100);
    // setLedPart(LEDpart.INTAKE_VERTICAL, 100, 100, 100);

    for (var i = 0; i < m_ledBuffer.getLength(); i++) {
      m_ledBuffer.setRGB(i, red, green, blue);
    }
    m_led.setData(m_ledBuffer);
  }

  private int adjustBrightness(int colorValue, double brightness) {
    return (int) (colorValue * brightness);
  }
}
