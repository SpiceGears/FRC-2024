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

package frc.robot;

import edu.wpi.first.math.util.Units;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
  public static final Mode currentMode = Mode.REAL;

  public static enum Mode {
    /** Running on a real robot. */
    REAL,

    /** Running a physics simulator. */
    SIM,

    /** Replaying from a log file. */
    REPLAY
  }

  public static class Intake {

    public static final double INTAKING_VOLTS = 5; // how much power to use when intaking notes
    public static final double BACKING_VOLTS = -3.5; // how much power to use when intaking notes
    public static final double PASSING_VOLTS = 6; // how much power to use when passing to shooter
    public static final double PASSING_TIME = 1; // (seconds) for how long intake rolls when passing
  }

  public static class Shooter {

    public static final double SHOOTING_POWER = 1.0; // how much power to use when shooting
    public static final double SHOOTER_SPEEDUP_DELAY =
        0.5; // (seconds) how much time between speeding up shooter and passing

    public static final double DEFAULT_RPM = 4200;
  }

  public static class Arm {

    public static final double INTAKING_SETPOINT = 16; // angle for intaking
    public static final double MAX_SETPOINT = 90; // angle for shooting
    public static final double MIDDLE_SETPOINT = 40; // angle for shooting

    public static final double MANUAL_SPEED_UP = 0.5;
    public static final double MANUAL_SPEED_DOWN = -0.3;

    public static final double ENCODER_OFFSET_DEGREES = 51;
    public static final double PWM_TEST_POWER = 0.2;

    public static final double KP = 6.20;
    public static final double kD = 0;
    public static final double kI = 0;

    public static final double REDUCTION_CHAIN = 2.3125; // 32t -> 74t = 2.3125:1

    public static final double DEADZONE_LOW = 0;
    public static final double DEADZONE_HIGH = 100;

    public static final double SPEED_MULTIPLIER = 0.3;

    public static final double ENCODER_ANGLES_PER_ROTATION =
        360 / REDUCTION_CHAIN; // 1 rotation = 360 degrees
    public static final double ENCODER_TICK_RATE = 2048;
    public static final double ENCODER_MIN_RATE = 10;
    public static final int ENCODER_SAMPLES_TO_AVERAGE = 5; // Can be between 1 and 127
    public static final boolean ENCODER_REVERSE = true;

    public static final double kMaxVelocityRadPerSecond = Math.toRadians(30);
    public static final double kMaxAccelerationRadPerSecSquared = Math.toRadians(30);
    public static final double kArmOffsetRads = -0.3; // arm rest position counting from horizontal

    public static final double MAX_VOLTAGE_OUTPUT_UP = 8.0;
    public static final double MAX_VOLTAGE_OUTPUT_DOWN = 8.0;

    public static final double kSVolts = 0;
    public static final double kGVolts = 0;
    // 2.5v utrzymuje w horizontal, wyzej 2.5v to za duzo
    public static final double kVVoltSecondPerRad = 5;
    public static final double kAVoltSecondSquaredPerRad = 5;

    public class LIMELIGHT {
      public static final double LIMELIGHT_ANGLE = 25 //TODO check limelight angle from vertical position
      public static final double LIMELIGHT_HEIGHT = 2 //limelight height in inches TODO check limelight height

    }

    public class POSITION {
      public static final double INTAKE = -0.3;
      public static final double HORIZONTAL = 0.0;
      public static final double SECONDLEVEL = 0.7;
      public static final double VERTICAL = 1.4;
    }
  }

  public static class Swerve {

    public static final double SPEED_LIMELIGHT = 0.8;
    public static final double SPEED_BOOSTED = 1.0;
    public static final double SPEED_NOT_BOOSTED = 0.69;
    public static final double ROBOT_MAX_SPEED = 4.5;
    // Units.feetToMeters(14.5);
    public static final double ROBOT_TRACK_WIDTH_X = 0.70; // meters
    public static final double ROBOT_TRACK_WIDTH_Y = 0.70; // meters
    public static final double WHEEL_RADIUS = Units.inchesToMeters(2.0); // radius - promień
    public static final double DRIVE_GEAR_RATIO =
        (50.0 / 14.0) * (17.0 / 27.0) * (45.0 / 15.0); // 6.75:1 == L2 ratio
    public static final double TURN_GEAR_RATIO = 150.0 / 7.0; // default

    public static final double ABSOLUTE_ENCODER_OFFSET_FL = -2.05;
    public static final double ABSOLUTE_ENCODER_OFFSET_FR = 2.95;
    public static final double ABSOLUTE_ENCODER_OFFSET_BL = 2.75;
    public static final double ABSOLUTE_ENCODER_OFFSET_BR = -1.25;

    public static class DriveSettings {
      // driveFeedforward = new SimpleMotorFeedforward(0.0, 0.0);
      // driveFeedback = new PIDController(0.0, 0.0, 0.0);
      // turnFeedback = new PIDController(0.0, 0.0, 0.0);

      // CASE: REAL
      public class Real {
        public static final double DRIVE_FF_kS = 0;
        public static final double DRIVE_FF_kV = 0;

        public static final double DRIVE_PID_kP = 0.5;
        public static final double DRIVE_PID_kI = 0;
        public static final double DRIVE_PID_kD = 0;

        public static final double TURN_PID_kP = 0.5;
        public static final double TURN_PID_kI = 0;
        public static final double TURN_PID_kD = 0;
      }
    }
  }
}
