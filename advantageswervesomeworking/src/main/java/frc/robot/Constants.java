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

    public static final double INTAKING_POWER = 0.5; // how much power to use when intaking notes
    public static final double PASSING_POWER = 0.5; // how much power to use when passing to shooter
    public static final double PASSING_TIME = 0.7; // (seconds) for how long intake rolls when passing
  }

  public static class Shooter {

    public static final double SHOOTING_POWER = 1.0; // how much power to use when shooting
    public static final double SHOOTER_SPEEDUP_DELAY = 0.5; // (seconds) how much time between speeding up shooter and passing
  }

  public static class Swerve {

    public static final double ROBOT_MAX_SPEED = 4.5; // maters/s | domyslnie Units.feetToMeters(14.5);
    public static final double ROBOT_TRACK_WIDTH_X = 0.70; // meters
    public static final double ROBOT_TRACK_WIDTH_Y = 0.70; // meters
    public static final double WHEEL_RADIUS = Units.inchesToMeters(2.0); // radius - promień
    public static final double DRIVE_GEAR_RATIO = (50.0 / 14.0) * (17.0 / 27.0) * (45.0 / 15.0); // 6.75:1 == L2 ratio
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
        //TODO tune in 
        public static final double DRIVE_FF_kS = 0;
        public static final double DRIVE_FF_kV = 0;
  
        //TODO tune in 
        public static final double DRIVE_PID_kP = 0.5;
        public static final double DRIVE_PID_kI = 0;
        public static final double DRIVE_PID_kD = 0;
        
        //TODO tune in 
        public static final double TURN_PID_kP = 0.5;
        public static final double TURN_PID_kI = 0;
        public static final double TURN_PID_kD = 0;
      }

    }
  }
}
