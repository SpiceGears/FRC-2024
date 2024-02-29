package frc.robot;

public class PortMap {

  // TODO set correct ports
  public class Intake {

    public static final int INTAKE_MASTER_PORT = 2; // PWM
    public static final int INTAKE_SENSOR_PORT = 0; // DIO port
  }

  public class Shooter {

    // CAN ID
    public static final int SHOOTER_MASTER_PORT = 9;
    public static final int SHOOTER_SLAVE_PORT = 10;
  }

  public class Elevator {

    public static final int ELEVATOR_LEFT_PORT = 3;
    public static final int ELEVATOR_RIGHT_PORT = 4;
  }

  public class Swerve {

    // CAN ID
    public static final int SWERVE_DRIVE_FL = 7;
    public static final int SWERVE_DRIVE_FR = 1;
    public static final int SWERVE_DRIVE_BL = 5;
    public static final int SWERVE_DRIVE_BR = 3;

    // CAN ID
    public static final int SWERVE_TURN_FL = 8;
    public static final int SWERVE_TURN_FR = 2;
    public static final int SWERVE_TURN_BL = 6;
    public static final int SWERVE_TURN_BR = 4;

    // ANALOG
    public static final int ABSOLUTE_ENCODER_FL = 3;
    public static final int ABSOLUTE_ENCODER_FR = 0;
    public static final int ABSOLUTE_ENCODER_BL = 2;
    public static final int ABSOLUTE_ENCODER_BR = 1;
  }

  public class Arm {

    // PWM
    public static final int MASTER_PORT = 0;
    public static final int SLAVE_PORT = 1;

    public static final int ENCODER_PORT = 4; // ! NAVX2 AI0 PORT

    public static final int FRONT_LIMIT_SWITCH = 7;
    public static final int BACK_LIMIT_SWITCH = 8;
  }
}
