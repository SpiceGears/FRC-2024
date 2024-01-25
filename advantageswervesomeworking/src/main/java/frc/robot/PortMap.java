package frc.robot;

public class PortMap {

  // TODO set correct ports
  public class Intake {

    public static final int INTAKE_MASTER_PORT = 7;
    public static final int INTAKE_SLAVE_PORT = 0;
    public static final int INTAKE_SENSOR_PORT = 0; // DIO port    
  }


  public class Shooter {

    public static final int SHOOTER_MASTER_PORT = 9;
    public static final int SHOOTER_SLAVE_PORT = 8;  
  }
  

  public class Swerve {

    public static final int SWERVE_DRIVE_FL = 7;
    public static final int SWERVE_DRIVE_FR = 1;
    public static final int SWERVE_DRIVE_BL = 5;
    public static final int SWERVE_DRIVE_BR = 3;

    public static final int SWERVE_TURN_FL = 8;
    public static final int SWERVE_TURN_FR = 2;
    public static final int SWERVE_TURN_BL = 6;
    public static final int SWERVE_TURN_BR = 4;

    public static final int ABSOLUTE_ENCODER_FL = 3;
    public static final int ABSOLUTE_ENCODER_FR = 0;  
    public static final int ABSOLUTE_ENCODER_BL = 2;  
    public static final int ABSOLUTE_ENCODER_BR = 1;  
  }
  
}
