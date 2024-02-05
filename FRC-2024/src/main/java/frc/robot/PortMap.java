package frc.robot;

public class PortMap {

    public class JOYSTICK {

        public static final int DRIVER_JOYSTICK = 0;
        public static final int OPERATOR_JOYSTICK = 1;   
    
        }
        

    public class INTA{

    //TODO set correct ports
    public static final int INTAKE_MASTER_PORT = 7;
    public static final int INTAKE_SLAVE_PORT = 0;
    public static final int INTAKE_SENSOR_PORT = 0; // DIO port

    public static final int SHOOTER_MASTER_PORT = 9;
    public static final int SHOOTER_SLAVE_PORT = 8;

    public static final int LIMELIGHT_PORT = 5;
    


        public static final int LEFT_MASTER_PORT = 4;
        public static final int RIGHT_MASTER_PORT = 5;
 }
  public class ARM {

        public static final int LEFT_MASTER_PORT = 4;
        public static final int RIGHT_MASTER_PORT = 5;
        public static final int LEFT_SLAVE_PORT = 6;
        public static final int RIGHT_SLAVE_PORT = 7;

        public static final int ENCODER_PORT_A = 4;	
        public static final int ENCODER_PORT_B = 5;

        public static final int FRONT_LIMIT = 6;
        public static final int BACK_LIMIT = 7;
    }


    public static final int LIMELIGHT_PORT = 5;

}
