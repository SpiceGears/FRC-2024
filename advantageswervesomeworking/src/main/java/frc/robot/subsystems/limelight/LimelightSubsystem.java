package frc.robot.subsystems.limelight;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import frc.robot.LimelightHelpers;


public class LimelightSubsystem extends SubsystemBase {

    public LimelightSubsystem() {}

    NetworkTable table = NetworkTableInstance.getDefault().getTable("limelight");
    NetworkTableEntry tx = table.getEntry("tx");
    NetworkTableEntry ty = table.getEntry("ty");
    NetworkTableEntry ta = table.getEntry("ta");
    double[] pose_target_space = NetworkTableInstance.getDefault().getTable("limelight").getEntry("camerapose_targetspace").getDoubleArray(new double[6]);

    @Override
    public void periodic() {
        //read values periodically
        double tx = tx.getDouble(0.0);
        double ty = ty.getDouble(0.0);
        double ta = ta.getDouble(0.0);

        //post to smart dashboard periodically
        SmartDashboard.putNumber("LimelightX", x);
        SmartDashboard.putNumber("LimelightY", y);
        SmartDashboard.putNumber("LimelightArea", area);

      
    }

    



  
}
