package frc.robot.subsystems;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.LimelightHelpers;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;

public class LimelightSubsystem extends SubsystemBase {

    public LimelightSubsystem() {}

    


    



    NetworkTable table = NetworkTableInstance.getDefault().getTable("limelight");
    NetworkTableEntry tx = table.getEntry("tx");
    NetworkTableEntry ty = table.getEntry("ty");
    NetworkTableEntry ta = table.getEntry("ta");

    @Override
    public void periodic() {
        //read values periodically
        double x = tx.getDouble(0.0);
        double y = ty.getDouble(0.0);
        double area = ta.getDouble(0.0);

        //post to smart dashboard periodically
        SmartDashboard.putNumber("LimelightX", x);
        SmartDashboard.putNumber("LimelightY", y);
        SmartDashboard.putNumber("LimelightArea", area);

      
    }

    
    public NetworkTable getTable() {
        return table;
    }

    public void setTable(NetworkTable table) {
        this.table = table;
    }

    public NetworkTableEntry getTx() {
        return tx;
    }

    public void setTx(NetworkTableEntry tx) {
        this.tx = tx;
    }

    public NetworkTableEntry getTy() {
        return ty;
    }

    public void setTy(NetworkTableEntry ty) {
        this.ty = ty;
    }

    public NetworkTableEntry getTa() {
        return ta;
    }

    public void setTa(NetworkTableEntry ta) {
        this.ta = ta;
    }

    public double[] getPoseTargetSpace() {
        double[] pose_target_space = NetworkTableInstance.getDefault().getTable("limelight").getEntry("camerapose_targetspace").getDoubleArray(new double[6]);
       System.out.println(pose_target_space);
        return pose_target_space;
       //double[] pose_TargetSpace = LimelightHelpers.getBotPose_TargetSpace(null);
       //return pose_TargetSpace;
    }
}
