package frc.robot.subsystems.limelight;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
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
    double[] pose_target_space = NetworkTableInstance.getDefault().getTable("limelight").getEntry("camerapose_targetspace").getDoubleArray(new double[6]);

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


    public double getXTargetSpace() {
        double x_target_space = pose_target_space[0];
        return x_target_space;
    }

     public double getYTargetSpace() {
        double y_target_space = pose_target_space[1];
        return y_target_space;
    }

    public double getZTargetSpace() {
        double z_target_space = pose_target_space[2];
        return z_target_space;
    }
//hujighjkfgukdyfjtusfxd

}
