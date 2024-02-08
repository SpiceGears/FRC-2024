package frc.robot.subsystems.limelight;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;


public class LimelightSubsystem extends SubsystemBase {

    public LimelightSubsystem() {}

    NetworkTable table = NetworkTableInstance.getDefault().getTable("limelight");
    NetworkTableEntry txNetworkTableEntry = table.getEntry("tx");
    NetworkTableEntry tyNetworkTableEntry = table.getEntry("ty");
    NetworkTableEntry taNetworkTableEntry = table.getEntry("ta");

    @Override
    public void periodic() {
        //read values periodically
        double tx = txNetworkTableEntry.getDouble(0.0);
        double ty = tyNetworkTableEntry.getDouble(0.0);
        double ta = taNetworkTableEntry.getDouble(0.0);

        //post to smart dashboard periodically
        SmartDashboard.putNumber("LimelightX", tx);
        SmartDashboard.putNumber("LimelightY", ty);
        SmartDashboard.putNumber("LimelightArea", ta);
    }

    // public double getTx()

  
}
