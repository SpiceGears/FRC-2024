package frc.robot.commands.Limelight;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.LimelightHelpers;
import frc.robot.PortMap;
import frc.robot.subsystems.LimelightSubsystem;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;



public class AimToSpeaker extends CommandBase {
    @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
    private final LimelightSubsystem m_subsystem;
    private final double desired_y = 0; //y+ w dół
    private final double desired_x = 0; //x+ w prawo
    private final double desired_a = 50; //ile ekranu zajmuje cel [%]
    private boolean isAuto = false;
    private double is_target = NetworkTableInstance.getDefault().getTable("limelight").getEntry("tv").getDouble(0); //czy jest apriltag
    private double tid = NetworkTableInstance.getDefault().getTable("limelight").getEntry("tid").getDouble(4); //id apriltaga
    

    
    
  
    /**
     * Creates a new Command.
     *
     * @param subsystem The subsystem used by this command.
     */
    public AimToSpeaker(LimelightSubsystem subsystem) {
      m_subsystem = subsystem;
      // Use addRequirements() here to declare subsystem dependencies.
      addRequirements(null);
    }
  
    // Called when the command is initially scheduled.
    @Override
    public void initialize() {}
  
    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
      SmartDashboard.putBoolean("IsAuto", isAuto);
      double tx = LimelightHelpers.getTX(null);
      double ty = LimelightHelpers.getTY(null);
      double ta = LimelightHelpers.getTA(null);

      double error_x = desired_x - tx;
      double error_y = desired_y - ty;
      double error_a = desired_a - ta;

      if(error_y != desired_y) {
        if(error_y < desired_y) {
          
        }


      }

    }
  
    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {}
  
    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
      return false;
    }
  }