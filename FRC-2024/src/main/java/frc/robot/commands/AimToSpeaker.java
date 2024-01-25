package frc.robot.commands;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.LimelightHelpers;
import frc.robot.subsystems.LimelightSubsystem;


public class AimToSpeaker extends CommandBase {
    @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
    private final LimelightSubsystem m_subsystem;
    private final double desired_position_y = 0; //y+ downwards
    private final double desired_position_x = 0; //x+ right
    private final double desired_position_z = 10; //z+ pointing out of target centre
    

    
    
  
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
      final double current_position_x = 0;
      final double current_position_y = 0;
      final double current_position_z = 0;
      final double error_x = desired_position_x - current_position_x;
      final double error_y = desired_position_y - current_position_y;
      final double error_z = desired_position_z - current_position_z;

      if(error_x != 0) {
        if(current_position_x > desired_position_x) {
          if(current_position_x < desired_position_x) {



          }
          else{


          }

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