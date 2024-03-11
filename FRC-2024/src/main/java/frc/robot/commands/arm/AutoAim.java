package frc.robot.commands.arm;
import java.lang.Math;

public class AutoAim extends Command {
    double ty = LimelightSubsystem.getTyDouble;

    double goalHeightInches = 6.125; //goal height in inch

    double angleToGoalDegrees = ty + Constants.LIMELIGHT.LIMELIGHT_ANGLE;

    double angleToGoalRadians = angleToGoalDegrees * (3.14159 / 180.0);

    double distanceToGoalInches = (goalHeightInches - limelightLensHeightInches) / Math.tan(angleToGoalRadians);
    //distance to goal in inch

    
}
