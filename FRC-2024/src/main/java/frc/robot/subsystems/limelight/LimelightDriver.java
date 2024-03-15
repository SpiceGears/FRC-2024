package frc.robot.subsystems.limelight;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class LimelightDriver {

  NetworkTable table = NetworkTableInstance.getDefault().getTable("limelight");

  double tx =
      table
          .getEntry("tx")
          .getDouble(
              0); // Horizontal Offset From Crosshair To Target (LL1: -27 degrees to 27 degrees /
  // LL2: -29.8 to 29.8 degrees)
  double ty =
      table
          .getEntry("ty")
          .getDouble(
              0); // Vertical Offset From Crosshair To Target (LL1: -20.5 degrees to 20.5 degrees /
  // LL2: -24.85 to 24.85 degrees)
  double ta = table.getEntry("ta").getDouble(0); // Target Area (0% of image to 100% of image)
  int tv =
      (int)
          table.getEntry("tv").getDouble(0); // Whether the limelight has any valid targets (0 or 1)
  double[] botpose =
      table
          .getEntry("botpose")
          .getDoubleArray(new double[6]); // Robot transform in field-space. Translation (X,Y,Z)
  // Rotation(Roll,Pitch,Yaw), total latency (cl+tl)

  double distanceFromLimelightToGoalCm;

  public void updateLimelightVariables() {

    // //System.out.println("updateLimelightVariables()");
    tx =
        table
            .getEntry("tx")
            .getDouble(
                0); // Horizontal Offset From Crosshair To Target (LL1: -27 degrees to 27 degrees /
    // LL2: -29.8 to 29.8 degrees)
    ty =
        table
            .getEntry("ty")
            .getDouble(
                0); // Vertical Offset From Crosshair To Target (LL1: -20.5 degrees to 20.5 degrees
    // / LL2: -24.85 to 24.85 degrees)
    ta = table.getEntry("ta").getDouble(0); // Target Area (0% of image to 100% of image)
    tv =
        (int)
            table
                .getEntry("tv")
                .getDouble(0); // Whether the limelight has any valid targets (0 or 1)
    botpose =
        table
            .getEntry("botpose")
            .getDoubleArray(new double[6]); // Robot transform in field-space. Translation (X,Y,Z)
    // Rotation(Roll,Pitch,Yaw), total latency (cl+tl)

    // how many degrees back is your limelight rotated from perfectly vertical?
    double limelightMountAngleDegrees = 33.5;

    // distance from the center of the Limelight lens to the floor
    double limelightLensHeightCm = 20.0;

    // distance from the target to the floor
    double goalHeightCm = 145; // to center of speaker apriltag

    double angleToGoalDegrees = limelightMountAngleDegrees + ty;
    double angleToGoalRadians = angleToGoalDegrees * (3.14159 / 180.0);

    // calculate distance
    distanceFromLimelightToGoalCm =
        (goalHeightCm - limelightLensHeightCm) / Math.tan(angleToGoalRadians);

    SmartDashboard.putNumber(
        "limelight/distanceFromLimelightToGoalCm", distanceFromLimelightToGoalCm);
    SmartDashboard.putNumber("limelight/tx", tx);
    SmartDashboard.putNumber("limelight/ty", ty);
    SmartDashboard.putNumber("limelight/tv", tv);
  }

  public double getTxDouble() {
    return tx;
  }

  public double getTyDouble() {
    return ty;
  }

  public double getTaDouble() {
    return ta;
  }

  public int getTvInt() {
    return tv;
  }

  /**
   * @return distance from LL to goal in Cm
   */
  public double getDistance() {
    return distanceFromLimelightToGoalCm;
  }

  public double[] getBotpose() {
    return botpose;
  }
}
