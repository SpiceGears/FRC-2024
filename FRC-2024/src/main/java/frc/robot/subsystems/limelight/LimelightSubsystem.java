package frc.robot.subsystems.limelight;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class LimelightSubsystem extends SubsystemBase {

  public LimelightSubsystem() {}

  NetworkTable table = NetworkTableInstance.getDefault().getTable("limelight");

  //   NetworkTableEntry tx = table.getEntry("tx");
  //   NetworkTableEntry ty = table.getEntry("ty");
  //   NetworkTableEntry ta = table.getEntry("ta");

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

  @Override
  public void periodic() {
    // read values periodically
    updateLimelightVariables();
  }

  private void updateLimelightVariables() {
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

  public double[] getBotpose() {
    return botpose;
  }
}
