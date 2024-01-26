// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.Constants.OperatorConstants;
import frc.robot.commands.Autos;
import frc.robot.commands.ExampleCommand;
import frc.robot.commands.Intake.IntakeNote;
import frc.robot.commands.Shooter.PassAndShootNote;
import frc.robot.commands.Swerve.SetZeroHeading;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.ExampleSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import java.util.List;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.XboxController.Button;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.OIConstants;
import frc.robot.commands.SwerveJoystickCmd;
import frc.robot.subsystems.SwerveSubsystem;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */

 
public class RobotContainer {
  // The robot's subsystems and commands are defined here...
  private final ExampleSubsystem exampleSubsystem = new ExampleSubsystem();
  private final IntakeSubsystem intakeSubsystem = new IntakeSubsystem();
  private final ShooterSubsystem shooterSubsystem = new ShooterSubsystem();
  public final static ArmSubsystem armSubsystem = new ArmSubsystem();
 public static XboxController driver = new XboxController(PortMap.JOYSTICK.DRIVER_JOYSTICK);
  private final SwerveSubsystem swerveSubsystem = new SwerveSubsystem();

  private final Joystick driverController = new Joystick(OIConstants.kDriverControllerPort);

  // Replace with CommandPS4Controller or CommandJoystick if needed
  //private final CommandXboxController driverController =
      //new CommandXboxController(OperatorConstants.kDriverControllerPort);

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    swerveSubsystem.setDefaultCommand(new SwerveJoystickCmd(
      swerveSubsystem,
      () -> -driverController.getRawAxis(OIConstants.kDriverYAxis),
      () -> driverController.getRawAxis(OIConstants.kDriverXAxis),
      () -> driverController.getRawAxis(OIConstants.kDriverRotAxis),
      () -> !driverController.getRawButton(OIConstants.kDriverFieldOrientedButtonIdx)));
    // Configure the trigger bindings
    configureBindings();
  }

private void configureBindings() {
  

   new JoystickButton(driver, Button.kA.value)
      .onTrue(
        Commands.runOnce(
          () -> {
            armSubsystem.setGoal(Constants.ARM.POSITION.INTAKE);
            armSubsystem.enable();
          }
        )
      );
      new JoystickButton(driver, Button.kB.value)
      .onTrue(
        Commands.runOnce(
          () -> {
            armSubsystem.setGoal(Constants.ARM.POSITION.HORIZONTAL);
            armSubsystem.enable();
          }
        )
      );
      new JoystickButton(driver, Button.kY.value)
      .onTrue(
        Commands.runOnce(
          () -> {
            armSubsystem.setGoal(Constants.ARM.POSITION.SECONDLEVEL);
            armSubsystem.enable();
          }
        )
      );
      new JoystickButton(driver, Button.kX.value)
      .onTrue(
        Commands.runOnce(
          () -> {
            armSubsystem.setGoal(1.4);
            armSubsystem.enable();
          }
        )
      );

  /**
   * Use this method to define your trigger->command mappings. Triggers can be created via the
   * {@link Trigger#Trigger(java.util.function.BooleanSupplier)} constructor with an arbitrary
   * predicate, or via the named factories in {@link
   * edu.wpi.first.wpilibj2.command.button.CommandGenericHID}'s subclasses for {@link
   * CommandXboxController Xbox}/{@link edu.wpi.first.wpilibj2.command.button.CommandPS4Controller
   * PS4} controllers or {@link edu.wpi.first.wpilibj2.command.button.CommandJoystick Flight
   * joysticks}.
   */
  
      // driverController.getRawButton(1).whileTrue(new IntakeNote(intakeSubsystem));
      // driverController.getRawButton(0)
      // driverController.b().whileTrue(new PassAndShootNote(shooterSubsystem, intakeSubsystem));
      // driverController.povUp().whileTrue(new SetZeroHeading(swerveSubsystem));

    new JoystickButton(driverController, 0).whileTrue(new SetZeroHeading(swerveSubsystem));
    //new JoystickButton(driverJoytick, 2).whenPressed(() -> swerveSubsystem.zeroHeading());

    // Schedule `ExampleCommand` when `exampleCondition` changes to `true`
    // new Trigger(exampleSubsystem::exampleCondition)
    //     .onTrue(new ExampleCommand(m_exampleSubsystem));


    // Schedule `exampleMethodCommand` when the Xbox controller's B button is pressed,
    // cancelling on release.
    // m_driverController.b().whileTrue(exampleSubsystem.exampleMethodCommand());
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // An example command will be run in autonomous
    return Autos.exampleAuto(exampleSubsystem);
  }
}
