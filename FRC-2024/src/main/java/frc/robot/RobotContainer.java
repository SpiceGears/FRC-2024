// Copyright 2021-2024 FRC 6328
// http://github.com/Mechanical-Advantage
//
// This program is free software; you can redistribute it and/or
// modify it under the terms of the GNU General Public License
// version 3 as published by the Free Software Foundation or
// available in the root directory of this project.
//
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
// GNU General Public License for more details.

package frc.robot;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.commands.PathPlannerAuto;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.simulation.ElevatorSim;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.commands.arm.ArmPwmCommand;
import frc.robot.commands.drive.DriveCommands;
import frc.robot.commands.drive.FeedForwardCharacterization;
import frc.robot.commands.elevator.SetElevatorManual;
import frc.robot.commands.intake.IntakeNote;
import frc.robot.commands.intake.PassNoteToShooter;
import frc.robot.commands.shooter.StopShooterPID;
import frc.robot.subsystems.ShuffleBoard;
import frc.robot.subsystems.arm.ArmSubsystem;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.drive.GyroIO;
import frc.robot.subsystems.drive.GyroIONavx;
import frc.robot.subsystems.drive.ModuleIO;
import frc.robot.subsystems.drive.ModuleIOSim;
import frc.robot.subsystems.drive.ModuleIOSparkMax;
import frc.robot.subsystems.elevator.ElevatorSubsystem;
import frc.robot.subsystems.intake.IntakeSubsystem;
import frc.robot.subsystems.limelight.LimelightSubsystem;
import frc.robot.subsystems.shooter.ShooterSubsystem;
import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // Subsystems
  private final Drive drive;
  private final IntakeSubsystem intakeSubsystem;
  private final ShooterSubsystem shooterSubsystem;
  private final ElevatorSubsystem elevatorSubsystem;
  private final LimelightSubsystem limelightSubsystem;
  private final ArmSubsystem armSubsystem;
  private final ShuffleBoard
      shuffleBoard; // TODO test if it works in shuffleboard (shuffleboardsubsystem tab)
  // private final Flywheel flywheel;

  // Controller
  private final CommandXboxController controllerDriver = new CommandXboxController(0);

  // Dashboard inputs
  private final LoggedDashboardChooser<Command> autoChooser;
  // private final LoggedDashboardNumber flywheelSpeedInput =
  //     new LoggedDashboardNumber("Flywheel Speed", 1500.0);

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    switch (Constants.currentMode) {
      case REAL:
        // Real robot, instantiate hardware IO implementations
        drive =
            new Drive(
                new GyroIONavx(),
                new ModuleIOSparkMax(0),
                new ModuleIOSparkMax(1),
                new ModuleIOSparkMax(2),
                new ModuleIOSparkMax(3));
        intakeSubsystem = new IntakeSubsystem();
        shooterSubsystem = new ShooterSubsystem();
        elevatorSubsystem = new ElevatorSubsystem();
        limelightSubsystem = new LimelightSubsystem();
        armSubsystem = new ArmSubsystem();
        shuffleBoard = new ShuffleBoard(intakeSubsystem, shooterSubsystem, drive);

        // ! add new subsystems here!
        // ! add new commands here!

        // flywheel = new Flywheel(new FlywheelIOSparkMax());
        // drive = new Drive(
        // new GyroIOPigeon2(),
        // new ModuleIOTalonFX(0),
        // new ModuleIOTalonFX(1),
        // new ModuleIOTalonFX(2),
        // new ModuleIOTalonFX(3));
        // flywheel = new Flywheel(new FlywheelIOTalonFX());
        break;

      case SIM:
        // Sim robot, instantiate physics sim IO implementations
        drive =
            new Drive(
                new GyroIO() {},
                new ModuleIOSim(),
                new ModuleIOSim(),
                new ModuleIOSim(),
                new ModuleIOSim());
        intakeSubsystem = new IntakeSubsystem();
        shooterSubsystem = new ShooterSubsystem();
        elevatorSubsystem = new ElevatorSubsystem();
        limelightSubsystem = new LimelightSubsystem();
        armSubsystem = new ArmSubsystem();
        shuffleBoard = new ShuffleBoard(intakeSubsystem, shooterSubsystem, drive);
        // ! add new subsystems here!
        // ! add new commands here!
        // flywheel = new Flywheel(new FlywheelIOSim());
        break;

      default:
        // Replayed robot, disable IO implementations
        drive =
            new Drive(
                new GyroIO() {},
                new ModuleIO() {},
                new ModuleIO() {},
                new ModuleIO() {},
                new ModuleIO() {});
        intakeSubsystem = new IntakeSubsystem();
        shooterSubsystem = new ShooterSubsystem();
        elevatorSubsystem = new ElevatorSubsystem();
        limelightSubsystem = new LimelightSubsystem();
        armSubsystem = new ArmSubsystem();
        shuffleBoard = new ShuffleBoard(intakeSubsystem, shooterSubsystem, drive);

        // ! add new subsystems here!
        // ! add new commands here!
        // flywheel = new Flywheel(new FlywheelIO() {});
        break;
    }

    // Set up auto routines
    // NamedCommands.registerCommand(
    //     "Run Flywheel",
    //     Commands.startEnd(
    //             () -> flywheel.runVelocity(flywheelSpeedInput.get()), flywheel::stop, flywheel)
    //         .withTimeout(5.0));
    autoChooser = new LoggedDashboardChooser<>("Auto Choices", AutoBuilder.buildAutoChooser());

    // Set up feedforward characterization
    autoChooser.addOption(
        "Drive FF Characterization",
        new FeedForwardCharacterization(
            drive, drive::runCharacterizationVolts, drive::getCharacterizationVelocity));
    autoChooser.addDefaultOption(
        "all-notes - all notes one by one with subwoofer align", new PathPlannerAuto("all-notes"));
    autoChooser.addOption("rotate-test - 2 circles around", new PathPlannerAuto("rotate-test"));
    autoChooser.addOption("rotate over notes", new PathPlannerAuto("rotateovernotes"));
    autoChooser.addOption("fastallnotes", new PathPlannerAuto("fastallnotes"));

    // autoChooser.addOption(
    //     "Flywheel FF Characterization",
    //     new FeedForwardCharacterization(
    //         flywheel, flywheel::runVolts, flywheel::getCharacterizationVelocity));

    // Configure the button bindings
    configureButtonBindings();
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
   * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {
    drive.setDefaultCommand(
        DriveCommands.joystickDrive(
            drive,
            () -> -controllerDriver.getLeftY(),
            () -> -controllerDriver.getLeftX(),
            () -> -controllerDriver.getRightX()));
    controllerDriver
        .x()
        .whileTrue(
            DriveCommands.angleRotate(
                drive,
                () -> -controllerDriver.getLeftY(),
                () -> -controllerDriver.getLeftX(),
                limelightSubsystem,
                limelightSubsystem.getTvInt()));
    controllerDriver
        .b()
        .onTrue(
            Commands.runOnce(
                    () ->
                        drive.setPose(
                            new Pose2d(drive.getPose().getTranslation(), new Rotation2d())),
                    drive)
                .ignoringDisable(true));

    // ! ARM AND SHOOTER CONTROLS FOR TESTS
    controllerDriver.povUp().whileTrue(new SetElevatorManual(elevatorSubsystem, 0.5));
    controllerDriver.povDown().whileTrue(new SetElevatorManual(elevatorSubsystem, -0.5));
    controllerDriver.rightBumper().whileTrue(new ArmPwmCommand(armSubsystem, 0.5));
    controllerDriver.leftBumper().whileTrue(new ArmPwmCommand(armSubsystem, -0.5));
    // controllerDriver.leftBumper().whileTrue(new StartShooterPID(shooterSubsystem, 1000));
    // controllerDriver.rightBumper().whileTrue(new StopShooterPID(shooterSubsystem));

    // controller.rightBumper().whileTrue(new IntakeNote(intakeSubsystem));
    // controller.leftBumper().onTrue(new PassAndShootNote(shooterSubsystem, intakeSubsystem));

    // controller
    //     .a()
    //     .whileTrue(
    //         Commands.startEnd(
    //             () -> flywheel.runVelocity(flywheelSpeedInput.get()), flywheel::stop, flywheel));
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    return autoChooser.get();
  }
}
