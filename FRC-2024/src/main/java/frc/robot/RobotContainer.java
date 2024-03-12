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
import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.commands.TurboCommand;
import frc.robot.commands.arm.ArmPwmCommand;
import frc.robot.commands.arm.DisableArm;
import frc.robot.commands.arm.SetArm;
import frc.robot.commands.arm.SetArmJoystick;
import frc.robot.commands.arm.SetArmLimelight;
import frc.robot.commands.drive.DriveCommands;
import frc.robot.commands.intake.IntakeNote;
import frc.robot.commands.intake.PassNoteToShooter;
import frc.robot.commands.shooter.SetShooterTrapezoid;
import frc.robot.commands.shooter.StopShooter;
import frc.robot.subsystems.LedSubsystem;
import frc.robot.subsystems.arm.ArmSubsystemNew;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.drive.GyroIO;
import frc.robot.subsystems.drive.GyroIONavx;
import frc.robot.subsystems.drive.ModuleIO;
import frc.robot.subsystems.drive.ModuleIOSim;
import frc.robot.subsystems.drive.ModuleIOSparkMax;
import frc.robot.subsystems.intake.IntakeSubsystem;
import frc.robot.subsystems.limelight.LimelightDriver;
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
  public final Drive drive;
  private final IntakeSubsystem intakeSubsystem;
  private final ShooterSubsystem shooterSubsystem;
  // private final ElevatorSubsystem elevatorSubsystem;
  public final LimelightDriver limelightSubsystem;
  private final ArmSubsystemNew armSubsystemNew;
  private final LedSubsystem ledSubsystem;
  // private final Flywheel flywheel;

  // Controller
  public static final CommandXboxController controllerDriver = new CommandXboxController(0);
  public static final CommandXboxController controllerOperator = new CommandXboxController(1);
  public final Joystick joystick = new Joystick(0);
  public SteeringDevice steeringDevice = SteeringDevice.GAMEPAD;

  private enum SteeringDevice {
    GAMEPAD,
    JOYSTICK
  }

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
        // elevatorSubsystem = new ElevatorSubsystem();
        limelightSubsystem = new LimelightDriver();
        armSubsystemNew = new ArmSubsystemNew();
        ledSubsystem =
            new LedSubsystem(
                intakeSubsystem, shooterSubsystem, armSubsystemNew, limelightSubsystem);

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
        // elevatorSubsystem = new ElevatorSubsystem();
        limelightSubsystem = new LimelightDriver();
        armSubsystemNew = new ArmSubsystemNew();
        ledSubsystem =
            new LedSubsystem(
                intakeSubsystem, shooterSubsystem, armSubsystemNew, limelightSubsystem);
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
        // elevatorSubsystem = new ElevatorSubsystem();
        limelightSubsystem = new LimelightDriver();
        armSubsystemNew = new ArmSubsystemNew();
        ledSubsystem =
            new LedSubsystem(
                intakeSubsystem, shooterSubsystem, armSubsystemNew, limelightSubsystem);

        // ! add new subsystems here!
        // ! add new commands here!
        // flywheel = new Flywheel(new FlywheelIO() {});
        break;
    }

    // Creates UsbCamera and MjpegServer [1] and connects them
    CameraServer.startAutomaticCapture();

    // // Creates the CvSink and connects it to the UsbCamera
    // CvSink cvSink = CameraServer.getVideo();

    // // Creates the CvSource and MjpegServer [2] and connects them
    // CvSource outputStream = CameraServer.putVideo("Blur", 640, 480);

    // ! TODO register all commands for autonomous
    // ! TODO rainbow LED in autonomous
    NamedCommands.registerCommand(
        "SetShooterTrapezoid(0.5s)",
        new SetShooterTrapezoid(shooterSubsystem, Constants.Shooter.DEFAULT_RPM));
    NamedCommands.registerCommand("SetArm(40)", new SetArm(armSubsystemNew, 40));
    NamedCommands.registerCommand("SetArm(28)", new SetArm(armSubsystemNew, 28));
    NamedCommands.registerCommand("StopShooter", new StopShooter(shooterSubsystem));
    NamedCommands.registerCommand("IntakeNote", new IntakeNote(intakeSubsystem));
    NamedCommands.registerCommand("PassNote(1s)", new PassNoteToShooter(intakeSubsystem));
    NamedCommands.registerCommand(
        "SetArmLimelight()", new SetArmLimelight(armSubsystemNew, limelightSubsystem));
    NamedCommands.registerCommand(
        "RotateSwerveToTarget",
        DriveCommands.angleRotate(
            drive, () -> Constants.Swerve.SPEED_LIMELIGHT, () -> 0, () -> 0, limelightSubsystem));
    NamedCommands.registerCommand(
        "SetArm(Intake)", new SetArm(armSubsystemNew, Constants.Arm.INTAKING_SETPOINT));
    NamedCommands.registerCommand(
        "TurboCommand",
        new TurboCommand(
            2,
            shooterSubsystem,
            intakeSubsystem,
            armSubsystemNew,
            limelightSubsystem,
            ledSubsystem,
            drive));
    NamedCommands.registerCommand(
        "TurboCommandSlow",
        new TurboCommand(
            2.5,
            shooterSubsystem,
            intakeSubsystem,
            armSubsystemNew,
            limelightSubsystem,
            ledSubsystem,
            drive));

    autoChooser = new LoggedDashboardChooser<>("Auto Choices", AutoBuilder.buildAutoChooser());

    // Set up feedforward characterization
    autoChooser.addOption("alpha", new PathPlannerAuto("alpha"));
    autoChooser.addOption("beta", new PathPlannerAuto("beta"));

    // autoChooser.addOption(
    //     "Drive FF Characterization",
    //     new FeedForwardCharacterization(
    //         drive, drive::runCharacterizationVolts, drive::getCharacterizationVelocity));
    // autoChooser.addDefaultOption(
    //     "all-notes - all notes one by one with subwoofer align", new
    // PathPlannerAuto("all-notes"));
    // autoChooser.addOption("rotate-test - 2 circles around", new PathPlannerAuto("rotate-test"));
    // autoChooser.addOption("rotate over notes", new PathPlannerAuto("rotateovernotes"));
    // autoChooser.addOption("fastallnotes", new PathPlannerAuto("fastallnotes"));

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

    switch (steeringDevice) {
      case GAMEPAD:
        drive.setDefaultCommand(
            DriveCommands.joystickDrive(
                drive,
                () -> {
                  if (controllerDriver.leftBumper().getAsBoolean()) {
                    return 1;
                  } else {
                    return 0.7;
                  }
                },
                () -> -controllerDriver.getLeftY(),
                () -> -controllerDriver.getLeftX(),
                () -> -controllerDriver.getRightX()));

        controllerDriver
            .x()
            .onTrue(
                Commands.runOnce(
                        () ->
                            drive.setPose(
                                new Pose2d(drive.getPose().getTranslation(), new Rotation2d())),
                        drive)
                    .ignoringDisable(true));

        // ! ARM AND SHOOTER CONTROLS FOR TESTS
        controllerDriver.leftTrigger().whileTrue(new IntakeNote(intakeSubsystem));
        controllerDriver
            .leftTrigger()
            .whileTrue(new SetArm(armSubsystemNew, Constants.Arm.INTAKING_SETPOINT));
        controllerDriver.rightBumper().whileTrue(new PassNoteToShooter(intakeSubsystem));

        controllerDriver
            .rightTrigger()
            .whileTrue(
                new ParallelCommandGroup(
                    // new DisableArm(armSubsystemNew),
                    new SetShooterTrapezoid(shooterSubsystem, Constants.Shooter.DEFAULT_RPM)));

        controllerDriver.povUp().whileTrue(new SetArm(armSubsystemNew, Constants.Arm.MAX_SETPOINT));
        controllerDriver
            .povDown()
            .whileTrue(new SetArm(armSubsystemNew, Constants.Arm.INTAKING_SETPOINT));
        controllerDriver
            .povRight()
            .whileTrue(new SetArm(armSubsystemNew, Constants.Arm.MIDDLE_SETPOINT));
        controllerDriver.povLeft().whileTrue(new DisableArm(armSubsystemNew));

        controllerDriver
            .a()
            .whileTrue(
                new ParallelCommandGroup(
                    new SetArmLimelight(armSubsystemNew, limelightSubsystem),
                    DriveCommands.angleRotate(
                        drive,
                        () -> Constants.Swerve.SPEED_LIMELIGHT,
                        () -> -controllerDriver.getLeftY(),
                        () -> -controllerDriver.getLeftX(),
                        limelightSubsystem)));

        // fully automatic sequence used in autonomous
        controllerDriver
            .b()
            .whileTrue(
                new TurboCommand(
                    1.5,
                    shooterSubsystem,
                    intakeSubsystem,
                    armSubsystemNew,
                    limelightSubsystem,
                    ledSubsystem,
                    drive));

        // controllerDriver.a().whileTrue(new SetArmLimelight(armSubsystemNew, limelightSubsystem));
        // controllerDriver
        // .a()
        // .whileTrue(
        //     DriveCommands.angleRotate(
        //         drive,
        //         () -> Constants.Swerve.SPEED_LIMELIGHT,
        //         () -> -controllerDriver.getLeftY(),
        //         () -> -controllerDriver.getLeftX(),
        //         limelightSubsystem,
        //         limelightSubsystem.getTvInt()));

        controllerOperator
            .a()
            .whileTrue(new SetArmJoystick(armSubsystemNew, () -> -controllerDriver.getLeftY()));

        controllerOperator.b().whileTrue(new DisableArm(armSubsystemNew));

        controllerOperator
            .leftBumper()
            .whileTrue(new ArmPwmCommand(armSubsystemNew, Constants.Arm.MANUAL_SPEED_UP));
        controllerOperator
            .rightBumper()
            .whileTrue(new ArmPwmCommand(armSubsystemNew, Constants.Arm.MANUAL_SPEED_DOWN));

        break;

      case JOYSTICK:
        // TODO jak chcemy joystick
        break;
    }

    //   case JOYSTICK:
    //     drive.setDefaultCommand(
    //         DriveCommands.joystickDrive(
    //             drive,
    //             () -> joystick.getRawAxis(3),
    //             () -> -joystick.getY(),
    //             () -> -joystick.getX(),
    //             () -> -joystick.getRawAxis(2)));
    //     if (joystick.getRawButton(1)) {
    //       DriveCommands.angleRotate(
    //           drive,
    //           () -> joystick.getRawAxis(3),
    //           () -> -joystick.getY(),
    //           () -> -joystick.getX(),
    //           limelightSubsystem,
    //           limelightSubsystem.getTvInt());
    //     }
    //     if (joystick.getRawButton(2)) {
    //       Commands.runOnce(
    //               () ->
    //                   drive.setPose(new Pose2d(drive.getPose().getTranslation(), new
    // Rotation2d())),
    //               drive)
    //           .ignoringDisable(true);
    //     }

    //     // ! ARM AND SHOOTER CONTROLS FOR TESTS
    //     if (joystick.getRawButtonPressed(1)) {
    //       new SetShooterManual(shooterSubsystem).schedule();
    //     }
    //     controllerDriver.rightBumper().whileTrue(new ArmPwmCommand(armSubsystemNew, 0.5));
    //     controllerDriver.leftBumper().whileTrue(new ArmPwmCommand(armSubsystemNew, -0.3));
    //     controllerDriver.a().whileTrue(new IntakeNote(intakeSubsystem));
    //     controllerDriver.y().whileTrue(new SetShooterManual(shooterSubsystem));
    //     // controllerDriver.leftBumper().whileTrue(new StartShooterPID(shooterSubsystem, 1000));
    //     // controllerDriver.rightBumper().whileTrue(new StopShooterPID(shooterSubsystem));

    //     // controller.rightBumper().whileTrue(new IntakeNote(intakeSubsystem));
    //     // controller.leftBumper().onTrue(new PassAndShootNote(shooterSubsystem,
    // intakeSubsystem));
    //     break;
    // }
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
