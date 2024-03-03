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
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.commands.arm.ArmPwmCommand;
import frc.robot.commands.drive.DriveCommands;
import frc.robot.commands.drive.FeedForwardCharacterization;
import frc.robot.commands.intake.IntakeNote;
import frc.robot.commands.intake.PassNoteToShooter;
import frc.robot.commands.shooter.StartShooterManual;
import frc.robot.commands.shooter.StartShooterManualForSeconds;
import frc.robot.commands.shooter.StartShooterPID;
import frc.robot.commands.shooter.StopShooterManual;
import frc.robot.commands.shooter.StopShooterPID;
import frc.robot.subsystems.ShuffleBoard;
import frc.robot.subsystems.arm.ArmSubsystem;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.drive.GyroIO;
import frc.robot.subsystems.drive.GyroIONavx;
import frc.robot.subsystems.drive.ModuleIO;
import frc.robot.subsystems.drive.ModuleIOSim;
import frc.robot.subsystems.drive.ModuleIOSparkMax;
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
        limelightSubsystem = new LimelightSubsystem();
        armSubsystem = new ArmSubsystem();
        shuffleBoard = new ShuffleBoard(intakeSubsystem, shooterSubsystem, drive);
        NamedCommands.registerCommand("IntakeNote", new IntakeNote(intakeSubsystem));
        NamedCommands.registerCommand("PassNoteToShooter", new PassNoteToShooter(intakeSubsystem));
        NamedCommands.registerCommand(
            "RollShooterForSeconds(5)", new StartShooterManualForSeconds(shooterSubsystem, 5));
        NamedCommands.registerCommand(
            "StartShooterManual", new StartShooterManual(shooterSubsystem));
        NamedCommands.registerCommand("StopShooterManual", new StopShooterManual(shooterSubsystem));
        NamedCommands.registerCommand(
            "StartShooterPID(1000rpm)", new StartShooterPID(shooterSubsystem, 1000));
        NamedCommands.registerCommand("StopShooterPID", new StopShooterPID(shooterSubsystem));

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
        limelightSubsystem = new LimelightSubsystem();
        armSubsystem = new ArmSubsystem();
        shuffleBoard = new ShuffleBoard(intakeSubsystem, shooterSubsystem, drive);
        NamedCommands.registerCommand("IntakeNote", new IntakeNote(intakeSubsystem));
        NamedCommands.registerCommand("PassNoteToShooter", new PassNoteToShooter(intakeSubsystem));
        NamedCommands.registerCommand(
            "RollShooterForSeconds(5)", new StartShooterManualForSeconds(shooterSubsystem, 5));
        NamedCommands.registerCommand(
            "StartShooterManual", new StartShooterManual(shooterSubsystem));
        NamedCommands.registerCommand("StopShooterManual", new StopShooterManual(shooterSubsystem));
        NamedCommands.registerCommand(
            "StartShooterPID(1000rpm)", new StartShooterPID(shooterSubsystem, 1000));
        NamedCommands.registerCommand("StopShooterPID", new StopShooterPID(shooterSubsystem));
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
        limelightSubsystem = new LimelightSubsystem();
        armSubsystem = new ArmSubsystem();
        shuffleBoard = new ShuffleBoard(intakeSubsystem, shooterSubsystem, drive);
        NamedCommands.registerCommand("IntakeNote", new IntakeNote(intakeSubsystem));
        NamedCommands.registerCommand("PassNoteToShooter", new PassNoteToShooter(intakeSubsystem));
        NamedCommands.registerCommand(
            "RollShooterForSeconds(5)", new StartShooterManualForSeconds(shooterSubsystem, 5));
        NamedCommands.registerCommand(
            "StartShooterManual", new StartShooterManual(shooterSubsystem));
        NamedCommands.registerCommand("StopShooterManual", new StopShooterManual(shooterSubsystem));
        NamedCommands.registerCommand(
            "StartShooterPID(1000rpm)", new StartShooterPID(shooterSubsystem, 1000));
        NamedCommands.registerCommand("StopShooterPID", new StopShooterPID(shooterSubsystem));

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
    
    // Driver: While Right Stick button is pressed, drive while pointing to alliance speaker
        // AND adjusting Arm angle AND running Shooter
        m_driverCtrl.rightStick().whileTrue(Commands.parallel(
            m_drivetrain.applyRequest(
                () -> m_head.withVelocityX(-m_driverCtrl.getLeftY() * m_MaxSpeed)
                        .withVelocityY(-m_driverCtrl.getLeftX() * m_MaxSpeed)
                        .withTargetDirection(m_drivetrain.RotToSpeaker())
                        .withDeadband(m_MaxSpeed * 0.1)
                        .withRotationalDeadband(m_AngularRate * 0.1)
            ),
            new LookUpShot(m_armSubsystem, m_shooterSubsystem, () -> m_drivetrain.calcDistToSpeaker())
        ));

        m_driverCtrl.start().onTrue(new LookUpShot(m_armSubsystem, m_shooterSubsystem, () -> m_drivetrain.calcDistToSpeaker()));

         // Driver: DPad Left: put swerve modules in Brake mode (modules make an 'X') (while pressed)
        m_driverCtrl.povLeft().whileTrue(m_drivetrain.applyRequest(() -> m_brake));

         // Driver: DPad Up: Reset the field-centric heading (when pressed)
        m_driverCtrl.povUp().onTrue(m_drivetrain.runOnce(() -> m_drivetrain.seedFieldRelative()));

        // Driver: While Left Bumper is held, reduce speed by 50%
         m_driverCtrl.leftBumper().onTrue(runOnce(() -> m_MaxSpeed = TunerConstants.kSpeedAt12VoltsMps * m_HalfSpeed)
                .andThen(() -> m_AngularRate = m_HalfAngularRate));
        m_driverCtrl.leftBumper().onFalse(runOnce(() -> m_MaxSpeed = TunerConstants.kSpeedAt12VoltsMps * m_lastSpeed)
                .andThen(() -> m_AngularRate = m_MaxAngularRate));
        
        // Driver: While Right Bumper is held, reduce speed by 25%
         m_driverCtrl.leftBumper().onTrue(runOnce(() -> m_MaxSpeed = TunerConstants.kSpeedAt12VoltsMps * m_QuarterSpeed)
                .andThen(() -> m_AngularRate = m_QuarterAngularRate));
        m_driverCtrl.leftBumper().onFalse(runOnce(() -> m_MaxSpeed = TunerConstants.kSpeedAt12VoltsMps * m_lastSpeed)
                .andThen(() -> m_AngularRate = m_MaxAngularRate));
        
        // Driver: When LeftTrigger is pressed, lower the Arm and then run the Intake and Stage until a Note is found
        m_driverCtrl.leftTrigger(0.4).onTrue(m_armSubsystem.prepareForIntakeCommand()
            .andThen(new intakeNote(m_intakeSubsystem, m_stageSubsystem)));

        // Driver: When RightTrigger is pressed, release Note to shooter, then lower Arm
        m_driverCtrl.rightTrigger(0.4).onTrue(m_stageSubsystem.feedNote2ShooterCommand()
            .andThen(m_armSubsystem.prepareForIntakeCommand()));

        // Driver: While start button held, adjust Arm elevation based on goal
        //m_driverCtrl.start().onTrue(Commands.parallel(m_shooterSubsystem.runShooterCommand(),m_armSubsystem.moveToDegreeCommand()));

        /*
         * OPERATOR Controls
         */
        // Operator: When A button is pressed, stop Shooter
        //m_operatorCtrl.a().onTrue(m_shooterSubsystem.runShooterCommand());
        m_operatorCtrl.a().onFalse(m_shooterSubsystem.stopShooterCommand());

        

         // Operator: X Button: Arm to Stowed Position (when pressed)
         m_operatorCtrl.x().onTrue(new prepareToShoot(RobotConstants.STOWED, ()->m_stageSubsystem.isNoteInStage(),
                m_armSubsystem, m_shooterSubsystem));

        // Operator: Use Left Bumper and Left Stick Y-Axis to manually control Arm
        m_armSubsystem.setDefaultCommand(
            new ArmDefault(m_armSubsystem, m_operatorCtrl.leftBumper(), () -> (-1.0)*m_operatorCtrl.getLeftY())
        );

         // Operator: DPad Left: Arm to Podium position (when pressed)
         m_operatorCtrl.povLeft().onTrue(new prepareToShoot(RobotConstants.PODIUM, ()->m_stageSubsystem.isNoteInStage(),
                m_armSubsystem, m_shooterSubsystem));

         // Operator: DPad Up: Shooter/Arm to AMP Position & Speed (when pressed)
        m_operatorCtrl.povUp().onTrue(new prepareToShoot(RobotConstants.AMP, ()->m_stageSubsystem.isNoteInStage(),
                m_armSubsystem, m_shooterSubsystem));

         // Operator: DPad Right: Arm to Wing Position (when pressed)
         m_operatorCtrl.povRight().onTrue(new prepareToShoot(RobotConstants.WING, ()->m_stageSubsystem.isNoteInStage(),
                m_armSubsystem, m_shooterSubsystem));

         // Operator: DPad Down: Arm to Subwoofer Position (when pressed)
         m_operatorCtrl.povDown().onTrue(new prepareToShoot(RobotConstants.SUBWOOFER, ()->m_stageSubsystem.isNoteInStage(),
                m_armSubsystem, m_shooterSubsystem));

        // Operator: Use Left and Right Triggers to run Intake at variable speed (left = in, right = out)
        m_intakeSubsystem.setDefaultCommand(new IntakeDefault(m_intakeSubsystem, m_stageSubsystem,
                                            ()-> m_operatorCtrl.getLeftTriggerAxis(),
                                            () -> m_operatorCtrl.getRightTriggerAxis()));

        //
        m_operatorCtrl.rightTrigger().whileTrue(m_stageSubsystem.ejectBackManualCommand());
        m_operatorCtrl.leftTrigger().whileTrue(m_stageSubsystem.ejectFrontManualCommand());
        m_operatorCtrl.b().whileTrue(m_stageSubsystem.ejectFrontManualCommand());
        m_operatorCtrl.y().onTrue(m_armSubsystem.prepareForIntakeCommand());

        /*
         * Put Commands on Shuffleboard
         */
        if (RobotConstants.kIsShooterTuningMode) {
            SmartDashboard.putData("Update Shooter Gains", m_shooterSubsystem.updateShooterGainsCommand());
            SmartDashboard.putData("Run Shooter", m_shooterSubsystem.runShooterCommand());
            SmartDashboard.putData("Stop Shooter", m_shooterSubsystem.stopShooterCommand());
            SmartDashboard.putData("Arm to Angle", m_armSubsystem.moveToDegreeCommand());
        }
        SmartDashboard.putData("Move Arm To Setpoint", m_armSubsystem.tuneArmSetPointCommand());
       
    }

    private void configureSysIDProfiling() {

        /*
         * These bindings will only be used when characterizing the Drivetrain. They can
         * eventually be commented out.
         */
/*
        m_driverCtrl.x().and(m_driverCtrl.pov(0)).whileTrue(m_drivetrain.runDriveQuasiTest(Direction.kForward));
        m_driverCtrl.x().and(m_driverCtrl.pov(180)).whileTrue(m_drivetrain.runDriveQuasiTest(Direction.kReverse));

        m_driverCtrl.y().and(m_driverCtrl.pov(0)).whileTrue(m_drivetrain.runDriveDynamTest(Direction.kForward));
        m_driverCtrl.y().and(m_driverCtrl.pov(180)).whileTrue(m_drivetrain.runDriveDynamTest(Direction.kReverse));

        m_driverCtrl.a().and(m_driverCtrl.pov(0)).whileTrue(m_drivetrain.runSteerQuasiTest(Direction.kForward));
        m_driverCtrl.a().and(m_driverCtrl.pov(180)).whileTrue(m_drivetrain.runSteerQuasiTest(Direction.kReverse));

        m_driverCtrl.b().and(m_driverCtrl.pov(0)).whileTrue(m_drivetrain.runSteerDynamTest(Direction.kForward));
        m_driverCtrl.b().and(m_driverCtrl.pov(180)).whileTrue(m_drivetrain.runSteerDynamTest(Direction.kReverse));

        // Drivetrain needs to be placed against a sturdy wall and test stopped
        // immediately upon wheel slip
        m_driverCtrl.back().and(m_driverCtrl.pov(0)).whileTrue(m_drivetrain.runDriveSlipTest());
    */
    }
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