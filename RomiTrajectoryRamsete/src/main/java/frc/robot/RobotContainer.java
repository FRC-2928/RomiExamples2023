// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.List;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.RamseteController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import frc.robot.Constants.ControlConstants;
import frc.robot.Constants.DrivetrainConstants;
import frc.robot.commands.ArcadeDrive;
import frc.robot.commands.AutonomousDistance;
import frc.robot.commands.RunRamseteTrajectory;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.OnBoardIO;
import frc.robot.subsystems.OnBoardIO.ChannelMode;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import edu.wpi.first.wpilibj2.command.RamseteCommand;
import edu.wpi.first.wpilibj2.command.button.Trigger;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...
  private final Drivetrain m_drivetrain = new Drivetrain();
  private final OnBoardIO m_onboardIO = new OnBoardIO(ChannelMode.INPUT, ChannelMode.INPUT);

  // Assumes a gamepad plugged into channnel 0
  // private final Joystick m_joystick = new Joystick(0);
  private final XboxController m_joystick = new XboxController(0);

  // Create SmartDashboard chooser for autonomous routines
  private final SendableChooser<Command> m_chooser = new SendableChooser<>();

  // NOTE: The I/O pin functionality of the 5 exposed I/O pins depends on the hardware "overlay"
  // that is specified when launching the wpilib-ws server on the Romi raspberry pi.
  // By default, the following are available (listed in order from inside of the board to outside):
  // - DIO 8 (mapped to Arduino pin 11, closest to the inside of the board)
  // - Analog In 0 (mapped to Analog Channel 6 / Arduino Pin 4)
  // - Analog In 1 (mapped to Analog Channel 2 / Arduino Pin 20)
  // - PWM 2 (mapped to Arduino Pin 21)
  // - PWM 3 (mapped to Arduino Pin 22)
  //
  // Your subsystem configuration should take the overlays into account

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    // Configure the button bindings
    configureButtonBindings();
    setupShuffleboard();
  }

  /**
   * Setup Shuffleboard
   *
   */
  private void setupShuffleboard() {

    // Create a tab for the Drivetrain
    // ShuffleboardTab driveTab = Shuffleboard.getTab("Drivetrain");
    
  }

  /**
   * Drives a straight line 2 meters so as you can calibrate your Romi
   * You should make sure that the robot ends up right on the 2 meter mark.
   *
   */
  public Trajectory calibrateTrajectory() {
    
    // Note that all coordinates are in meters, and follow NWU conventions.
    Trajectory trajectory = TrajectoryGenerator.generateTrajectory(
        // Start at the origin facing the +X direction
        new Pose2d(0.5, 2.5, new Rotation2d(0)),
        List.of(
            new Translation2d(1.0, 2.5) 
        ),
        new Pose2d(2.5, 2.5, new Rotation2d(0)), // left
        DrivetrainConstants.kTrajectoryConfig);

    return trajectory;
  }

  /**
   * Drives a curved path
   */
  public Trajectory curvedTrajectory() {
    // Note that all coordinates are in meters, and follow NWU conventions.
    Trajectory trajectory = TrajectoryGenerator.generateTrajectory(
        // Start at the origin facing the +X direction
        new Pose2d(0, 0, new Rotation2d(0)),
        List.of(
            new Translation2d(0.5, 0.25), // 1 Left
            new Translation2d(1.0, -0.25), // 2 Right
            new Translation2d(1.5, 0.25)  // 3 Left           
        ),
        new Pose2d(-0.0, -0.2, new Rotation2d(Math.PI)),
        DrivetrainConstants.kTrajectoryConfig);

    return trajectory;
  }

  /**
   * Navigates three cone placed 0.5 meters apart.
   * It should end up back at its starting point
   *
   */
  public Trajectory navigateConesTrajectory() {
    double xOffset = 0.5, yOffset = 2.5;
    // Note that all coordinates are in meters, and follow NWU conventions.
    Trajectory trajectory = TrajectoryGenerator.generateTrajectory(
        // Start at the origin facing the +X direction
        new Pose2d(xOffset, yOffset, new Rotation2d(0)),
        List.of(
            new Translation2d(xOffset + 0.5, yOffset + 0.25), // 1 Left
            new Translation2d(xOffset + 1.0, yOffset + -0.25), // 2 Right
            new Translation2d(xOffset + 1.5, yOffset +  0.25),  // 3 Left
            new Translation2d(xOffset + 2.0, yOffset + 0.0),  // 4 Center
            new Translation2d(xOffset + 1.5, yOffset + -0.25), // 5  Right        
            new Translation2d(xOffset + 1.0, yOffset + 0.25),  // 6 Left
            new Translation2d(xOffset + 0.7, yOffset + -0.25)   // 7 Left
            // new Translation2d(0.4, -0.20) 
        ),
        new Pose2d(xOffset + 0.0, yOffset + -0.2, new Rotation2d(Math.PI)),
        DrivetrainConstants.kTrajectoryConfig);

    return trajectory;
  }

  /**
   * Drives a 0.5 meter square trajectory. 
   *
   */
  public Trajectory driveSquareTrajectory() {   
    // Note that all coordinates are in meters, and follow NWU conventions.
    Trajectory trajectory = TrajectoryGenerator.generateTrajectory(
        // Start at the origin facing the +X direction
        new Pose2d(0, 0, new Rotation2d(0)),
        List.of(
            new Translation2d(0.5, 0.0), // forward 
            new Translation2d(0.5, 0.5), // right 
            new Translation2d(0.0, 0.5) // back           
        ),
        new Pose2d(0.0, 0.0, new Rotation2d(0)), // left
        DrivetrainConstants.kTrajectoryConfig);

    return trajectory;
  }

  /**
   * Generate a trajectory following Ramsete command
   * 
   * This is very similar to the WPILib RamseteCommand example. It uses
   * constants defined in the Constants.java file. These constants were 
   * found empirically by using the frc-characterization tool.
   * 
   * @return A SequentialCommand that sets up and executes a trajectory following Ramsete command
   */
  private Command generateRamseteCommand(Trajectory exampleTrajectory) {

    RamseteCommand ramseteCommand = new RamseteCommand(
        exampleTrajectory,
        m_drivetrain::getPose,
        new RamseteController(ControlConstants.kRamseteB, ControlConstants.kRamseteZeta),
        DrivetrainConstants.kFeedForward,
        DrivetrainConstants.kDriveKinematics,
        m_drivetrain::getWheelSpeeds,
        new PIDController(DrivetrainConstants.kPDriveVelLeft, 0, 0),
        new PIDController(DrivetrainConstants.kPDriveVelRight, 0, 0),
        m_drivetrain::tankDriveVolts,
        m_drivetrain);

    m_drivetrain.resetOdometry(exampleTrajectory.getInitialPose());

    // Set up a sequence of commands
    // First, we want to reset the drivetrain odometry
    return new InstantCommand(() -> m_drivetrain.resetOdometry(exampleTrajectory.getInitialPose()), m_drivetrain)
        // next, we run the actual ramsete command
        .andThen(ramseteCommand);

        // Finally, we make sure that the robot stops
        // .andThen(new InstantCommand(() -> m_drivetrain.tankDriveVolts(0, 0), m_drivetrain));
  } 

  /**
   * Drives the the designated dropoff point
   */
  public Trajectory navigateToDropoffTrajectory(Pose2d startPose, 
                                                Pose2d endPose,
                                                int direction) {

    // Note that all coordinates are in meters, and follow NWU conventions.
    Translation2d leftWaypoint1 = new Translation2d(2.5, 0.75);
    Translation2d leftWaypoint2 = new Translation2d(1.5, 0.75);
    Translation2d rightWaypoint1 = new Translation2d(2.5, 3.75);
    Translation2d rightWaypoint2 = new Translation2d(1.5, 3.75);

    Trajectory trajectory = TrajectoryGenerator.generateTrajectory(
        // Start at the origin facing the +X direction
        startPose,
        List.of(
            leftWaypoint1, // 1 Left
            leftWaypoint2 // 2 Right          
        ),
        endPose,
        DrivetrainConstants.kTrajectoryConfig);

    return trajectory;
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
   * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {
    // Default command is arcade drive. This will run unless another command
    // is scheduled over it.
    m_drivetrain.setDefaultCommand(getArcadeDriveCommand());

    // Example of how to use the onboard IO
    Trigger onboardButtonA = new Trigger(m_onboardIO::getButtonAPressed);
    onboardButtonA
        .onTrue(new PrintCommand("Button A Pressed"))
        .onFalse(new PrintCommand("Button A Released"));

    // Setup SmartDashboard options
    m_chooser.addOption("Auto Routine Distance", new AutonomousDistance(m_drivetrain));
    m_chooser.setDefaultOption("Navigate Cones Trajectory", new RunRamseteTrajectory(m_drivetrain, navigateConesTrajectory()));
    m_chooser.addOption("Calibrate Trajectory", new RunRamseteTrajectory(m_drivetrain, calibrateTrajectory()));
    m_chooser.addOption("Drive Square Trajectory", new RunRamseteTrajectory(m_drivetrain, driveSquareTrajectory()));
    m_chooser.addOption("Old Calibrate Trajectory", generateRamseteCommand(calibrateTrajectory()));
    m_chooser.addOption("Drive To Dropoff", new RunRamseteTrajectory(m_drivetrain, navigateToDropoffTrajectory(m_drivetrain.getPose(), // Start
                                                                                                                    new Pose2d(), // End assigned to a button 
                                                                                                                    0))); // Way around the balance platform
    
    SmartDashboard.putData(m_chooser);
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    return m_chooser.getSelected();
  }

  /**
   * Use this to pass the teleop command to the main {@link Robot} class.
   *
   * @return the command to run in teleop
   */
  public Command getArcadeDriveCommand() {
    return new ArcadeDrive(
        m_drivetrain, () -> -m_joystick.getRawAxis(1), () -> m_joystick.getRawAxis(4));
  }
}
