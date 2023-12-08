// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.math.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.math.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.BuiltInAccelerometer;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import frc.robot.Constants;
import frc.robot.sensors.RomiGyro;
import edu.wpi.first.wpilibj.motorcontrol.Spark;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Drivetrain extends SubsystemBase {

  // The Romi has the left and right motors set to
  // PWM channels 0 and 1 respectively
  private final Spark m_leftMotor = new Spark(0);
  private final Spark m_rightMotor = new Spark(1);

  // The Romi has onboard encoders that are hardcoded
  // to use DIO pins 4/5 and 6/7 for the left and right
  private final Encoder m_leftEncoder = new Encoder(4, 5);
  private final Encoder m_rightEncoder = new Encoder(6, 7);

  // Set up the differential drive controller
  private final DifferentialDrive m_diffDrive = new DifferentialDrive(m_leftMotor, m_rightMotor);

  // Setup differential drive kinematics
  public static final DifferentialDriveKinematics m_driveKinematics =
        new DifferentialDriveKinematics(Constants.kTrackwidthMeters);
        
  // Set up the RomiGyro
  private final RomiGyro m_gyro = new RomiGyro();

  // Set up the BuiltInAccelerometer
  private final BuiltInAccelerometer m_accelerometer = new BuiltInAccelerometer();

  // Odometry class for tracking robot pose
  private final DifferentialDriveOdometry m_odometry;

  // Show a field diagram for tracking odometry
  private final Field2d m_field2d = new Field2d();

  // Used to put telemetry data onto Shuffleboard
  GenericEntry m_headingEntry, m_avgDistanceEntry;
  GenericEntry m_leftWheelPositionEntry, m_rightWheelPositionEntry;
  
  // -----------------------------------------------------------
  // Initialization
  // -----------------------------------------------------------

  /** Creates a new Drivetrain. */
  public Drivetrain() {
    // We need to invert one side of the drivetrain so that positive voltages
    // result in both sides moving forward. Depending on how your robot's
    // gearbox is constructed, you might have to invert the left side instead.
    m_rightMotor.setInverted(true);

    // Use inches as unit for encoder distances
    m_leftEncoder.setDistancePerPulse((Math.PI * Constants.kWheelDiameterMeters) / Constants.kCountsPerRevolution);
    m_rightEncoder.setDistancePerPulse((Math.PI * Constants.kWheelDiameterMeters) / Constants.kCountsPerRevolution);
    resetEncoders();
    setupShuffleboard();

    // Setup Odometry 
    Pose2d initialPose = new Pose2d(0, 1.5, m_gyro.getRotation2d()); 
    m_field2d.setRobotPose(initialPose);
    m_odometry = new DifferentialDriveOdometry(m_gyro.getRotation2d(), 
                   getLeftDistanceMeters(), getRightDistanceMeters(), 
                   initialPose);
    SmartDashboard.putData("field", m_field2d);
  }

  private void setupShuffleboard() {

    // Create a tab for the Drivetrain
    ShuffleboardTab m_driveTab = Shuffleboard.getTab("Drivetrain");
    m_headingEntry = m_driveTab.add("Heading Deg.", getHeading())
        .withWidget(BuiltInWidgets.kGraph)      
        .withSize(3,3)
        .withPosition(0, 0)
        .getEntry();  
    m_leftWheelPositionEntry = m_driveTab.add("Left Wheel Pos.", getLeftDistanceMeters())
        .withWidget(BuiltInWidgets.kGraph)      
        .withSize(3,3)  
        .withPosition(4, 0)
        .getEntry();  
    m_rightWheelPositionEntry = m_driveTab.add("Right Wheel Pos.", getRightDistanceMeters())
        .withWidget(BuiltInWidgets.kGraph)      
        .withSize(3,3)
        .withPosition(7, 0)
        .getEntry(); 
    m_avgDistanceEntry = m_driveTab.add("Average Distance", getAverageDistanceMeters())
        .withWidget(BuiltInWidgets.kGraph)      
        .withSize(3,3)
        .withPosition(10, 0)
        .getEntry();     
  }

  // -----------------------------------------------------------
  // Control Input
  // -----------------------------------------------------------
  public void arcadeDrive(double xaxisSpeed, double zaxisRotate) {
    m_diffDrive.arcadeDrive(xaxisSpeed, zaxisRotate);
  }

  public void resetEncoders() {
    m_leftEncoder.reset();
    m_rightEncoder.reset();
  }

  /** Reset the gyro. */
  public void resetGyro() {
    m_gyro.reset();
  }

  /**
   * Resets the odometry to the specified pose
   * @param pose The pose to which to set the odometry
   */
  public void resetOdometry(Pose2d pose) {
    resetEncoders();
    resetGyro();
    m_odometry.resetPosition(m_gyro.getRotation2d(),
          getLeftDistanceMeters(), getRightDistanceMeters(), 
          pose);
  }

  // -----------------------------------------------------------
  // System State
  // -----------------------------------------------------------
  public int getLeftEncoderCount() {
    return m_leftEncoder.get();
  }

  public int getRightEncoderCount() {
    return m_rightEncoder.get();
  }

  public double getLeftDistanceMeters() {
    return m_leftEncoder.getDistance();
  }

  public double getRightDistanceMeters() {
    return m_rightEncoder.getDistance();
  }

  public double getAverageDistanceMeters() {
    return (getLeftDistanceMeters() + getRightDistanceMeters()) / 2.0;
  }

  /**
   * Returns the current wheel speeds of the robot.
   * @return The current wheel speeds
   */
  public DifferentialDriveWheelSpeeds getWheelSpeeds() {
    // Convert to wheel speeds
    return new DifferentialDriveWheelSpeeds(m_leftEncoder.getDistance(), 
                                            m_rightEncoder.getDistance());
  }
  
  /**
   * The acceleration in the X-axis.
   *
   * @return The acceleration of the Romi along the X-axis in Gs
   */
  public double getAccelX() {
    return m_accelerometer.getX();
  }

  /**
   * The acceleration in the Y-axis.
   *
   * @return The acceleration of the Romi along the Y-axis in Gs
   */
  public double getAccelY() {
    return m_accelerometer.getY();
  }

  /**
   * The acceleration in the Z-axis.
   *
   * @return The acceleration of the Romi along the Z-axis in Gs
   */
  public double getAccelZ() {
    return m_accelerometer.getZ();
  }

  /**
   * Current angle of the Romi around the X-axis.
   *
   * @return The current angle of the Romi in degrees
   */
  public double getGyroAngleX() {
    return m_gyro.getAngleX();
  }

  /**
   * Current angle of the Romi around the Y-axis.
   *
   * @return The current angle of the Romi in degrees
   */
  public double getGyroAngleY() {
    return m_gyro.getAngleY();
  }

  /**
   * Current angle of the Romi around the Z-axis.
   *
   * @return The current angle of the Romi in degrees
   */
  public double getGyroAngleZ() {
    return m_gyro.getAngleZ();
  }

  public double getHeading() {
    return m_gyro.getRotation2d().getDegrees();
  }
  
  // -----------------------------------------------------------
  // Process Logic
  // -----------------------------------------------------------
  @Override
  public void periodic() {
    // Update the odometry in the periodic block
    m_odometry.update(m_gyro.getRotation2d(), 
                      m_leftEncoder.getDistance(), 
                      m_rightEncoder.getDistance());

    // This method will be called once per scheduler run
    publishTelemetry();
  }

  public void publishTelemetry() {
        
    // Display the meters per/second for each wheel and the heading
    DifferentialDriveWheelSpeeds wheelSpeeds = getWheelSpeeds();
    SmartDashboard.putNumber("Left wheel speed", wheelSpeeds.leftMetersPerSecond);
    SmartDashboard.putNumber("Right wheel speed", wheelSpeeds.leftMetersPerSecond);

    // SmartDashboard.putNumber("Left Wheel Speed", m_leftEncoder.getRate());
    // SmartDashboard.putNumber("Right Wheel Speed", m_rightEncoder.getRate());

    SmartDashboard.putNumber("Heading", getHeading());

    // Display the distance travelled for each wheel
    m_leftWheelPositionEntry.setDouble(getLeftDistanceMeters());
    m_rightWheelPositionEntry.setDouble(getRightDistanceMeters()); 
    m_avgDistanceEntry.setDouble(getAverageDistanceMeters());
    m_headingEntry.setDouble(getHeading());

    m_field2d.setRobotPose(m_odometry.getPoseMeters());  
  }

}
