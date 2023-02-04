// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.ctre.phoenix.sensors.WPI_PigeonIMU;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.math.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.EncoderConstants;


public class DriveTrain extends SubsystemBase {
 
  //Drive Motors
  private final WPI_TalonFX m_frontLeftMotor = new WPI_TalonFX(DriveConstants.kFrontLeftChannel);
  private final WPI_TalonFX m_rearLeftMotor = new WPI_TalonFX(DriveConstants.kRearLeftChannel);
  private final WPI_TalonFX m_frontRightMotor = new WPI_TalonFX(DriveConstants.kFrontRightChannel);
  private final WPI_TalonFX m_rearRightMotor = new WPI_TalonFX(DriveConstants.kRearRightChannel);

  // Drive Class
  private DifferentialDrive m_robotDrive;
  double joyThreshold = 0.05; // Default threshold value from XboxController
  //Declare drive train information
  public static final double kMaxSpeed = 3.0; // meters per second
  public static final double kMaxAngularSpeed = 2 * Math.PI; // one rotation per second

  private static final double kTrackWidth = 0.381 * 2; // meters
  private static final double kWheelRadius = 0.0508; // meters
  private static final int kEncoderResolution = 4096;
  private final PIDController m_leftPIDController = new PIDController(1, 0, 0);
  private final PIDController m_rightPIDController = new PIDController(1, 0, 0);
  private final Encoder m_leftEncoder = new Encoder(0, 1);
  private final Encoder m_rightEncoder = new Encoder(2, 3);

  private final DifferentialDriveKinematics m_kinematics =
      new DifferentialDriveKinematics(kTrackWidth);

  private final DifferentialDriveOdometry m_odometry;

  // Gains are for example purposes only - must be determined for your own robot!
  private final SimpleMotorFeedforward m_feedforward = new SimpleMotorFeedforward(1, 3);
  private WPI_PigeonIMU m_pigeon;

  public DriveTrain() {
    
    // Invert the right side motors.
    // You may need to change or remove this to match your robot.
    m_frontRightMotor.setInverted(true);
    m_rearRightMotor.setInverted(true);

    m_robotDrive = new DifferentialDrive(m_frontLeftMotor, m_frontRightMotor);
    //Set Masters and Followers
    m_rearLeftMotor.follow(m_frontLeftMotor);
    m_rearRightMotor.follow(m_frontRightMotor);
    // Configures the Drive Train Falcon's to default configuration
    m_frontLeftMotor.configFactoryDefault();
    m_rearLeftMotor.configFactoryDefault();
    m_frontRightMotor.configFactoryDefault();
    m_rearLeftMotor.configFactoryDefault();
    // Drive Train Encoder Setup
    m_frontLeftMotor.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor, EncoderConstants.kPIDLoopIdx, EncoderConstants.kTimeoutMs);
    m_frontRightMotor.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor, EncoderConstants.kPIDLoopIdx, EncoderConstants.kTimeoutMs);
    //reset IMU
    m_pigeon.reset();
    //reset encoders
    

    //get odmetry information
    m_odometry =
        new DifferentialDriveOdometry(
            m_pigeon.getRotation2d(), m_frontLeftMotor.getSelectedSensorPosition(Constants.EncoderConstants.kPIDLoopIdx), 
            m_frontRightMotor.getSelectedSensorPosition(Constants.EncoderConstants.kPIDLoopIdx));

  
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
     // Pushing Drive Encoder Data to the SmartDashboard
   SmartDashboard.putNumber("LeftSensorPosition", m_frontLeftMotor.getSelectedSensorPosition(Constants.EncoderConstants.kPIDLoopIdx));
   SmartDashboard.putNumber("RightSensorPosition", m_frontRightMotor.getSelectedSensorPosition(Constants.EncoderConstants.kPIDLoopIdx));
  }
  

  // Drive Type
  public void drive(double ySpeed, double xSpeed){

    // Drive with arcade drive.
    // That means that the Y axis drives forward
    // and backward, and the X turns left and right.
    m_robotDrive.arcadeDrive(-xSpeed, -ySpeed,true);

/* If needed we can add something like this!
    if(Math.abs(xSpeed) > joyThreshold  || Math.abs(zRotation) > joyThreshold ) {
      //m_Drive.arcadeDrive(xSpeed, zRotation);

      //m_Drive.arcadeDrive(xSpeed*1.0, zRotation*-0.6);
    }
    else {
      //m_Drive.arcadeDrive(0.0, 0.0);
    } */
  }

  public void autoDrive(){
    
  }
 
}