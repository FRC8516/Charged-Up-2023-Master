// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
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
  
 // private WPI_PigeonIMU m_pigeon;

  public DriveTrain() {
    
    // Invert the right side motors.
    // You may need to change or remove this to match your robot.
    m_frontRightMotor.setInverted(false);
    m_rearRightMotor.setInverted(false);

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
    //m_pigeon.reset();  
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
  }

  public void autoDrive(){
    
  }
 
}