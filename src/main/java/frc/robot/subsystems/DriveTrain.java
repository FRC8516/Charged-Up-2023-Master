// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.ctre.phoenix.sensors.WPI_Pigeon2;
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
  // IMU pigeon 2
  private WPI_Pigeon2 m_pigeon = new WPI_Pigeon2(DriveConstants.kPigeon,"rio");

  public DriveTrain() {
    
    // Invert the right side motors.
    // You may need to change or remove this to match your robot.
    m_frontRightMotor.setInverted(false);
    m_rearRightMotor.setInverted(false);
    
    //Coast Mode
    m_frontLeftMotor.setNeutralMode(NeutralMode.Coast);
    m_frontRightMotor.setNeutralMode(NeutralMode.Coast);
    m_rearLeftMotor.setNeutralMode(NeutralMode.Coast);
    m_rearRightMotor.setNeutralMode(NeutralMode.Coast);
    // Set Masters and Followers
    m_rearLeftMotor.follow(m_frontLeftMotor);
    m_rearRightMotor.follow(m_frontRightMotor);
    //ensure motors are safety is off
    m_frontLeftMotor.setSafetyEnabled(false);
    m_frontRightMotor.setSafetyEnabled(false);
    //Differential Drive train
    m_robotDrive = new DifferentialDrive(m_frontLeftMotor, m_frontRightMotor);
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
    //DifferentialDrive.curvatureDriveIK(-xSpeed, -xSpeed, true);
  }

  public void SetBrakes () {
    m_frontLeftMotor.setNeutralMode(NeutralMode.Brake);
    m_frontRightMotor.setNeutralMode(NeutralMode.Brake);
    m_rearLeftMotor.setNeutralMode(NeutralMode.Brake);
    m_rearRightMotor.setNeutralMode(NeutralMode.Brake);
  }

  public void SetCoastMode () {
    m_frontLeftMotor.setNeutralMode(NeutralMode.Coast);
    m_frontRightMotor.setNeutralMode(NeutralMode.Coast);
    m_rearLeftMotor.setNeutralMode(NeutralMode.Coast);
    m_rearRightMotor.setNeutralMode(NeutralMode.Coast);
  }

  public void autoDrive(double speed) {
    m_frontLeftMotor.set(ControlMode.PercentOutput, -speed);
    m_frontRightMotor.set(ControlMode.PercentOutput, speed);
  }

  public void Rotate(double angle) {
    if (angle > 0) {
      m_frontLeftMotor.set(ControlMode.PercentOutput, -angle);
      m_frontRightMotor.set(ControlMode.PercentOutput, angle);
    }
    if (angle < 0) {
      m_frontLeftMotor.set(ControlMode.PercentOutput, angle);
      m_frontRightMotor.set(ControlMode.PercentOutput, -angle);
    }
  }

  public void stopMotion() {
    m_frontLeftMotor.set(ControlMode.PercentOutput, 0);
    m_frontRightMotor.set(ControlMode.PercentOutput, 0);
  }
 
}