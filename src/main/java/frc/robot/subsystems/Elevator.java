// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.Preferences;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.EncoderConstants;
import frc.robot.Constants.ManipulatorConstants;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.ctre.phoenix.motorcontrol.TalonFXControlMode;
import com.ctre.phoenix.motorcontrol.TalonFXFeedbackDevice;
import com.ctre.phoenix.motorcontrol.StatusFrameEnhanced;

public class Elevator extends SubsystemBase {
  	/* Hardware */
 	 private final WPI_TalonFX m_ElevatorMotor = new WPI_TalonFX(ManipulatorConstants.kElevatorMotor, "rio");
  	/** How much smoothing [0,8] to use during MotionMagic */
	int _smoothing = 0;
	 //backup key values not returned from perference table on shuffleboard
	 final double LowScore = 0.0;      //22.0; *max height
	 final double HighScore = 20.0;
	 final double MidScore = 10.0;
	 final double Default = 0.0;
	 // Used to get numbers from the smart dashboard perference values
	 final String ElevatorHigh = "Score_HL";
	 final String ElevatorMid = "Score_ML";
	 final String ElevatorLow = "Score_LL";
	 final String ElevatorDefault = "Default";
	 //local setpoint for moving to position by magic motion
	 private double setPoint;
	 private double backUp;

  /** Creates a new Elevator. */
  public Elevator() {
    /* Factory default hardware to prevent unexpected behavior */
		m_ElevatorMotor.configFactoryDefault();

		/* Configure Sensor Source for Pirmary PID */
		m_ElevatorMotor.configSelectedFeedbackSensor(TalonFXFeedbackDevice.IntegratedSensor, EncoderConstants.kPIDLoopIdx,
      EncoderConstants.kTimeoutMs);

		/* set deadband to super small 0.001 (0.1 %).
			The default deadband is 0.04 (4 %) */
        m_ElevatorMotor.configNeutralDeadband(0.001, EncoderConstants.kTimeoutMs);

		/**
		 * Configure Talon FX Output and Sensor direction accordingly Invert Motor to
		 * have green LEDs when driving Talon Forward / Requesting Positive Output Phase
		 * sensor to have positive increment when driving Talon Forward (Green LED)
		 */
		m_ElevatorMotor.setSensorPhase(true);
		m_ElevatorMotor.setInverted(true);
		/*
		 * Talon FX does not need sensor phase set for its integrated sensor
		 * This is because it will always be correct if the selected feedback device is integrated sensor (default value)
		 * and the user calls getSelectedSensor* to get the sensor's position/velocity.
		 *

		/* Set relevant frame periods to be at least as fast as periodic rate */
		m_ElevatorMotor.setStatusFramePeriod(StatusFrameEnhanced.Status_13_Base_PIDF0, 10, EncoderConstants.kTimeoutMs);
      	m_ElevatorMotor.setStatusFramePeriod(StatusFrameEnhanced.Status_10_MotionMagic, 10, EncoderConstants.kTimeoutMs);

		/* Set the peak and nominal outputs */
		m_ElevatorMotor.configNominalOutputForward(0, EncoderConstants.kTimeoutMs);
		m_ElevatorMotor.configNominalOutputReverse(0, EncoderConstants.kTimeoutMs);
		m_ElevatorMotor.configPeakOutputForward(1, EncoderConstants.kTimeoutMs);
      	m_ElevatorMotor.configPeakOutputReverse(-1, EncoderConstants.kTimeoutMs);

		/* Set Motion Magic gains in slot0 - see documentation */
		m_ElevatorMotor.selectProfileSlot(EncoderConstants.kSlotIdx, EncoderConstants.kPIDLoopIdx);
		m_ElevatorMotor.config_kF(EncoderConstants.kSlotIdx, EncoderConstants.kGains.kF, EncoderConstants.kTimeoutMs);
		m_ElevatorMotor.config_kP(EncoderConstants.kSlotIdx, EncoderConstants.kGains.kP, EncoderConstants.kTimeoutMs);
		m_ElevatorMotor.config_kI(EncoderConstants.kSlotIdx, EncoderConstants.kGains.kI, EncoderConstants.kTimeoutMs);
		m_ElevatorMotor.config_kD(EncoderConstants.kSlotIdx, EncoderConstants.kGains.kD, EncoderConstants.kTimeoutMs);

		/* Set acceleration and vcruise velocity - see documentation */  //15000, 6000
		m_ElevatorMotor.configMotionCruiseVelocity(45000, EncoderConstants.kTimeoutMs);
		m_ElevatorMotor.configMotionAcceleration(9000, EncoderConstants.kTimeoutMs);

		/* Zero the sensor once on robot boot up */
			m_ElevatorMotor.setSelectedSensorPosition(0, EncoderConstants.kPIDLoopIdx, EncoderConstants.kTimeoutMs);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
	SmartDashboard.putNumber("Elevator Encoder", m_ElevatorMotor.getSelectedSensorPosition(EncoderConstants.kPIDLoopIdx));
  }

  public void SetElevatorToPosition(String Key) {
    //set up the grab from values at Smart Dashboard perference table
	switch (Key) {
		case ElevatorLow:;
			//Elevator Low
		  	backUp = LowScore;
			Key = ElevatorLow;
			break;
		case ElevatorMid:;
		    //Elevator Mid
		    backUp = MidScore;
			Key = ElevatorMid;
			break;
		case ElevatorHigh:;
		    //Elevator Mid
		    backUp = HighScore;
			Key = ElevatorHigh;
			break;
		case ElevatorDefault:;
			//Elevator default
			backUp = Default;
			Key = ElevatorDefault;
			break;
		}
	   
	  //gets the current value
	  setPoint = getPreferencesDouble(Key, backUp);  
	
	/* Motion Magic */
	  /* 2048 ticks/rev * x Rotations in either direction */
	  double targetPos = setPoint * 4096;
	  
	  this.MoveToPosition(targetPos);
  }

  private void MoveToPosition(double targetPos) {
	m_ElevatorMotor.set(TalonFXControlMode.MotionMagic, targetPos);
  }


	/**
    * Retrieve numbers from the preferences table. If the specified key is in
    * the preferences table, then the preference value is returned. Otherwise,
    * return the backup value, and also start a new entry in the preferences
    * table.
	 * @return 
    */
     private double getPreferencesDouble(String key, double backup) {
		
     if (backup != Preferences.getDouble(key, backup)) {

		 return Preferences.getDouble(key, backup);
     }
     else {
		return backup;
	 }
   }
}
