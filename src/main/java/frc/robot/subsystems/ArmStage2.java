// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.Preferences;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.EncoderConstants;
import frc.robot.Constants.ManipulatorConstants;
import frc.robot.Constants.RobotArmPos;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.ctre.phoenix.motorcontrol.TalonFXFeedbackDevice;
import com.ctre.phoenix.motorcontrol.StatusFrameEnhanced;
import com.ctre.phoenix.motorcontrol.TalonFXControlMode;

public class ArmStage2 extends SubsystemBase {
   /* Hardware */
   private final WPI_TalonFX m_ArmStageMotor2 = new WPI_TalonFX(ManipulatorConstants.kArmMotor2, "rio");
   /** How much smoothing [0,8] to use during MotionMagic */
   int _smoothing = 0;
   //backup key values not returned from perference table on shuffleboard
	 final double LowScore = RobotArmPos.extendStage2;
	 final double HighScore = RobotArmPos.extendStage2;
	 final double MidScore = RobotArmPos.extendStage2;
	 final double Default = 0.0;
	 // Used to get numbers from the smart dashboard perference values
	 final String Stage2High = "Score_HL";
	 final String Stage2Mid = "Score_ML";
	 final String Stage2Low = "Score_LL";
	 final String Stage2Default = "Default";
	 //local setpoint for moving to position by magic motion
	 private double setPoint;
	 private double backUp;

  /** Creates a new ArmStage1. */
  public ArmStage2() {
    /* Factory default hardware to prevent unexpected behavior */
		m_ArmStageMotor2.configFactoryDefault();

		/* Configure Sensor Source for Pirmary PID */
		m_ArmStageMotor2.configSelectedFeedbackSensor(TalonFXFeedbackDevice.IntegratedSensor, EncoderConstants.kPIDLoopIdx,
      EncoderConstants.kTimeoutMs);

		/* set deadband to super small 0.001 (0.1 %).
			The default deadband is 0.04 (4 %) */
      m_ArmStageMotor2.configNeutralDeadband(0.001, EncoderConstants.kTimeoutMs);

		/**
		 * Configure Talon FX Output and Sensor direction accordingly Invert Motor to
		 * have green LEDs when driving Talon Forward / Requesting Postiive Output Phase
		 * sensor to have positive increment when driving Talon Forward (Green LED)
		 */
		  m_ArmStageMotor2.setSensorPhase(true);
		  m_ArmStageMotor2.setInverted(true);
		/*
		 * Talon FX does not need sensor phase set for its integrated sensor
		 * This is because it will always be correct if the selected feedback device is integrated sensor (default value)
		 * and the user calls getSelectedSensor* to get the sensor's position/velocity.
		 * 
		 * https://phoenix-documentation.readthedocs.io/en/latest/ch14_MCSensor.html#sensor-phase
		 */
        // _talon.setSensorPhase(true);

		/* Set relevant frame periods to be at least as fast as periodic rate */
		m_ArmStageMotor2.setStatusFramePeriod(StatusFrameEnhanced.Status_13_Base_PIDF0, 10, EncoderConstants.kTimeoutMs);
      	m_ArmStageMotor2.setStatusFramePeriod(StatusFrameEnhanced.Status_10_MotionMagic, 10, EncoderConstants.kTimeoutMs);

		/* Set the peak and nominal outputs */
		m_ArmStageMotor2.configNominalOutputForward(0, EncoderConstants.kTimeoutMs);
	    m_ArmStageMotor2.configNominalOutputReverse(0, EncoderConstants.kTimeoutMs);
	    m_ArmStageMotor2.configPeakOutputForward(1, EncoderConstants.kTimeoutMs);
      	m_ArmStageMotor2.configPeakOutputReverse(-1, EncoderConstants.kTimeoutMs);

		/* Set Motion Magic gains in slot0 - see documentation */
		  m_ArmStageMotor2.selectProfileSlot(EncoderConstants.kSlotIdx, EncoderConstants.kPIDLoopIdx);
      m_ArmStageMotor2.config_kF(EncoderConstants.kSlotIdx, EncoderConstants.kGains.kF, EncoderConstants.kTimeoutMs);
      m_ArmStageMotor2.config_kP(EncoderConstants.kSlotIdx, EncoderConstants.kGains.kP, EncoderConstants.kTimeoutMs);
      m_ArmStageMotor2.config_kI(EncoderConstants.kSlotIdx, EncoderConstants.kGains.kI, EncoderConstants.kTimeoutMs);
      m_ArmStageMotor2.config_kD(EncoderConstants.kSlotIdx, EncoderConstants.kGains.kD, EncoderConstants.kTimeoutMs);

		/* Set acceleration and vcruise velocity - see documentation */
		  m_ArmStageMotor2.configMotionCruiseVelocity(10000, EncoderConstants.kTimeoutMs);
		  m_ArmStageMotor2.configMotionAcceleration(2000, EncoderConstants.kTimeoutMs);

		/* Zero the sensor once on robot boot up */
		 m_ArmStageMotor2.setSelectedSensorPosition(0, EncoderConstants.kPIDLoopIdx, EncoderConstants.kTimeoutMs);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
	SmartDashboard.putNumber("ArmStage2 Encoder", m_ArmStageMotor2.getSelectedSensorPosition(EncoderConstants.kPIDLoopIdx));
  }

  public void SetArmStage2ToPosition(String Key) {
    //set up the grab from values at Smart Dashboard perference table
	switch (Key) {
		case Stage2Low:;
			//Elvator Low
		  	backUp = LowScore;
			Key = Stage2Low;
			break;
		case Stage2Mid:;
		    //Elavator Mid
		    backUp = MidScore;
			Key = Stage2Mid;
			break;
		case Stage2High:;
		    //Elevator Mid
		    backUp = HighScore;
			Key = Stage2High;
			break;
		case Stage2Default:;
			//Elevator default
			backUp = Default;
			Key = Stage2Default;
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
	m_ArmStageMotor2.set(TalonFXControlMode.MotionMagic, targetPos);
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
