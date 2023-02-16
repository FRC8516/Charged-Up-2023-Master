// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

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
		 * have green LEDs when driving Talon Forward / Requesting Postiive Output Phase
		 * sensor to have positive increment when driving Talon Forward (Green LED)
		 */
		  m_ElevatorMotor.setSensorPhase(false);
		  m_ElevatorMotor.setInverted(false);
		/*
		 * Talon FX does not need sensor phase set for its integrated sensor
		 * This is because it will always be correct if the selected feedback device is integrated sensor (default value)
		 * and the user calls getSelectedSensor* to get the sensor's position/velocity.
		 * 
		 * https://phoenix-documentation.readthedocs.io/en/latest/ch14_MCSensor.html#sensor-phase
		 */
        // _talon.setSensorPhase(true);

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

		/* Set acceleration and vcruise velocity - see documentation */
		  m_ElevatorMotor.configMotionCruiseVelocity(15000, EncoderConstants.kTimeoutMs);
		  m_ElevatorMotor.configMotionAcceleration(6000, EncoderConstants.kTimeoutMs);

		/* Zero the sensor once on robot boot up */
		 m_ElevatorMotor.setSelectedSensorPosition(0, EncoderConstants.kPIDLoopIdx, EncoderConstants.kTimeoutMs);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  public void SetElevatorToPosition() {
      /* Motion Magic */
			/* 2048 ticks/rev * 10 Rotations in either direction */
			double targetPos = 1 * 2048;
			m_ElevatorMotor.set(TalonFXControlMode.MotionMagic, targetPos);
  }
}
