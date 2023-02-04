// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.RunCommand;
import frc.robot.Constants.OIConstants;
import frc.robot.subsystems.DriveTrain;
//import oi.limelightvision.limelight.frc.LimeLight;

public class RobotContainer {
   //Subsystems that will be used in the rest of the file
  private final DriveTrain m_driveTrain = new DriveTrain();
  
  //Controllers that are used on the robot
  XboxController m_driverController = new XboxController(OIConstants.kdriveJoyStick);
  XboxController m_actuatorController = new XboxController(OIConstants.kactuatorJoyStick);

// LimeLight 
// The limelight is used a couple times here, the declaration is separated from the other subsystems
// because it is extra special
 // private final LimeLight m_limeLight = new LimeLight();

  public RobotContainer() {
    m_driveTrain.setDefaultCommand(new RunCommand(
      () -> m_driveTrain.drive(m_driverController.getRightY(), m_driverController.getRightX()),m_driveTrain));
    
      configureBindings();
  }

  private void configureBindings() {
    //Configure
  }

  public Command getAutonomousCommand() {
    return Commands.print("No autonomous command configured");
  }
}
