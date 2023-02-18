// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Constants.OIConstants;
import frc.robot.commands.CloseGripper;
import frc.robot.commands.MoveToHighScore;
import frc.robot.commands.MoveToLowScore;
import frc.robot.commands.MoveToMidScore;
import frc.robot.commands.OpenGripper;
import frc.robot.subsystems.ArmStage1;
import frc.robot.subsystems.ArmStage2;
import frc.robot.subsystems.DriveTrain;
//import oi.limelightvision.limelight.frc.LimeLight;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.Gripper;

public class RobotContainer {
   //Subsystems that will be used in the rest of the file
  private final DriveTrain m_driveTrain = new DriveTrain();
  private final Elevator m_Elevator = new Elevator();
  private final ArmStage1 m_ArmStage1 = new ArmStage1();
  private final ArmStage2 m_ArmStage2 = new ArmStage2();
  private final Gripper m_Gripper = new Gripper(); 
  
  //Controllers that are used on the robot
  private final CommandXboxController m_driverController = new CommandXboxController(OIConstants.kdriveJoyStick);
  private final CommandXboxController m_actuatorController = new CommandXboxController(OIConstants.kactuatorJoyStick);

  //Commands ---private final IntakeBall m_intakeBall = new IntakeBall(m_intake);
  private final MoveToLowScore m_MoveToLowScore = new MoveToLowScore(m_Elevator,m_ArmStage2,m_ArmStage1);
  private final MoveToMidScore m_MoveToMidScore = new MoveToMidScore(m_Elevator,m_ArmStage2,m_ArmStage1);
  private final MoveToHighScore m_MoveToHighScore = new MoveToHighScore(m_Elevator,m_ArmStage2,m_ArmStage1);
   private final CloseGripper m_CloseGripper = new CloseGripper(m_Gripper);
  private final OpenGripper m_OpenGripper = new OpenGripper(m_Gripper);
 // private final TestElevator m_TestElevator = new TestElevator(m_ArmStage2, "Score_LL");
 // private final TestElevator2 m_TestElevator2 = new TestElevator2(m_ArmStage2, "Score_ML");

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
    //Configure Joysticks
   // m_actuatorController.a().onTrue(m_MoveToHighScore);
   // m_actuatorController.x().onTrue(m_MoveToLowScore);
   // m_actuatorController.y().onTrue(m_MoveToMidScore);
   // m_actuatorController.b().onTrue(m_MoveToHighScore);
    //Open Gripper
   // m_actuatorController.x().whileTrue(
     //   startEnd(() -> m_Gripper.OpenGripper(), m_Gripper.Stop(), m_Gripper));
       
       /*  controller.x().whileTrue(
          startEnd(() -> wristSubsystem.moveWr0ist(.3), wristSubsystem::stop, wristSubsystem)); */

    // PRESET MODE
    m_actuatorController.x().onTrue(m_MoveToLowScore);
    m_actuatorController.y().onTrue(m_MoveToMidScore);
    m_actuatorController.b().onTrue(m_MoveToHighScore);
   // m_actuatorController.a().onTrue(m_MoveToDefault);
    m_actuatorController.rightTrigger().onTrue(m_OpenGripper);
    m_actuatorController.leftTrigger().onTrue(m_CloseGripper);

    // USER CONTROL MODE
    /*m_actuatorController.x().onTrue(m_MoveToLowScore);
    m_actuatorController.y().onTrue(m_MoveToMidScore);
    m_actuatorController.b().onTrue(m_MoveToHighScore);
    m_actuatorController.a().onTrue(m_MoveToDefault);
    m_actuatorController.rightTrigger().onTrue(m_OpenGripper);
    m_actuatorController.leftTrigger().onTrue(m_CloseGripper);*/
  }
  
  public Command getAutonomousCommand() {
    return Commands.print("No autonomous command configured");
  }
}
