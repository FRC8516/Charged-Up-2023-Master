// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.LedLights;

public class CandleControl extends SubsystemBase {
 
  /** Creates a new CandleControl. */
  public CandleControl() {}

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  //Init color from driver station connection
  public void AllianceColor(Alliance m_Alliance) {
    if (m_Alliance == Alliance.Red) {
      
    }
    else {

    }
  }
  
  //change color to GREEN
  public void InTeleOpMode () {

  }

  //Nofity Human Player that drive team needs a cube.
  public void RequestCube() {

  }

  //Nofity Human Player that drive team needs a cone.
  public void RequestCone() {
    
  }

  //Used to make generic calls to change lights
  public void ChangeLedColor(String mColor) {

    switch (mColor) {
      case LedLights.Yellow:;
        this.RequestCone();
        break;
      case LedLights.Purple:;
        this.RequestCube();
        break;
      case LedLights.Blue:;

        break;
      case LedLights.Red:;

        break;
      case LedLights.Green:;

        break;
    }

  }

}
