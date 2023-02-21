// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class CandleControl extends SubsystemBase {
  /** Creates a new CandleControl. */
  public CandleControl() {}

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  //Init color from driver station connection
  public void AllianceColor() {

  }
  
  //Nofity Human Player that drive team needs a cube.
  public void RequestCube() {

  }

  //Nofity Human Player that drive team needs a cone.
  public void RequestCone() {
    
  }

}
