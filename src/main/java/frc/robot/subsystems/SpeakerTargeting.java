// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;


public class SpeakerTargeting extends SubsystemBase {

  /** Creates a new SpeakerTargeting. */
  public SpeakerTargeting() {
  }

  @Override
  public void periodic() {
  }

  public double getDesiredAngle(double distance){
    return 0.352*Math.pow(distance, -1.143);
  }
  
  public double getDesiredLSpeed(){
    return 3500;
  }

  public double getDesiredRSpeed(){
    return 5500;
  }

}
