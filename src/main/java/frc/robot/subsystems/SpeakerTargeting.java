// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class SpeakerTargeting extends SubsystemBase {
  /** Creates a new SpeakerTargeting. */
  public SpeakerTargeting() {}

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  /**
   * Decides if a target is present and in range
   * Note: Range can (and will) be adjusted post testing
   * 
   * @return boolean (true if target, false if not)
   */
  public boolean IsTarget() {
    // boolean target = m_hubCamera.isTargetPresent();
    // double distance = EstimateDistance();

    // // we have valid target if distance is >2.9m
    // return (target == true && distance >= 2.90);
    return false;
  }


  /**
   * finds angle of rotation to speaker
   * 
   * @return rotation angle
   */
  public double getSpeakerAngle() {
    // double tx = m_hubCamera.getHorizontalTargetOffsetAngle();
    // return tx;
    return 0;
  }
}
