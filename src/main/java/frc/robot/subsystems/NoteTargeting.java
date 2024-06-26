// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class NoteTargeting extends SubsystemBase {
  private Limelight intakeCamera;

  /** Creates a new SpeakerTargeting. */
  public NoteTargeting(Limelight intakeCamera) {
    this.intakeCamera = intakeCamera;
  }

  @Override
  public void periodic() {}

  /**
   * Decides if a target is present and in range
   * Note: Range can (and will) be adjusted post testing
   * 
   * @return boolean (true if target, false if not)
   */
  public boolean IsTarget() {
    boolean target = intakeCamera.isTargetPresent();
    return (target);
  }

  /**
   * finds angle of rotation to note
   * 
   * @return rotation angle
   */
  public double getNoteHorAngle() {
    double tx = intakeCamera.getHorizontalTargetOffsetAngle();
    return tx;
  }

  public double getNoteVertAngle() {
    double ty = intakeCamera.getVerticalTargetOffsetAngle();
    return ty;
  }
}