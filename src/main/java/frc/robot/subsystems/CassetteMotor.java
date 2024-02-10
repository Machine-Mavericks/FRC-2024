// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class CassetteMotor extends SubsystemBase {

  /** Creates a new CassetteMotor. */
  public CassetteMotor() {
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  /**
   * Start or stop intake at constant (tested) speed
   * @param on
   */
  public void intakeRun(boolean on) {
  }

  /**
   * Run the left shooter motor at the provided percent of tested speed
   * @param percent
   */
  public void leftShootRun(int percent) {
  }

  /**
   * Run the right shooter motor at the provided percent of tested speed
   * @param percent
   */
  public void rightShootRun(int percent) {
  }

  /**
   * stops shooter
   */
  public void stopShooter() {
    rightShootRun(0);
    leftShootRun(0);
  }
}
