// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class CassetteEffector extends SubsystemBase {
  //TODO: figure out angle values
  static final int MAX_TOP_ANGLE = 0;
  static final int MAX_BOTTOM_ANGLE = 0;
  static final int NEUTRAL_ANGLE = 0;
  static final int GROUND_ANGLE = 0;
  static final int SOURCE_ANGLE = 0;
  static final int AMP_ANGLE = 0; //from flush against
  static final int SPEAKER_ANGLE = 0; //from flush against

  private TalonFX m_EffectorMotor;
  private CANcoder m_CANcoder;

  /** Creates a new CassetteEffector. */
  public CassetteEffector() {}

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  /**
   * sets the cassette to the angle provided
   * @param angle degrees
   */
  public void setAngle(int angle){}
}
