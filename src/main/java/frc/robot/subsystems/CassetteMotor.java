// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.TalonFX;
import com.fasterxml.jackson.databind.type.PlaceholderForType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotMap;

public class CassetteMotor extends SubsystemBase {
  private TalonFX m_IntakeMotor;
  private TalonFX m_LShootMotor;
  private TalonFX m_RShootMotor;
  private PlaceholderForType m_PhotoSensor;
  private double INTAKE_SPEED;
  private double SHOOTER_SPEED;

  /** Creates a new CassetteMotor. */
  public CassetteMotor() {
    m_IntakeMotor = new TalonFX(RobotMap.CANID.IN_CASSETTE);
    m_LShootMotor = new TalonFX(RobotMap.CANID.L_OUT_CASSETTE);
    m_RShootMotor = new TalonFX(RobotMap.CANID.R_OUT_CASSETTE);
    m_PhotoSensor = new PlaceholderForType(RobotMap.CANID.PHOTOSENSOR);
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
    if (on) {
      m_IntakeMotor.set(INTAKE_SPEED);
    } else {
      m_IntakeMotor.set(0);
    }
  }

  /**
   * Run the left shooter motor at the provided percent of tested speed
   * @param percent
   */
  public void leftShootRun(int percent) {
    m_LShootMotor.set(SHOOTER_SPEED*percent);
  }

  /**
   * Run the right shooter motor at the provided percent of tested speed
   * @param percent
   */
  public void rightShootRun(int percent) {
    m_RShootMotor.set(SHOOTER_SPEED*percent);
  }

  /**
   * stops shooter
   */
  public void stopShooter() {
    rightShootRun(0);
    leftShootRun(0);
  }

  /**
   * gets photosensor information
   * @return true if present, false if nothing
   */
  public boolean getSwitch() {
    m_PhotoSensor.actualType();
    return true; // return if photosensor senses anything
  }
}
