// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.motorcontrol.Spark;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotMap;

public class CassetteMotor extends SubsystemBase {
  private Spark m_IntakeMotor1;
  private Spark m_IntakeMotor2;
  private TalonFX m_LShootMotor;
  private TalonFX m_RShootMotor;
  private DigitalInput m_PhotoSensor;
  private double INTAKE_SPEED;
  private double SHOOTER_SPEED;

  /** Creates a new CassetteMotor. */
  public CassetteMotor() {
    m_IntakeMotor1 = new Spark(RobotMap.CANID.IN1_CASSETTE);
    m_IntakeMotor2 = new Spark(RobotMap.CANID.IN2_CASSETTE);
    m_LShootMotor = new TalonFX(RobotMap.CANID.L_OUT_CASSETTE);
    m_RShootMotor = new TalonFX(RobotMap.CANID.R_OUT_CASSETTE);
    m_PhotoSensor = new DigitalInput(RobotMap.CANID.PHOTOSENSOR);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  /**
   * Start or stop intake at constant (tested) speed
   * @param state Use -1 to invert, 0 to stop, and 1 to run forward
   */
  public void intakeRun(int state) {
    m_IntakeMotor1.set(INTAKE_SPEED * state);
    m_IntakeMotor2.set(-INTAKE_SPEED * state);
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
   * Checks if shooter motors are at speeds they are supposed to be at
   * @param lpercent percent speed the left motor should be running at
   * @param rpercent percent speed the right motor should be running at
   * @return
   */
  public boolean shooterAtSpeed(int lpercent, int rpercent) {
    return m_LShootMotor.getVelocity().getValueAsDouble()==(SHOOTER_SPEED*lpercent) && 
           m_RShootMotor.getVelocity().getValueAsDouble()==(SHOOTER_SPEED*rpercent);
  }

  /**
   * gets photosensor information
   * @return true if present, false if nothing
   */
  public boolean getSwitch() {
    return m_PhotoSensor.get();
  }
}
