// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.controls.VelocityDutyCycle;
import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.motorcontrol.Spark;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotMap;

public class CassetteShooter extends SubsystemBase {
  // Physical components
  private TalonFX m_LShootMotor;
  private TalonFX m_RShootMotor;
  // private DigitalInput m_PhotoSensor;

  /* Percent shooter speed can be off by before being considered either too slow or fast */
  private static double allowedSpeedError = 0.05;

  private VelocityDutyCycle m_motorVelocityControl = new VelocityDutyCycle(0);

  /** Creates a new CassetteShooter. */
  public CassetteShooter() {
    m_LShootMotor = new TalonFX(RobotMap.CANID.L_OUT_CASSETTE);
    m_RShootMotor = new TalonFX(RobotMap.CANID.R_OUT_CASSETTE);
    //m_PhotoSensor = new DigitalInput(RobotMap.CANID.PHOTOSENSOR);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }




  /**
   * Run the left shooter motor at the provided percent of tested speed
   * @param percent
   */
  public void leftShootRun(double speed) {
    m_LShootMotor.set(speed);
    // m_LShootMotor.setControl(m_motorVelocityControl.withVelocity(speed));
  }

  /**
   * Run the right shooter motor at the provided percent of tested speed
   * @param percent
   */
  public void rightShootRun(double speed) {
    m_RShootMotor.set(speed);
    // m_RShootMotor.setControl(m_motorVelocityControl.withVelocity(speed));
  }

  /**
   * stops shooter
   */
  public void stopShooter() {
    rightShootRun(0);
    leftShootRun(0);
  }

  /**
   * Checks if shooter motors are at speeds they are supposed to be at (Overload offers checking both speeds independantly)
   * @param targetSpeed speed in rotations per second
   * @return
   */
  public boolean isShooterAtSpeed(double targetSpeed) {
    return 
      Math.abs(m_LShootMotor.getVelocity().getValueAsDouble() - targetSpeed) < allowedSpeedError && 
      Math.abs(m_RShootMotor.getVelocity().getValueAsDouble() - targetSpeed) < allowedSpeedError;   
  }

  /**
   * Checks if shooter motors are at speeds they are supposed to be at (Overload offers checking both speeds independantly)
   * @param leftSpeed target speed of left motor in rotations per second
   * @param leftSpeed target speed of left motor in rotations per second
   * @return
   */
  public boolean isShooterAtSpeed(double leftSpeed, double rightSpeed) {
    return 
      Math.abs(m_LShootMotor.getVelocity().getValueAsDouble() - leftSpeed) < allowedSpeedError && 
      Math.abs(m_RShootMotor.getVelocity().getValueAsDouble() - rightSpeed) < allowedSpeedError;   
  }

  /**
   * gets photosensor information
   * @return true if present, false if nothing
   */
  public boolean getSwitch() {
    // return m_PhotoSensor.get();
    return true;
  }
}
