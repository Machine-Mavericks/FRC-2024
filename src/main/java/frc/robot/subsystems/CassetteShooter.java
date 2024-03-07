// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotMap;

public class CassetteShooter extends SubsystemBase {
  // Physical components
  private TalonFX m_LShootMotor;
  private TalonFX m_RShootMotor;

  /** The current speed the left motor is trying to reach (rotations per second) */
  private double m_currentSetpointL = 0;
  /** The current speed the right motor is trying to reach (rotations per second) */
  private double m_currentSetpointR = 0;

  /* Rotations per second shooter speed can be off by before being considered either too slow or fast */
  private static double allowedSpeedError = 5;

  private VelocityVoltage m_motorVelocityControl = new VelocityVoltage(0);


  /** Creates a new CassetteShooter. */
  public CassetteShooter() {
    m_LShootMotor = new TalonFX(RobotMap.CANID.L_OUT_CASSETTE);
    m_RShootMotor = new TalonFX(RobotMap.CANID.R_OUT_CASSETTE);
   
    var slot0Configs = new Slot0Configs();
    
    slot0Configs.kV = 0.11;
    slot0Configs.kP = 0.09;   // was 0.05
    slot0Configs.kI = .001;  // was 0.0 // was 0.005
    slot0Configs.kD = 0;

    m_LShootMotor.getConfigurator().apply(slot0Configs);
    m_RShootMotor.getConfigurator().apply(slot0Configs);
    //m_PhotoSensor = new DigitalInput(RobotMap.CANID.PHOTOSENSOR);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }




  /**
   * Run the left shooter motor at the provided percent of tested speed
   * @param speed Flywheel speed in rpm
   */
  public void leftShootRun(double speed) {
    m_LShootMotor.setControl(m_motorVelocityControl.withVelocity(speed / 60));
    m_currentSetpointL = speed / 60;
  }
  /**
   * Run the right shooter motor at the provided percent of tested speed
   * @param speed Flywheel speed in rpm
   */
  public void rightShootRun(double speed) {
    m_RShootMotor.setControl(m_motorVelocityControl.withVelocity(speed / 60));
    m_currentSetpointR = speed / 60;
  }

  /**
   * stops shooter
   */
  public void stopShooter() {
    rightShootRun(0);
    leftShootRun(0);
  }

  public double getSpeedL(){
    return m_LShootMotor.getVelocity().getValueAsDouble();
  }

    public double getSpeedR(){
    return m_RShootMotor.getVelocity().getValueAsDouble();
  }

  /**
   * Checks if shooter motors are at speeds specified by the current setpoint
   * @return
   */
  public boolean isShooterAtSpeedSetpoint() {
    return 
      Math.abs(getSpeedL() - m_currentSetpointL) < allowedSpeedError && 
      Math.abs(getSpeedR() - m_currentSetpointR) < allowedSpeedError;   
  }
}
