// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.controls.VelocityDutyCycle;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotContainer;
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
    // slot0Configs.kS = 0.06;
    slot0Configs.kV = 0.11;
    slot0Configs.kP = 0.05;
    slot0Configs.kI = 0;
    slot0Configs.kD = 0;

    m_LShootMotor.getConfigurator().apply(slot0Configs);
    m_RShootMotor.getConfigurator().apply(slot0Configs);
    //m_PhotoSensor = new DigitalInput(RobotMap.CANID.PHOTOSENSOR);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    RobotContainer.operatorInterface.OutputLSpeed.setDouble(m_LShootMotor.getVelocity().getValue());
  }




  /**
   * Run the left shooter motor at the provided percent of tested speed
   * @param percent
   */
  public void leftShootRun(double speed) {
    m_LShootMotor.setControl(m_motorVelocityControl.withVelocity(speed));
    m_currentSetpointL = speed;
  }
  /**
   * Run the right shooter motor at the provided percent of tested speed
   * @param percent
   */
  public void rightShootRun(double speed) {
    m_RShootMotor.setControl(m_motorVelocityControl.withVelocity(speed));
    m_currentSetpointR = speed;
  }

  /**
   * stops shooter
   */
  public void stopShooter() {
    rightShootRun(0);
    leftShootRun(0);
  }

  // /**
  //  * Checks if shooter motors are at speeds they are supposed to be at (Overload offers checking both speeds independantly)
  //  * @param targetSpeed speed in rotations per second
  //  * @return
  //  */
  // public boolean isShooterAtSpeed(double targetSpeed) {
  //   return 
  //     Math.abs(m_LShootMotor.getVelocity().getValueAsDouble() - targetSpeed) < allowedSpeedError && 
  //     Math.abs(m_RShootMotor.getVelocity().getValueAsDouble() - targetSpeed) < allowedSpeedError;   
  // }

  // /**
  //  * Checks if shooter motors are at speeds they are supposed to be at (Overload offers checking both speeds independantly)
  //  * @param leftSpeed target speed of left motor in rotations per second
  //  * @param leftSpeed target speed of left motor in rotations per second
  //  * @return
  //  */
  // public boolean isShooterAtSpeed(double leftSpeed, double rightSpeed) {
  //   return 
  //     Math.abs(m_LShootMotor.getVelocity().getValueAsDouble() - leftSpeed) < allowedSpeedError && 
  //     Math.abs(m_RShootMotor.getVelocity().getValueAsDouble() - rightSpeed) < allowedSpeedError;   
  // }

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
      Math.abs(m_LShootMotor.getVelocity().getValueAsDouble() - m_currentSetpointL) < allowedSpeedError && 
      Math.abs(m_RShootMotor.getVelocity().getValueAsDouble() - m_currentSetpointR) < allowedSpeedError;   
  }
}
