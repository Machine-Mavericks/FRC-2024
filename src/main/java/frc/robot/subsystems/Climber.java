// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.controls.VelocityDutyCycle;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;


public class Climber extends SubsystemBase {
  private VelocityDutyCycle m_mVelocityControl = new VelocityDutyCycle(0);

  // Motors and Sensors
  private TalonFX m_climbMotor;
  
  /** Creates a new Climber. */
  public Climber() {
    //m_climbMotor = new TalonFX(RobotMap.CANID.CLIMB_MOTOR);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  public void HANG_MOTOR_RUN(Double speed){
   m_climbMotor.setControl(m_mVelocityControl.withVelocity(speed));
  }

  public void HANG_OFF(){
    HANG_MOTOR_RUN(0.0);
  }
}
