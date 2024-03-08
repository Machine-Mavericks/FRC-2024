// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.controls.VelocityDutyCycle;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.networktables.ProtobufPublisher;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.motorcontrol.Spark;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotMap;


public class Hang extends SubsystemBase {
  private VelocityDutyCycle m_mVelocityControl = new VelocityDutyCycle(0);

   // Motors and Sensors
      private TalonFX HANG_MOTOR;
      private DigitalInput tbdlimitswitch;
  /** Creates a new Hang. */
  public Hang() {
    HANG_MOTOR = new TalonFX(RobotMap.CANID.HANG_MOTOR);
    //HANG_MOTOR = new input (RobotMap)
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  public void HANG_MOTOR_RUN(Double speed){
   HANG_MOTOR.setControl(m_mVelocityControl.withVelocity(speed));
  }

  public void HANG_OFF(){
    HANG_MOTOR_RUN(0.0);
  }
}
