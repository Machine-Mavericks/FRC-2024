// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkBase.IdleMode;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotMap;

public class CassetteIntake extends SubsystemBase {
  // Physical components
  private CANSparkMax m_IntakeMotor1;
  private CANSparkMax m_IntakeMotor2;
  private DigitalInput m_IntakeSensor;

  /* Intake speed in rps (Doesn't account for mechanism gearing on it's own) */
  private static double INTAKE_SPEED = 1; // TODO: tune
  
                                                                                                                  
  /** Creates a new CassetteIntake. */
  public CassetteIntake() {
    m_IntakeMotor1 = new CANSparkMax (RobotMap.CANID.IN1_CASSETTE,MotorType.kBrushless);
    m_IntakeMotor2 = new CANSparkMax(RobotMap.CANID.IN2_CASSETTE,MotorType.kBrushless);

    m_IntakeMotor1.setIdleMode(IdleMode.kBrake);
    m_IntakeMotor2.setIdleMode(IdleMode.kBrake);

    m_IntakeSensor = new DigitalInput(RobotMap.DIOPorts.DIO_IntakeSensor);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run

  }

  /**
   * Start or stop intake at constant (tested) speed
   * @param state Use -1 to invert, 0 to stop, and 1 to run forward
  */
  public void intakeRun(double state) {
    m_IntakeMotor1.set(INTAKE_SPEED * state); // Todo, this uses percent output, not velocity control as it should
    m_IntakeMotor2.set(INTAKE_SPEED * state);
  }
  public boolean NoteOrNoNote(){
    //seeing if theres a note  
    return !m_IntakeSensor.get(); 

  
  }


}

