// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.controls.VelocityDutyCycle;

import edu.wpi.first.wpilibj.motorcontrol.Spark;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotMap;

public class CassetteIntake extends SubsystemBase {
  // Physical components
  private Spark m_IntakeMotor1; // NOTE, the Spark class only seems to provide PWM control, need to figure out how to control with CAN
  private Spark m_IntakeMotor2;

  /* Intake speed in rps (Doesn't account for mechanism gearing on it's own) */
  private static double INTAKE_SPEED = 1; // TODO: tune

  /** Creates a new CassetteIntake. */
  public CassetteIntake() {
    // m_IntakeMotor1 = new Spark(RobotMap.CANID.IN1_CASSETTE); // If these lines are uncommented, robot crashes on startup since no sparks are plugged into PWM ports
    // m_IntakeMotor2 = new Spark(RobotMap.CANID.IN2_CASSETTE);
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
    // m_IntakeMotor1.set(INTAKE_SPEED * state); // Todo, this uses percent output, not velocity control as it should
    // m_IntakeMotor2.set(-INTAKE_SPEED * state);
  }
}
