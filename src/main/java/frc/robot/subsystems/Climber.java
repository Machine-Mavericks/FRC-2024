// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.ForwardLimitTypeValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotMap;


public class Climber extends SubsystemBase {
  private DutyCycleOut m_DutyCycle = new DutyCycleOut(0);
  //private static final double CLIMBER_RPM = 3000;

  // Motors and Sensors
  private TalonFX m_climbMotor;
  
  /** Creates a new Climber. */
  public Climber() {
    m_climbMotor = new TalonFX(RobotMap.CANID.CLIMB_MOTOR, Drivetrain.CAN_BUS_NAME);

    m_climbMotor.setNeutralMode(NeutralModeValue.Brake);

    TalonFXConfiguration config = new TalonFXConfiguration();
    m_climbMotor.getConfigurator().refresh(config);
    config.HardwareLimitSwitch.ForwardLimitEnable = true;
    config.HardwareLimitSwitch.ForwardLimitType = ForwardLimitTypeValue.NormallyClosed;
    config.Slot0.kP = 0.018;

    m_climbMotor.getConfigurator().apply(config);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  public void HANG_MOTOR_RUN(double speed){
   m_climbMotor.setControl(m_DutyCycle.withOutput(Math.min(1, Math.max(speed, -1))));
  }

  public void HANG_OFF(){
    HANG_MOTOR_RUN(0.0);
  }
}
