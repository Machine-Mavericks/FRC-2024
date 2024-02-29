// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.PositionDutyCycle;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.AbsoluteSensorRangeValue;
import com.ctre.phoenix6.signals.ForwardLimitTypeValue;

import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInLayouts;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardLayout;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotMap;
import frc.robot.util.ShuffleUser;
import frc.robot.util.SubsystemShuffleboardManager;


// TODO LIST:
// [-] Figure out what the actual gear reduction is
// [-] 
public class CassetteEffector extends SubsystemBase implements ShuffleUser {
  // Shuffleboard
  private GenericEntry EffectorAngle;
  private GenericEntry CANCoderRotations;
  private GenericEntry EffectorSetpoint;

  //TODO: figure out angle values
  private static final double MAX_TOP_ANGLE = 20;
  private static final double MIN_BOTTOM_ANGLE = 50;
  private static final double NEUTRAL_ANGLE = 0;
  private static final double GROUND_ANGLE = 0;
  private static final double SOURCE_ANGLE = 0;
  private static final double AMP_ANGLE = 0; //from flush against
  private static final double SPEAKER_ANGLE = 0; //from flush against

  private static final Slot0Configs EFFECTOR_GAINS = new Slot0Configs()
  .withKP(20).withKI(0).withKD(0.05)
  .withKS(0).withKV(1).withKA(0);

  /* Multiplying mechanism rotations by this value produces motor rotations */
  private static final double CASETTE_EFFECTOR_REDUCTION = 80; // Let's guess 80:1
  /* The CANCoder offset in degrees */
  private static final double CASETTE_EFFECTOR_OFFSET = 0;

  /* Mechanism angle in degrees */
  private double currentAngle;

  /* Mechanism angle setpoint in degrees */
  private double currentAngleSetpoint;

  // Hardware
  private TalonFX m_EffectorMotor;
  private CANcoder m_CANcoder;

  private PositionDutyCycle m_motorPositionController = new PositionDutyCycle(0);

  /** Creates a new CassetteEffector. */
  public CassetteEffector() {
    m_EffectorMotor = new TalonFX(RobotMap.CANID.EFFECTOR_MOTOR);
    m_CANcoder = new CANcoder(RobotMap.CANID.EFFECTOR_CAN_CODER);

    TalonFXConfiguration effectorConfig = new TalonFXConfiguration();

    // Enable limit switches
    effectorConfig.HardwareLimitSwitch.ForwardLimitEnable = true;
    effectorConfig.HardwareLimitSwitch.ReverseLimitEnable = true;

    effectorConfig.HardwareLimitSwitch.ForwardLimitType = ForwardLimitTypeValue.NormallyClosed;
    effectorConfig.HardwareLimitSwitch.ForwardLimitType = ForwardLimitTypeValue.NormallyClosed;

    // Set built in mechanism ratio (hopefully correctly)
    effectorConfig.Feedback.SensorToMechanismRatio = 1 / CASETTE_EFFECTOR_REDUCTION;

    // Set oboard PID values
    effectorConfig.Slot0 = EFFECTOR_GAINS;

    m_EffectorMotor.getConfigurator().apply(effectorConfig);

    SubsystemShuffleboardManager.RegisterShuffleUser(this);

    resetInternalEncoder();
  }

  /**
   * Resets the internal encoder in the cassette motor to the value of the absolute encoder
   */
  private void resetInternalEncoder(){
    currentAngle = (m_CANcoder.getAbsolutePosition().getValueAsDouble() * 360) + CASETTE_EFFECTOR_OFFSET;
    currentAngleSetpoint = currentAngle;

    m_EffectorMotor.setPosition(currentAngle);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run

    updateEffectorState();
  }

  public void updateEffectorState(){
    currentAngle = m_EffectorMotor.getPosition().getValueAsDouble();
  }

  /**
   * sets the cassette to the angle provided
   * @param angle degrees
   */
  public void setAngle(double angle){
    // Clamp to allowable range
    currentAngleSetpoint = Math.max(MIN_BOTTOM_ANGLE, Math.min(MAX_TOP_ANGLE, angle));
    m_EffectorMotor.setControl(m_motorPositionController.withPosition(currentAngleSetpoint * CASETTE_EFFECTOR_REDUCTION));
  }

  @Override
  public void initializeShuffleboard() {
    // Create page in shuffleboard
    ShuffleboardTab Tab = Shuffleboard.getTab("Cassette Effector");

    ShuffleboardLayout layout = Tab.getLayout("State", BuiltInLayouts.kList).withPosition(0, 0).withSize(1, 3);
    CANCoderRotations = layout.add("Startup CANCoder Rotations", 0).getEntry();
    EffectorAngle = layout.add("Effector Angle", 0).getEntry();
    EffectorSetpoint = layout.add("Effector Setpoint", 0).getEntry();
  }

  @Override
  public void updateShuffleboard() {
    EffectorAngle.setDouble(currentAngle);
    EffectorSetpoint.setDouble(currentAngleSetpoint);
    CANCoderRotations.setDouble(m_CANcoder.getAbsolutePosition().getValueAsDouble());
  }
}
