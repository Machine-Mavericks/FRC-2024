// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.Map;

import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.ClosedLoopGeneralConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.PositionDutyCycle;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.AbsoluteSensorRangeValue;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.ForwardLimitTypeValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.ReverseLimitTypeValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;

import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.util.datalog.BooleanLogEntry;
import edu.wpi.first.util.datalog.StringLogEntry;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInLayouts;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardLayout;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotContainer;
import frc.robot.RobotMap;
import frc.robot.util.ShuffleUser;
import frc.robot.util.SubsystemShuffleboardManager;


// TODO LIST:
// [X] Figure out what the actual gear reduction is
// [-] 
public class CassetteEffector extends SubsystemBase implements ShuffleUser {
  // Shuffleboard
  private GenericEntry EffectorAngle;
  private GenericEntry CANCoderRotations;
  private GenericEntry EffectorSetpoint;
  private GenericEntry MotorVoltage;
  private GenericEntry MotorAmps;
  private GenericEntry ClosedLoopError;

  StringLogEntry myStringLog;

  //TODO: figure out angle values
  public static final double MAX_TOP_ANGLE = 0.26;
  public static final double MIN_BOTTOM_ANGLE = 0.023;
  private static final double NEUTRAL_ANGLE = 0;
  private static final double GROUND_ANGLE = 0;
  private static final double SOURCE_ANGLE = 0;
  private static final double AMP_ANGLE = 0; //from flush against
  private static final double SPEAKER_ANGLE = 0; //from flush against

  private static final Slot0Configs EFFECTOR_GAINS = new Slot0Configs()
  .withKP(30).withKI(0).withKD(0.1)
  .withKS(0).withKV(0).withKA(0);

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

  private MotionMagicVoltage m_motorPositionController = new MotionMagicVoltage(0).withSlot(0);

  /** Creates a new CassetteEffector. */
  public CassetteEffector() {
    m_EffectorMotor = new TalonFX(RobotMap.CANID.EFFECTOR_MOTOR);
    m_CANcoder = new CANcoder(RobotMap.CANID.EFFECTOR_CAN_CODER);

    // -------------------- Motor Configuration --------------------
    TalonFXConfiguration effectorConfig = new TalonFXConfiguration();

    // Enable limit switches
    effectorConfig.HardwareLimitSwitch.ForwardLimitEnable = true;
    effectorConfig.HardwareLimitSwitch.ReverseLimitEnable = true;

    effectorConfig.HardwareLimitSwitch.ForwardLimitType = ForwardLimitTypeValue.NormallyClosed;
    effectorConfig.HardwareLimitSwitch.ReverseLimitType = ReverseLimitTypeValue.NormallyClosed;

    effectorConfig.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
  
    // Set built in mechanism ratio (hopefully correctly)
    //effectorConfig.Feedback.SensorToMechanismRatio = CASETTE_EFFECTOR_REDUCTION;

    // Setup remote CANCoder
    effectorConfig.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.RemoteCANcoder;
    effectorConfig.Feedback.FeedbackRemoteSensorID = m_CANcoder.getDeviceID();

    // Set oboard PID values
    effectorConfig.Slot0 = EFFECTOR_GAINS;

    effectorConfig.MotionMagic.MotionMagicAcceleration = 0.4;
    effectorConfig.MotionMagic.MotionMagicCruiseVelocity = 0.4;
    effectorConfig.MotionMagic.MotionMagicJerk = 3;

    m_EffectorMotor.getConfigurator().apply(effectorConfig);

    // -------------------- CANCoder Configuration --------------------
    CANcoderConfiguration config = new CANcoderConfiguration();
    config.MagnetSensor.SensorDirection = SensorDirectionValue.Clockwise_Positive;
    m_CANcoder.getConfigurator().apply(config);

    resetInternalEncoder();
    
    // Logging
    // DataLogManager.start();
    // var Log = DataLogManager.getLog();
    // myStringLog = new StringLogEntry(Log, "/effector/feedforward");

    SubsystemShuffleboardManager.RegisterShuffleUser(this);
  }

  /**
   * Resets the internal encoder in the cassette motor to the value of the absolute encoder
   */
  private void resetInternalEncoder(){
    currentAngle = m_CANcoder.getAbsolutePosition().getValueAsDouble() + (CASETTE_EFFECTOR_OFFSET / 360);
    currentAngleSetpoint = currentAngle;

    //m_EffectorMotor.setPosition(currentAngle);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    updateEffectorState();
   
    if (DriverStation.isEnabled()) {
      if (m_EffectorMotor.getClosedLoopError().getValueAsDouble() < 0.005) {
        //myStringLog.append(currentAngle + " " + m_EffectorMotor.getMotorVoltage());
        //System.out.println("Logging at setpoint");
      }
      
      //Run nonstop to adjust feedforward
      setAngle(RobotContainer.shuffleboard.EffectorTarget.getDouble(0.05));
    }
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
    //System.out.println("Setting Motor to " + currentAngleSetpoint); //(currentAngleSetpoint < 0.15) ? 0.6 : 0.1)
    m_EffectorMotor.setControl(m_motorPositionController.withPosition(currentAngleSetpoint).withFeedForward((currentAngle < 0.19) ? 0.4 : 0.5));
  }

  @Override
  public void initializeShuffleboard() {
    // Create page in shuffleboard
    ShuffleboardTab Tab = Shuffleboard.getTab("Cassette Effector");

    ShuffleboardLayout layout = Tab.getLayout("State", BuiltInLayouts.kList).withPosition(0, 0).withSize(2, 6);
    CANCoderRotations = layout.add("CANCoder Rotations", 0).getEntry();
    EffectorAngle = layout.add("Effector Angle", 0).getEntry();
    MotorVoltage = layout.add("Voltage", 0).getEntry();
    MotorAmps = layout.add("Amps", 0).getEntry();
    ClosedLoopError = layout.add("ClosedLoopError", 0).getEntry();

    // System.out.println("Init Setpoin");
    // EffectorTargetTEST = Tab.add("setpointTEST", 0.05)
    // .withPosition(8, 0)
    // .withWidget(BuiltInWidgets.kNumberSlider)
    // .withProperties(Map.of("min", CassetteEffector.MIN_BOTTOM_ANGLE, "max", CassetteEffector.MAX_TOP_ANGLE))
    // .getEntry();

    //EffectorSetpoint = layout.add("Effector Setpoint", 0).getEntry();

    EffectorSetpoint = layout.add("Output Setpoint", 0).getEntry();
  }

  @Override
  public void updateShuffleboard() {
    EffectorAngle.setDouble(currentAngle);
    EffectorSetpoint.setDouble(currentAngleSetpoint);
    CANCoderRotations.setDouble(m_CANcoder.getAbsolutePosition().getValueAsDouble());
    MotorVoltage.setDouble(m_EffectorMotor.getMotorVoltage().getValueAsDouble());
    MotorAmps.setDouble(m_EffectorMotor.getTorqueCurrent().getValueAsDouble());
    ClosedLoopError.setDouble(m_EffectorMotor.getClosedLoopError().getValueAsDouble());
  }
}
