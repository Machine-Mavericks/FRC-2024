// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.Map;

import org.opencv.core.Point;

import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.ForwardLimitTypeValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.ReverseLimitTypeValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;

import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.util.datalog.StringLogEntry;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInLayouts;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardLayout;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotMap;
import frc.robot.util.ShuffleUser;
import frc.robot.util.Spline1D;
import frc.robot.util.SubsystemShuffleboardManager;


// TODO LIST:
// [X] Figure out what the actual gear reduction is
// [X] Spline feedforward curve
// [X] Switch to Elastic because nobody likes Shuffleboard
public class CassetteEffector extends SubsystemBase implements ShuffleUser {
  // Shuffleboard
  private GenericEntry EffectorAngle;
  private GenericEntry CANCoderRotations;
  private GenericEntry EffectorSetpoint;
  private GenericEntry EffectorTarget;
  private GenericEntry MotorVoltage;
  private GenericEntry MotorAmps;
  private GenericEntry ClosedLoopError;

  private GenericEntry ManualFeedforward;

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
  .withKP(40).withKI(0).withKD(0.1)
  .withKS(0).withKV(0).withKA(0);

  private static final Slot0Configs FAKE_GAINS = new Slot0Configs()
  .withKP(0).withKI(0).withKD(0)
  .withKS(0).withKV(0).withKA(0);

  private static final Spline1D FEEDFORWARD_CURVE = new Spline1D(new Point[]{
    new Point(0.023,0.3),
    new Point(0.08, 0.42),
    new Point(0.12, 0.42),
    new Point(0.15, 0.7),
    new Point(0.28, 0.45)
  });

  /* Multiplying mechanism rotations by this value produces motor rotations */
  private static final double CASETTE_EFFECTOR_REDUCTION = 80; // Let's guess 80:1
  /* The CANCoder offset in degrees */
  private static final double CASETTE_EFFECTOR_OFFSET = 0;

  /* Mechanism angle in degrees */
  private double currentAngle;

  /* Mechanism angle setpoint in degrees */
  private double currentAngleSetpoint;

  private double internalSetpoint;

  // Hardware
  private TalonFX m_EffectorMotor;
  private CANcoder m_CANcoder;

  private MotionMagicVoltage m_motorPositionController = new MotionMagicVoltage(0).withSlot(0);
  //private PositionVoltage m_fakeController = new PositionVoltage(0).withSlot(0);

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

    // Motion magic cruise values
    effectorConfig.MotionMagic.MotionMagicAcceleration = 0.8;
    effectorConfig.MotionMagic.MotionMagicCruiseVelocity = 0.8;
    effectorConfig.MotionMagic.MotionMagicJerk = 5;

    m_EffectorMotor.getConfigurator().apply(effectorConfig);
    m_EffectorMotor.setNeutralMode(NeutralModeValue.Brake);

    // -------------------- CANCoder Configuration --------------------
    CANcoderConfiguration config = new CANcoderConfiguration();
    config.MagnetSensor.SensorDirection = SensorDirectionValue.Clockwise_Positive;
    m_CANcoder.getConfigurator().apply(config);

    resetInternalEncoder();
    
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
    // Update effector state
    currentAngle = m_EffectorMotor.getPosition().getValueAsDouble();
    
   
    if (DriverStation.isEnabled()) {
      //Run nonstop to adjust feedforward
      setAngle(EffectorTarget.getDouble(0.05));

      // if (m_EffectorMotor.getClosedLoopError().getValueAsDouble() < 0.01) {
      //   internalSetpoint = currentAngleSetpoint - 0.01;
      // }

      double feedForwardValue = FEEDFORWARD_CURVE.interpolate(currentAngle, true);
      m_EffectorMotor.setControl(m_motorPositionController.withPosition(internalSetpoint).withFeedForward(feedForwardValue*1.2));
    }
  }

  /**
   * sets the cassette to the angle provided
   * @param targetAngle degrees
   */
  public void setAngle(double targetAngle){
    if (currentAngleSetpoint == targetAngle) {
      return;
    }

    // Clamp to allowable range
    currentAngleSetpoint = Math.max(MIN_BOTTOM_ANGLE, Math.min(MAX_TOP_ANGLE, targetAngle));
    internalSetpoint = currentAngleSetpoint;

    // double feedForwardValue = (currentAngle < 0.19) ? 0.4 : 0.5; 

  }

  @Override
  public void initializeShuffleboard() {
    // Create page in shuffleboard
    ShuffleboardTab Tab = Shuffleboard.getTab("Cassette Effector");

    ShuffleboardLayout layout = Tab.getLayout("Stae", BuiltInLayouts.kList).withPosition(0, 0).withSize(2, 7);
    CANCoderRotations = layout.add("CANCoder Rotations", 0).getEntry();
    EffectorAngle = layout.add("Effector Angle", 0).getEntry();
    MotorVoltage = layout.add("Voltage", 0).getEntry();
    MotorAmps = layout.add("Amps", 0).getEntry();
    ClosedLoopError = layout.add("ClosedLoopError", 0).getEntry();

    EffectorTarget = Tab.add("setpoint", 0.05)
        .withPosition(8, 0)
        .withWidget(BuiltInWidgets.kNumberSlider)
        .withProperties(Map.of("min_value", CassetteEffector.MIN_BOTTOM_ANGLE, "max_value", CassetteEffector.MAX_TOP_ANGLE))
        .getEntry();

    ManualFeedforward = Tab.add("Janky feedforward", 0)
        .withPosition(8, 1)
        .withWidget(BuiltInWidgets.kNumberSlider)
        .withProperties(Map.of("min_value", 0, "max_value", 2))
        .getEntry();

    EffectorSetpoint = layout.add("Output Setpoint", 0).getEntry();
    //InterpVoltage = layout.add("Interpolated Volts", 0).getEntry();
  }

  @Override
  public void updateShuffleboard() {
    EffectorAngle.setDouble(currentAngle);
    EffectorSetpoint.setDouble(currentAngleSetpoint);
    CANCoderRotations.setDouble(m_CANcoder.getAbsolutePosition().getValueAsDouble());
    MotorVoltage.setDouble(m_EffectorMotor.getMotorVoltage().getValueAsDouble());
    MotorAmps.setDouble(m_EffectorMotor.getDutyCycle().getValueAsDouble());
    ClosedLoopError.setDouble(m_EffectorMotor.getClosedLoopError().getValueAsDouble());
  }
}
