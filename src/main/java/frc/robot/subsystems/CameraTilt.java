// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.Servo;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInLayouts;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardLayout;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotContainer;
import frc.robot.RobotMap;


public class CameraTilt extends SubsystemBase {

  // create servo
  Servo m_servo = new Servo(RobotMap.PWMPorts.CAMERA_SERVO_ID);

  public GenericEntry m_cameraTiltAngle;

  double m_currentangle=0.0;

  // PID gains for rotating robot towards note target
  private double kp = 0.011;  //0.010
  private double ki = 0.000;
  private double kd = 0.001;
  
  // camera tilt controller
  private PIDController pidController = new PIDController(kp, ki, kd);

  Timer notargettime = new Timer();

  /** Creates a new CameraTilt. */
  public CameraTilt() {
    initializeShuffleboard();
  
    // set default camera angle 180deg
    setAngle(180.0);

    notargettime.stop();
    notargettime.reset();
  }

  @Override
  public void periodic() {
    
    // go ahead and set camera angle
    m_servo.set(m_currentangle);
    
    // This method will be called once per scheduler run
    updateShuffleboard();

    // adjust camera
    if (RobotContainer.nvidia.IsNoteDetected())
    {
      notargettime.stop();
      notargettime.reset();
      //pidController.setP(0.006 / (10*getAngle()/100));
      
      double adjust = pidController.calculate(RobotContainer.nvidia.GetDetectedNoteY());
      
      if ((getAngle() + adjust)>177)
        setAngle(177.0);
      else
        setAngle(getAngle() + adjust); 


       
      //setAngle(adjust);
    }
    else
      {
        notargettime.start();
        pidController.reset();
        if (notargettime.hasElapsed(1.0))
          setAngle(175.0);
      }

    
  }

  // sets camera tilt to desired angle
  // Value is angle (deg)
  public void setAngle(double angle)
  {
     double TargetAngle = angle; 
      if (TargetAngle > 180.0)
          TargetAngle = 180.0;
      if (TargetAngle < 140.0)
          TargetAngle = 140.0;
    
    // convert 0 to 180deg range to 0 to 1.0 range
      m_currentangle = (TargetAngle/180.0);
  }

  /** returns current camera angle (in deg) */
  public double getAngle()
  {
    return m_currentangle*180.0;
  }

  
  // -------------------- Shuffleboard Commands --------------------

  /** Initialize subsystem shuffleboard page and controls */
private void initializeShuffleboard() { 
    // Create odometry page in shuffleboard
    ShuffleboardTab Tab = Shuffleboard.getTab("Camera Tilt");

    ShuffleboardLayout l1 = Tab.getLayout("Camera Tilt", BuiltInLayouts.kList);
    l1.withPosition(2, 0);
    l1.withSize(2, 4);
    m_cameraTiltAngle = l1.add("Angle(deg)", 0.0).getEntry();
  }

  /** Update subsystem shuffle board page with current Gyro values */
  private void updateShuffleboard() {
    // write current robot Gyro
    m_cameraTiltAngle.setDouble(getAngle());
  }
}
