// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Drive;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotContainer;
import frc.robot.subsystems.Odometry;
import frc.robot.util.AprilTagMap;
import frc.robot.util.Utils;

public class TurnToSpeaker extends Command {

  // speed to rotate robot - determined by PID controller
  double m_rotatespeed;
  double m_angleerror;
  double m_endangle;
  double m_timeout = 2;

  double m_AtTargetTime;
  
  boolean m_cameraControlled;

  // PID gains for rotating robot towards ball target
  double kp = 0.10;   // 0.12 //0.09
  double ki = 0.22;   //0.28
  double kd = 0.002; //0.00125
  PIDController pidController = new PIDController(kp, ki, kd);

  /** Creates a new TurnToSpeaker. */
  public TurnToSpeaker() {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(RobotContainer.drivetrain);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {

    // reset PID controller
    pidController.reset();

    m_angleerror = 0.0;

    m_AtTargetTime = 0.0;

  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    
    // get error to target angle from speakertargeting
    m_angleerror = RobotContainer.speakertargeting.getSpeakerAngle(RobotContainer.odometry.getPose2d());

    if (Math.abs(m_angleerror) < 1.0)
      m_AtTargetTime += 0.02;
    else
      m_AtTargetTime = 0.0;

    // execute PID controller
    m_rotatespeed = pidController.calculate(m_angleerror);
    
    if (m_rotatespeed > 5.0)
      m_rotatespeed = 5.0;
    if (m_rotatespeed < -5.0)
      m_rotatespeed = -5.0;

    // rotate robot
    RobotContainer.drivetrain.drive(new Translation2d(0.0, 0.0), m_rotatespeed, true);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    // stop robot
    RobotContainer.drivetrain.drive(new Translation2d(0.0,0.0), 0.0, true);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    // we are at target for >0.5s
    return (m_AtTargetTime>=0.2);
  }
}