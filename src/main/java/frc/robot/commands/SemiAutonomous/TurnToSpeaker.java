// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.SemiAutonomous;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotContainer;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Odometry;
import frc.robot.util.AprilTagMap;

public class TurnToSpeaker extends Command {
  double m_angle;

  // speed to rotate robot - determined by PID controller
  double m_rotatespeed;
  double m_angleerror;
  double m_endangle;
  double m_time;
  double m_timeout = 2;
  double offsetDegrees;
  
  boolean m_cameraControlled;

  // PID gains for rotating robot towards ball target
  double kp = 0.0125;
  double ki = 0.0;
  double kd = 0.00125;
  PIDController pidController = new PIDController(kp, ki, kd);

  /** Creates a new TurnToSpeaker. */
  public TurnToSpeaker() {
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    System.out.println("Init");
    // find speaker position
    Pose2d speakerPose;
    // find current position
    Pose2d currentPose=RobotContainer.odometry.getPose2d();
    if (DriverStation.getAlliance().get() == Alliance.Red){
      speakerPose=AprilTagMap.AprilTags[3];
      // find differences in position
      double xDif = currentPose.getX()-speakerPose.getX();
      double yDif = currentPose.getY()-speakerPose.getY();
      m_angle = (Math.atan2(-yDif,Math.abs(xDif)))/Odometry.DEGtoRAD;
      offsetDegrees = 0;
    } else {
      speakerPose=AprilTagMap.AprilTags[6];
      // find differences in position
      double xDif = currentPose.getX()-speakerPose.getX();
      double yDif = currentPose.getY()-speakerPose.getY();
      m_angle = (Math.atan2(yDif,xDif))/Odometry.DEGtoRAD;
      offsetDegrees = 180;
    }

    double currentAngle = RobotContainer.gyro.getYawDeg();
    // end angle is 180 or 0 degrees - current field relative angle + angle from speaker
    m_endangle = offsetDegrees-currentAngle+m_angle;
    if (m_endangle >180){
      m_endangle -= 360;
    } else if (m_endangle <-180){
      m_endangle += 360;
    }
    RobotContainer.odometry.m_angleAway.setDouble(m_endangle);
    
    // reset PID controller
    pidController.reset();

    m_angleerror = 0.0;
    m_time = 0.0;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    // increment time in command 
    m_time += 0.02;

    m_angleerror = m_endangle - RobotContainer.gyro.getYawDeg();

    // execute PID controller
    m_rotatespeed = pidController.calculate(m_angleerror);
    
    if (m_rotatespeed > 0.5)
      m_rotatespeed = 0.5;
    if (m_rotatespeed < -0.5)
      m_rotatespeed = -0.5;

    // rotate robot
    RobotContainer.drivetrain.drive(new Translation2d(0,0), -m_rotatespeed*Drivetrain.MAX_ANGULAR_VELOCITY_RADIANS_PER_SECOND, true);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    // stop robot
    RobotContainer.drivetrain.drive(new Translation2d(0,0), 0, false);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    // we are finished when within 2deg of target, or have timeed out
    return (Math.abs(m_angleerror) <=2.0);
  }
}
