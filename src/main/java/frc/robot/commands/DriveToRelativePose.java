// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.Odometry;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotContainer;
import frc.robot.util.Utils;

public class DriveToRelativePose extends Command {
  
 private Pose2d m_target;
 private double m_speed;
 private double m_rotspeed;
 
 private PIDController m_xController;
 private PIDController m_yController;
 private PIDController m_rotController;


 SwerveDriveOdometry m_Odometry;

  /** Creates a new DriveToRelativePose. */
  public DriveToRelativePose(Pose2d target){
    
    // set up the PID 
    m_xController = new PIDController(2.5, 0.002, 0.0);
    m_yController = new PIDController(2.5, 0.002, 0.0);
    m_rotController = new PIDController(0.05, 0.0001, 0.0000);

    m_target = target;
    

    m_Odometry = new SwerveDriveOdometry(RobotContainer.drivetrain.getKinematics(),
      new Rotation2d(0.0),
      RobotContainer.drivetrain.getSwervePositions(),
      new Pose2d(0.0,0.0,new Rotation2d(0.0)));

   addRequirements(RobotContainer.drivetrain); 
    
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {

    m_Odometry = new SwerveDriveOdometry(RobotContainer.drivetrain.getKinematics(),
      new Rotation2d(0.0),
      RobotContainer.drivetrain.getSwervePositions(),
      new Pose2d(0.0,0.0,new Rotation2d(0.0)));


  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    m_Odometry.update(new Rotation2d(RobotContainer.gyro.getYaw()*3.14/180.0),RobotContainer.drivetrain.getSwervePositions());
    
    //execut PID's 
    double xSpeed = m_xController.calculate(m_target.getX() - m_Odometry.getPoseMeters().getX());
    double ySpeed = m_yController.calculate(m_target.getY() - m_Odometry.getPoseMeters().getY());
    double rotSpeed = m_rotController.calculate(Utils.AngleDifference(m_target.getRotation().getDegrees(),m_Odometry.getPoseMeters().getRotation().getDegrees()));

    //limit speeds to allowable 
    if (xSpeed > m_speed)
      xSpeed = m_speed;
    if (xSpeed < -m_speed)
      xSpeed = -m_speed;
    if (ySpeed > m_speed)
      ySpeed = m_speed;
    if (ySpeed < -m_speed)
      ySpeed = -m_speed;
    if (rotSpeed > m_rotspeed)
      rotSpeed = m_rotspeed; 
    if (rotSpeed < -m_rotspeed)
      rotSpeed = -m_rotspeed;

    RobotContainer.drivetrain.drive(new Translation2d(xSpeed,ySpeed),rotSpeed,false);

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
