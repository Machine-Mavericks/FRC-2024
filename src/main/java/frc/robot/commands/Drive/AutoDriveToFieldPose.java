// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Drive;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotContainer;
import frc.robot.subsystems.Drivetrain;
import frc.robot.util.Utils;

public class AutoDriveToFieldPose extends Command { 

  private Pose2d m_TarPos; 

  private Pose2d m_target;
  
  private double m_XspeedLimit;
  private double m_YspeedLimit;
  private double m_rotspeed;
  private double m_timeout;

  private double m_time;


  // final position tolerance (m) / angle tolerance (deg) to consider we have arrived at destination
  private final double m_positiontolerance = 0.10;
  private final double m_angletolerance = 2.0;

  // x, y, rotation PID controllers
  private PIDController m_xController = new PIDController(1.25, 0.4, 0.00);
  private PIDController m_yController = new PIDController(1.25, 0.4, 0.00);
  private PIDController m_rotController = new PIDController(0.01, 0.0001, 0.001);
  
  private double xSpeed;
  private double ySpeed;
  private double rotSpeed;
  private double TimeAtTarget;
  
  private double oldx, oldy, oldrot;

  /** Creates a new AutoDriveToPoseCommand. 
   * Use this to have command to drive back to coordinate provided to it */
  public AutoDriveToFieldPose(Pose2d target, double speed, double rotationalspeed, double timeout) {
    addRequirements(RobotContainer.drivetrain);
    m_target = target;
    m_XspeedLimit =  speed;  m_YspeedLimit = speed;
    m_rotspeed = rotationalspeed;
    m_timeout = timeout; 
    m_TarPos = target;
  }

/** Creates a new AutoDriveToPoseCommand. 
   * Use this to have command to drive back to coordinate provided to it */
  public AutoDriveToFieldPose(Pose2d target, double Xspeed, double Yspeed, double rotationalspeed, double timeout) {
    addRequirements(RobotContainer.drivetrain);
    m_target = target;
    m_XspeedLimit =  Xspeed;  m_YspeedLimit = Yspeed;
    m_rotspeed = rotationalspeed;
    m_timeout = timeout; 
    m_TarPos = target;
  }


  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_time = 0.0; 
    m_xController.reset();
    m_yController.reset();
    m_rotController.reset();
    TimeAtTarget=0.0; 
    oldx = 0.0; oldy = 0.0; oldrot=0.0;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    // increment time
    m_time += 0.02;

    Pose2d CurrentPos = RobotContainer.odometry.getPose2d(); 
    xSpeed = m_xController.calculate(m_target.getX()-CurrentPos.getX());
    ySpeed = m_yController.calculate(m_target.getY()-CurrentPos.getY());
    rotSpeed = m_rotController.calculate(Utils.AngleDifference(m_TarPos.getRotation().getDegrees(),CurrentPos.getRotation().getDegrees()));
    
    m_xController.setIntegratorRange(-40.0,40.0);
    m_xController.setIZone(0.4);
    m_yController.setIntegratorRange(-40.0,40.0);
    m_yController.setIZone(0.4);

    // limit rate of change
    if (xSpeed > 0.0 && xSpeed > oldx + 0.01)
      xSpeed = oldx+0.01;
    if (xSpeed <0.0 && xSpeed < oldx - 0.01)
      xSpeed = oldx-0.01;
    if (ySpeed > 0.0 && ySpeed > oldy + 0.01)
      ySpeed = oldy+0.01;
    if (ySpeed <0.0 && ySpeed < oldy - 0.01)
      ySpeed = oldy-0.01;
    if (rotSpeed > 0.0 && rotSpeed > oldrot + 0.01)
      rotSpeed = oldrot+0.01;
    if (rotSpeed <0.0 && rotSpeed < oldrot - 0.01)
      rotSpeed = oldrot-0.01;
    
      oldx = xSpeed; oldy = ySpeed; oldrot = rotSpeed;


    // limit speeds to allowable
    if (xSpeed > m_XspeedLimit)
      xSpeed = m_XspeedLimit;
    if (xSpeed < -m_XspeedLimit)
      xSpeed = -m_XspeedLimit;
    if (ySpeed > m_YspeedLimit)
      ySpeed = m_YspeedLimit; 
    if (ySpeed < -m_YspeedLimit)
      ySpeed = -m_YspeedLimit;  
    if (rotSpeed >m_rotspeed)
      rotSpeed = m_rotspeed;
    if (rotSpeed < -m_rotspeed)
      rotSpeed = -m_rotspeed;

    //drive robot according to x,y,rot PID controller speeds
    RobotContainer.drivetrain.drive(new Translation2d(xSpeed*Drivetrain.MAX_VELOCITY_METERS_PER_SECOND,
                                                       ySpeed*Drivetrain.MAX_VELOCITY_METERS_PER_SECOND),
                                     rotSpeed*Drivetrain.MAX_ANGULAR_VELOCITY_RADIANS_PER_SECOND,
                                     true);

    // are we at target - if so, increment time, if not reset
    if (  (Math.abs(m_TarPos.getX() - CurrentPos.getX()) <  m_positiontolerance) &&
          (Math.abs(m_TarPos.getY() - CurrentPos.getY()) <  m_positiontolerance) &&
          (Math.abs(Utils.AngleDifference(m_TarPos.getRotation().getDegrees(),CurrentPos.getRotation().getDegrees())) < m_angletolerance))
      TimeAtTarget+=0.02;
    else
      TimeAtTarget=0.0;
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    // we have finished path. Stop robot
    RobotContainer.drivetrain.drive(new Translation2d(0, 0), 0, false);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() { 
    // we are finished if we are within erorr of target or command had timeed out
    if(TimeAtTarget >=0.35 || (m_time >= m_timeout)){
      System.out.println("Go to Pose finished");
      return true;
    }
    else{
      return false;
    }
    // return (TimeAtTarget >=0.5 || (m_time >= m_timeout)); 
  }

  
}