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

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotContainer;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;


public class DriveToRelativePose extends Command {
  
  // target position, speed and rotational speed
  private Pose2d m_target;
  private double m_speed;
  private double m_rotspeed;
  
  // command timeout and counter
  private double m_timeout;
  private Timer m_Timer;

  // x, y, rotation PID controllers to get us to the intended destination
  private PIDController m_xController; 
  private PIDController m_yController;
  private PIDController m_rotController;

  // final position tolerance (m) / angle tolerance (deg) to consider we have arrived at destination
  private final double m_positiontolerance = 0.3;
  private final double m_angletolerance = 5.0;

  private SwerveDriveOdometry m_odometry;

  final float DEGtoRAD = (float) (3.1415926 / 180.0);

  public static double AngleDifference(double angle1, double angle2)
  {
    double diff = ( angle2 - angle1 + 180 ) % 360 - 180;
    return diff < -180 ? diff + 360 : diff;
  }
  
  
  /** Creates a new DrivetoRelativePose. */
  public DriveToRelativePose( Pose2d target,
                              double speed,
                              double rotationalspeed,
                              double timeout) {
    // this command requires use of swervedrive subsystem
    addRequirements(RobotContainer.drivetrain);
    
    // set up PIDs
    m_xController = new PIDController(2.5, 0.002, 0.12);
    m_yController = new PIDController(2.5, 0.002, 0.12);
    m_rotController = new PIDController(0.05, 0.001, 0.0000);
   
    // record target
    m_target = target;

    // record speed limits
    m_speed = speed;
    m_rotspeed = rotationalspeed;

    // create timer, and record timeout limit
    m_Timer = new Timer();
    m_timeout = timeout;
  }
  
  
  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    // reset and start in this command
    m_Timer.reset();
    m_Timer.start();
    
    m_odometry = new SwerveDriveOdometry(RobotContainer.drivetrain.getKinematics(),
                                      new Rotation2d(RobotContainer.gyro.getYaw() * DEGtoRAD),                                    
                                        RobotContainer.drivetrain.getSwervePositions() );


    m_odometry.resetPosition(new Rotation2d(RobotContainer.gyro.getYaw() * DEGtoRAD),
    RobotContainer.drivetrain.getSwervePositions(),
    new Pose2d(0.0,0.0,new Rotation2d(0.0)));
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    // get gyro angle (in degrees) and make rotation vector
    Rotation2d gyroangle = new Rotation2d(RobotContainer.gyro.getYaw() * DEGtoRAD);
    m_odometry.update(gyroangle, RobotContainer.drivetrain.getSwervePositions());

    // only integrate errors in within 10cm or 5deg of target
    if (Math.abs(m_target.getX() - m_odometry.getPoseMeters().getX())<0.10)
    m_xController.setI(0.2);
    else
    m_xController.setI(0.0);
    if (Math.abs(m_target.getY() - m_odometry.getPoseMeters().getY())<0.10)
    m_yController.setI(0.2);
    else
    m_yController.setI(0.0);
    if (Math.abs(AngleDifference(m_target.getRotation().getDegrees(),m_odometry.getPoseMeters().getRotation().getDegrees()))<5.0)
    m_rotController.setI(0.05);
    else
    m_rotController.setI(0.0);

    // execute PIDs
    double xSpeed = m_xController.calculate(m_target.getX() - m_odometry.getPoseMeters().getX() );
    double ySpeed = m_yController.calculate( m_target.getY() - m_odometry.getPoseMeters().getY());
    double rotSpeed = m_rotController.calculate(AngleDifference(m_target.getRotation().getDegrees(),m_odometry.getPoseMeters().getRotation().getDegrees()));

    // limit speeds to allowable
    if (xSpeed > m_speed)
      xSpeed = m_speed;
    if (xSpeed < -m_speed)
      xSpeed = -m_speed;
    if (ySpeed > m_speed)
      ySpeed = m_speed; 
    if (ySpeed < -m_speed)
      ySpeed = -m_speed;  
    if (rotSpeed >m_rotspeed)
      rotSpeed = m_rotspeed;
    if (rotSpeed < -m_rotspeed)
      rotSpeed = -m_rotspeed;

    // drive robot according to x,y,rot PID controller speeds
    RobotContainer.drivetrain.drive(new Translation2d(xSpeed, ySpeed), rotSpeed, false);  
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
 
    // we have finished path. Stop robot
    RobotContainer.drivetrain.drive(new Translation2d(0.0, 0.0), 0.0, false);

  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    Pose2d CurrentPosition = m_odometry.getPoseMeters();

    // we are finished if we are within erorr of target or command had timeed out
    return (((Math.abs(m_target.getX() - CurrentPosition.getX()) <  m_positiontolerance) &&
          (Math.abs(m_target.getY() - CurrentPosition.getY()) <  m_positiontolerance) &&
          (Math.abs(AngleDifference(m_target.getRotation().getDegrees() ,CurrentPosition.getRotation().getDegrees())) < m_angletolerance)) ||
          (m_Timer.hasElapsed(m_timeout)));
  }
}