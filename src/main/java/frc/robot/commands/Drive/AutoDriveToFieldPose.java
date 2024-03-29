// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Drive;

import java.util.ArrayList;
import java.util.List;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInLayouts;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardLayout;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.wpilibj.Timer;

import frc.robot.RobotContainer;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.TrajGeneration;
import frc.robot.util.Utils;

public class AutoDriveToFieldPose extends Command { 

  private TrajGeneration trajgen_instance;
  private Trajectory m_Trajectory;
  private Pose2d m_TarPos; 
  private Pose2d m_initPos;

  private Pose2d m_target;
  
  private double m_speed;
  private double m_rotspeed;
  private double m_timeout;

  private double m_time;

    // subsystem shuffleboard controls
  private GenericEntry m_targetX;
  private GenericEntry m_targetY;
  private GenericEntry m_targetAngle;
  private GenericEntry m_xSpeed;
  private GenericEntry m_ySpeed;
  private GenericEntry m_rotSpeed;

  // final position tolerance (m) / angle tolerance (deg) to consider we have arrived at destination
  private final double m_positiontolerance = 0.20;
  private final double m_angletolerance = 2.0;

  // x, y, rotation PID controllers
  private PIDController m_xController = new PIDController(0.70, 0.4, 0.00);
  private PIDController m_yController = new PIDController(0.70, 0.4, 0.00);
  private PIDController m_rotController = new PIDController(0.01, 0.0001, 0.001);
  
  private double xSpeed;
  private double ySpeed;
  private double rotSpeed;
  private double TimeAtTarget;
  

  /** Creates a new AutoDriveToPoseCommand. 
   * Use this to have command to drive back to coordinate provided to it */
  public AutoDriveToFieldPose(Pose2d target, double speed, double rotationalspeed, double timeout) {
    addRequirements(RobotContainer.drivetrain);
    m_target = target;
    m_speed =  speed; //3.5;
    m_rotspeed = rotationalspeed; // 2.0; 
    m_timeout = timeout; 
    m_TarPos = target;

    // it crush the robot when run initializeShuffleboard();
    //initializeShuffleboard();
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_time = 0.0; 
    m_xController.reset();
    m_yController.reset();
    m_rotController.reset();
    TimeAtTarget=0.0; 
    //initializeShuffleboard();
    Pose2d m_initPos = RobotContainer.odometry.getPose2d(); 
    double limitVel = m_speed; //3.5;
    double limitAcc = 100.0;
    trajgen_instance = new TrajGeneration(limitVel, limitAcc); 
    
    // m_TarPos = new Pose2d(7.0,7.0, m_initPos.getRotation());

    m_Trajectory = trajgen_instance.genTraj(m_initPos, m_TarPos);

    // m_TarPos = new Pose2d(13.0,2.0, new Rotation2d(2.356125)); 
    // var interiorWaypoints = new ArrayList<Translation2d>();
    // test 2 , under starge
    // interiorWaypoints.add(new Translation2d(6.14,4.14));
    // interiorWaypoints.add(new Translation2d(9.25 ,1.85));
    // interiorWaypoints.add(new Translation2d(12.24 ,1.9 )); 

    // // test 1 from amp
    // interiorWaypoints.add(new Translation2d(5.6,7.0 ));
    // interiorWaypoints.add(new Translation2d(8.0 ,5.6 ));
    // interiorWaypoints.add(new Translation2d(8.0 ,2.6 ));
    // interiorWaypoints.add(new Translation2d(11.4 ,1.5 ));

    // m_Trajectory = trajgen_instance.genTrajThroughWaypoints(interiorWaypoints, m_initPos, m_TarPos);
        
    RobotContainer.odometry.setFieldTrajectory(m_Trajectory);
    // RobotContainer.leds.AtDestination = false;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    // increment time
    m_time += 0.02;

    if (m_time < m_Trajectory.getTotalTimeSeconds()) { 
      Trajectory.State state = m_Trajectory.sample(m_time); 
      // if( Math.abs ( m_initPos.getRotation().getRadians()-m_TarPos.getRotation().getRadians() ) < 0.05 ){
      //  m_target = new Pose2d( state.poseMeters.getX(),  state.poseMeters.getY(), m_initPos.getRotation());
      // }
      // else{
       m_target = state.poseMeters;
      // }
      System.out.println(" Time: " + m_time + "  x: " + m_target.getX());
      System.out.println(" Time: " + m_time + " y: " + m_target.getY());
    }

    Pose2d CurrentPos = RobotContainer.odometry.getPose2d(); 
    xSpeed = m_xController.calculate(m_target.getX()-CurrentPos.getX());
    ySpeed = m_yController.calculate(m_target.getY()-CurrentPos.getY());
    rotSpeed = m_rotController.calculate(Utils.AngleDifference(m_TarPos.getRotation().getDegrees(),CurrentPos.getRotation().getDegrees()));
    
    m_xController.setIntegratorRange(-40.0,40.0);
    m_xController.setIZone(0.4);
    m_yController.setIntegratorRange(-40.0,40.0);
    m_yController.setIZone(0.4);

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

    // it crush the robot when run updateShuffleboard();
    // updateShuffleboard();
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
    if(TimeAtTarget >=0.5 || (m_time >= m_timeout)){
      System.out.println("Go to Pose finished");
      // RobotContainer.leds.AtDestination = true;
      return true;
    }
    else{
      return false;
    }
    // return (TimeAtTarget >=0.5 || (m_time >= m_timeout)); 
  }

  /** Initialize subsystem shuffleboard page and controls */
  private void initializeShuffleboard() {
    // Create odometry page in shuffleboard
    ShuffleboardTab Tab = Shuffleboard.getTab("Auto drive");

 // Controls to set initial robot position and angle
    ShuffleboardLayout l2 = Tab.getLayout("Initial Position", BuiltInLayouts.kList);
    l2.withPosition(1, 0);
    l2.withSize(1, 3);
    m_targetX = l2.add("X3 (m)", 0.0).getEntry();           // eventually can use .addPersistent once code finalized
    m_targetY = l2.add("Y3 (m)", 0.0).getEntry();           // eventually can use .addPersentent once code finalized
    m_targetAngle = l2.add("Angle3(deg)", 0.0).getEntry();  // eventually can use .addPersentent once code finalized
    m_xSpeed = l2.add("xspeed",0.0).getEntry();
    m_ySpeed = l2.add("yspeed",0.0).getEntry();
    m_rotSpeed = l2.add("rotSpeed",0).getEntry();
  
  }

  /** Update subsystem shuffle board page with current odometry values */
  private void updateShuffleboard() {
    Pose2d vector = m_target;
    m_targetX.setDouble(vector.getX());
    m_targetY.setDouble(vector.getY());
    m_targetAngle.setDouble(vector.getRotation().getDegrees());
    m_xSpeed.setDouble(xSpeed);
    m_ySpeed.setDouble(ySpeed);
    m_rotSpeed.setDouble(rotSpeed);
    //m_field.setRobotPose(vector.getX(),vector.getY(),vector.getRotation());
  }
  
}