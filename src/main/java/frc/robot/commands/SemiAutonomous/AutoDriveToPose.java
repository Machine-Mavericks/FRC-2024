// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.SemiAutonomous;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInLayouts;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardLayout;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotContainer;
import frc.robot.subsystems.Drivetrain;

public class AutoDriveToPose extends Command {
  
  private Pose2d m_target;
  private double m_speed;
  private double m_rotspeed;
  private double m_timeout;
  private boolean m_recallPoint;

  private double m_time;

    // subsystem shuffleboard controls
  private GenericEntry m_targetX;
  private GenericEntry m_targetY;
  private GenericEntry m_targetAngle;
  private GenericEntry m_xSpeed;
  private GenericEntry m_ySpeed;
  private GenericEntry m_rotSpeed;
  
  // field visualization object to display on shuffleboard
  private Field2d m_field;

  // final position tolerance (m) / angle tolerance (deg) to consider we have arrived at destination
  private final double m_positiontolerance = 0.12;
  private final double m_angletolerance = 2.0;

  // x, y, rotation PID controllers
  private PIDController m_xController = new PIDController(0.50, 0, 0.035);
  private PIDController m_yController = new PIDController(0.50, 0, 0.035);
  private PIDController m_rotController = new PIDController(0.01, 0, 0.001);

  private double xSpeed;
  private double ySpeed;
  private double rotSpeed;

  public static double AngleDifference( double angle1, double angle2 )
  {
      double diff = ( angle2 - angle1 + 180 ) % 360 - 180;
      return diff < -180 ? diff + 360 : diff;
  }

  /** Creates a new AutoDriveToPoseCommand. 
   * Use this to have command to drive back to coordinate provided to it */
  public AutoDriveToPose(Pose2d target, double speed, double rotationalspeed, double timeout) {
    // create 2d field object
    m_field = new Field2d();

    addRequirements(RobotContainer.drivetrain);
    m_target = target;
    m_speed = speed;
    m_rotspeed = rotationalspeed;
    m_timeout = timeout;
    m_recallPoint = false;
  }

  /** use this during teleop to go to pre-recorded position*/
  public AutoDriveToPose(double speed, double rotationalspeed)
  {
    m_speed = speed;
    m_rotspeed = rotationalspeed;
    m_recallPoint = true;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_time = 0.0;

    // Pose2d curr = RobotContainer.swervepose.getPose2d();
    // m_target = new Pose2d(curr.getX(),curr.getY(), curr.getRotation().rotateBy(new Rotation2d(Math.toRadians(180))));
    // recall previously saved point and use it as our destination
    if (m_recallPoint)
      m_target = RobotContainer.swervepose.RecallPoint(0);
    initializeShuffleboard();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    Pose2d curr = RobotContainer.swervepose.getPose2d();
    
    // increment time
    m_time += 0.02;

    // execute PIDs
    xSpeed = -m_xController.calculate(curr.getX()- m_target.getX() );
    ySpeed = -m_yController.calculate(curr.getY() - m_target.getY() );
    
    rotSpeed = m_rotController.calculate(-AngleDifference(curr.getRotation().getDegrees(),m_target.getRotation().getDegrees()));
    //double rotSpeed = m_rotController.calculate(curr.getRotation().getDegrees() - m_target.getRotation().getDegrees());

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
    RobotContainer.drivetrain.drive(new Translation2d(xSpeed*0.2*Drivetrain.MAX_VELOCITY_METERS_PER_SECOND,
                                                      ySpeed*0.2*Drivetrain.MAX_VELOCITY_METERS_PER_SECOND),
                                    rotSpeed*0.5*Drivetrain.MAX_ANGULAR_VELOCITY_RADIANS_PER_SECOND,
                                    true);

    updateShuffleboard();
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
    Pose2d curr = RobotContainer.swervepose.getPose2d();

    // we are finished if we are within erorr of target or command had timeed out
    return (((Math.abs(m_target.getX() - curr.getX()) <  m_positiontolerance) &&
          (Math.abs(m_target.getY() - curr.getY()) <  m_positiontolerance) &&
          (Math.abs(m_target.getRotation().getDegrees() - curr.getRotation().getDegrees()) < m_angletolerance)) ||
          (m_time >= m_timeout));
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
  
    Tab.add("Fieldz", m_field)
    .withPosition(2, 0)
    .withSize(5, 3);
  
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
    m_field.setRobotPose(vector.getX(),vector.getY(),vector.getRotation());
  }
  
}