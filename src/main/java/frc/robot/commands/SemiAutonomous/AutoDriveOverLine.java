// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.SemiAutonomous;

import javax.lang.model.util.ElementScanner14;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInLayouts;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardLayout;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotContainer;
import frc.robot.subsystems.Drivetrain;
import frc.robot.util.Utils;

public class AutoDriveOverLine extends Command {
  
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
  private PIDController m_xController = new PIDController(0.1, 0.01, 0.00);
  private PIDController m_yController = new PIDController(0.1, 0.01, 0.00);
  private PIDController m_rotController = new PIDController(0.01, 0.0001, 0.001);
  
  private double xSpeed;
  private double ySpeed;
  private double rotSpeed;

  private double TimeAtTarget;


  /** Creates a new AutoDriveToPoseCommand. 
   * Use this to have command to drive back to coordinate provided to it */
  public AutoDriveOverLine(double speed, double rotationalspeed, double timeout) {

    addRequirements(RobotContainer.drivetrain);
    m_speed = speed;
    m_rotspeed = rotationalspeed;
    m_timeout = timeout;

  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_time = 0.0;

    // reset our Gyro - if on red team, we reset to 180deg
    if (DriverStation.getAlliance().get() == Alliance.Blue)
    {
    m_target = new Pose2d(RobotContainer.odometry.getPose2d().getX()+2.0,
          RobotContainer.odometry.getPose2d().getY(),
          RobotContainer.odometry.getPose2d().getRotation());
    }
    else
    {
      m_target = new Pose2d(RobotContainer.odometry.getPose2d().getX()-2.0,
          RobotContainer.odometry.getPose2d().getY(),
          RobotContainer.odometry.getPose2d().getRotation());
    }

    m_xController.reset();
    m_yController.reset();
    m_rotController.reset();
    TimeAtTarget=0.0;
    
    //initializeShuffleboard();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    Pose2d CurrentPos = RobotContainer.odometry.getPose2d();
    
    // increment time
    m_time += 0.02;

    double Xerrorabs = Math.abs(m_target.getX()-CurrentPos.getX());
    double Yerrorabs = Math.abs(m_target.getY()-CurrentPos.getY());

    
    double PgainMax = 0.45;
    double PgainMin = 0.15;
    double kOfGain = 0.05;
    double XIgain = 0.01, YIgain = 0.01;
    double XPgain = kOfGain*Xerrorabs+0.08;

    if (XPgain>PgainMax)
      XPgain=PgainMax;
    if (XPgain<PgainMin)
      XPgain=PgainMin;
    if(Xerrorabs>0.4)
      XIgain = 0.001;
    else
      XIgain = 0.02;

    double YPgain = kOfGain*Yerrorabs+0.08;
    if (YPgain>PgainMax)
      YPgain=PgainMax;
    if (YPgain<PgainMin)
      YPgain=PgainMin;
    if(Yerrorabs>0.4)
      YIgain = 0.001;
    else
      YIgain = 0.02;

    m_xController.setP(XPgain);
    m_xController.setI(XIgain);
    m_yController.setP(YPgain);
    m_yController.setI(YIgain);

    xSpeed = m_xController.calculate(m_target.getX()-CurrentPos.getX());
    ySpeed = m_yController.calculate(m_target.getY()-CurrentPos.getY());
    rotSpeed = m_rotController.calculate(Utils.AngleDifference(m_target.getRotation().getDegrees(),CurrentPos.getRotation().getDegrees()));
    
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
    if (  (Math.abs(m_target.getX() - CurrentPos.getX()) <  m_positiontolerance) &&
          (Math.abs(m_target.getY() - CurrentPos.getY()) <  m_positiontolerance) &&
          (Math.abs(Utils.AngleDifference(m_target.getRotation().getDegrees(),CurrentPos.getRotation().getDegrees())) < m_angletolerance))
      TimeAtTarget+=0.02;
    else
      TimeAtTarget=0.0;

    //updateShuffleboard();
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
    Pose2d curr = RobotContainer.odometry.getPose2d();

    // we are finished if we are within erorr of target or command had timeed out
    return (TimeAtTarget >=0.5 || (m_time >= m_timeout));
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
  
    //Tab.add("Fieldz", m_field)
    //.withPosition(2, 0)
    //.withSize(5, 3);
  
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