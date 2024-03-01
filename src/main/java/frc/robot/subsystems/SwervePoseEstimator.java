// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInLayouts;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardLayout;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.math.VecBuilder;
import frc.robot.RobotContainer;
import edu.wpi.first.math.trajectory.Trajectory;


// Swerve Position Estimator - New WPI Function for 2023
// Experimental subsystem for evaluation purposes - Jan 30/2023

public class SwervePoseEstimator extends SubsystemBase {
  // constant to convert degrees to radians
  final float DEGtoRAD = (float) (3.1415926 / 180.0);

  // create swerve position estimator object
  private SwerveDrivePoseEstimator m_estimator;

  // subsystem shuffleboard controls
  private GenericEntry m_robotX;
  private GenericEntry m_robotY;
  private GenericEntry m_robotAngle;
  private GenericEntry m_initialX;
  private GenericEntry m_initialY;
  private GenericEntry m_initialAngle;
  
  // field visualization object to display on shuffleboard
  private Field2d m_field;


  /** Creates a new SwervePosEstimator. */
  public SwervePoseEstimator() {

    // create 2d field object
    m_field = new Field2d();
    
    // create position estimator - set to (0,0,0)(x,y,ang)
    // initialize swerve drive odometry
    m_estimator = new SwerveDrivePoseEstimator(
                      RobotContainer.drivetrain.getKinematics(),
                      new Rotation2d(0.0),
                      RobotContainer.drivetrain.getSwervePositions(),
                      new Pose2d(0.0, 0.0, new Rotation2d(0.0)),
                      VecBuilder.fill(0.02, 0.02, 0.02),
                      VecBuilder.fill(0.2, 0.2, 0.3));

    // reset initial position
    setPosition(0.0,0.0,0.0,0.0);

    // create odometry shuffleboard page
    initializeShuffleboard();
  }


  // -------------------- Initialize and Update Odometry Methods

  /**
   * Initialize robot odometry use shuffleboard settings and current gyro angle
   */
  public void InitializefromShuffleboard() {
    setPosition(m_initialX.getDouble(0.0),
        m_initialY.getDouble(0.0),
        m_initialAngle.getDouble(0.0),
        0.0);
  } // get gyro angle from subsystem

  /** Initialize robot odometry to zero */
  public void InitializetoZero() {
    setPosition(0.0, 0.0, 0.0, 0.0);
  }

  /**
   * Used to set or reset odometry to fixed position
   * x, y displacement in m, robot angle in deg, gyro in deg
   */
  public void setPosition(double x, double y, double robotangle, double gyroangle) {

    // make robot position vector
    Pose2d position = new Pose2d(x, y, new Rotation2d(robotangle * DEGtoRAD));

    // set robot odometry
    m_estimator.resetPosition(new Rotation2d(gyroangle * DEGtoRAD),
                              RobotContainer.drivetrain.getSwervePositions(),
                              position);
  }

  /**
   * Adds vision measurement and confidence values based on data provided by the NVidia subsystem and AprilTagMap utilities
   * @param vision robot position on field based on apriltags
   * @param timeStamp timestamp from NVidia
   * @param distance distance from apriltag
   */
  public void addVision(Pose2d vision,double timeStamp, double distance){
    m_estimator.addVisionMeasurement(vision, timeStamp);
    double stdDevs = 0.1*distance;
    m_estimator.setVisionMeasurementStdDevs(VecBuilder.fill(stdDevs, stdDevs, stdDevs));
  }
  /** Update current robot dometry - called by scheduler at 50Hz */
   @Override
  public void periodic() {

    // get gyro angle (in degrees) and make rotation vector
    Rotation2d gyroangle = new Rotation2d(RobotContainer.gyro.getYaw() * DEGtoRAD);

    // update odometry with wheel drive and gyro
    m_estimator.update(gyroangle, RobotContainer.drivetrain.getSwervePositions());
     
    // update odemetry shuffleboard page
    updateShuffleboard();

    // update field representation
    //Pose2d pose = getPose2d();
    
    
    // if (DriverStation.getAlliance().get() == Alliance.Blue)
    //   m_field.setRobotPose(pose.getX(), pose.getY(), pose.getRotation());
    // else
    //   m_field.setRobotPose(16.54175-pose.getX(), 8.0137-pose.getY(), Rotation2d.fromDegrees(pose.getRotation().getDegrees()+180.0));  


  }

  
  // -------------------- Robot Current Odometry Access Methods --------------------

  /** return robot's current position vector Pose2d */
  public Pose2d getPose2d() {
    return m_estimator.getEstimatedPosition();
  }


  // -------------------- Field Visulation Methods --------------------


  // draws supplied trajectory on field widget on shuffleboard
  public void setFieldTrajectory(Trajectory trajectory)
  {
    m_field.getObject("Field").setTrajectory(trajectory);
  }

  // remote currently shown field trajectory
  public void deleteFieldTrajectory()
  {
    // remove by sending empty array pof poses to field object
    Pose2d poses[] = {};
    m_field.getObject("Field").setPoses(poses);
  }

  // -------------------- Subsystem Shuffleboard Methods --------------------

  /** Initialize subsystem shuffleboard page and controls */
  private void initializeShuffleboard() {
    // Create odometry page in shuffleboard
    ShuffleboardTab Tab = Shuffleboard.getTab("Swerve Estimator");

    // create controls to display robot position, angle, and gyro angle
    ShuffleboardLayout l1 = Tab.getLayout("Estimates", BuiltInLayouts.kList);
    l1.withPosition(0, 0);
    l1.withSize(1, 3);
    m_robotX = l1.add("X (m)", 0.0).getEntry();
    m_robotY = l1.add("Y (m)", 0.0).getEntry();
    m_robotAngle = l1.add("Angle(deg)", 0.0).getEntry();

    // Controls to set initial robot position and angle
    ShuffleboardLayout l2 = Tab.getLayout("Initial Position", BuiltInLayouts.kList);
    l2.withPosition(1, 0);
    l2.withSize(1, 3);
    m_initialX = l2.add("X2 (m)", 0.0).getEntry();           // eventually can use .addPersistent once code finalized
    m_initialY = l2.add("Y2 (m)", 0.0).getEntry();           // eventually can use .addPersentent once code finalized
    m_initialAngle = l2.add("Angle2(deg)", 0.0).getEntry();  // eventually can use .addPersentent once code finalized
  
    Tab.add("Field", m_field)
    .withPosition(2, 0)
    .withSize(5, 3);
  
  }

  /** Update subsystem shuffle board page with current odometry values */
  private void updateShuffleboard() {
    Pose2d vector = getPose2d();
    m_robotX.setDouble(vector.getX());
    m_robotY.setDouble(vector.getY());
    m_robotAngle.setDouble(vector.getRotation().getDegrees());
    m_field.setRobotPose(vector.getX(),vector.getY(),vector.getRotation());
  }
  
}
