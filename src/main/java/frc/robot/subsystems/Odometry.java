// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.math.VecBuilder;
import frc.robot.RobotContainer;


public class Odometry extends SubsystemBase {
  // constant to convert degrees to radians
  public final static float DEGtoRAD = (float) (3.1415926 / 180.0);

  // create swerve position estimator object
  private SwerveDrivePoseEstimator m_estimator;
  private boolean m_useCamAngle;


  /** Creates a new SwervePosEstimator. */
  public Odometry() {
   
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

  @Override
  public void periodic() {
    updateOdometry();
  }


  // -------------------- Update Odometry Methods


  /** Initialize robot odometry to zero */
  public void InitializetoZero() {
    setPosition(0.0, 0.0, 0.0, 0.0);
  }

  /* Called by the drivetrain synchronously with swerve module data updates to reduce latency */
  public void updateOdometry(){
    // get gyro angle (in degrees) and make rotation vector
    Rotation2d gyroangle = new Rotation2d(RobotContainer.gyro.getYaw() * DEGtoRAD);

    // get positions of all swerve modules from subsystem
    SwerveModulePosition[] positions = RobotContainer.drivetrain.getSwervePositions();
    
    // ensure we have proper length array of positions before accessing elements of array
    if (positions.length >=4) {
      // update the robot's odometry
      m_estimator.update(gyroangle, positions);
    }

    updateShuffleboard();
  }

  /**
   * set odometry mode true means camera values update angle
   * @param bool
   */
  public void setCamBool(boolean bool){
    m_useCamAngle = bool;
  }

  /**
   * Used to set or reset odometry to fixed position
   * x, y displacement in m, robot angle in deg, gyro in deg
   */
  public void SetPosition (Pose2d position, double gyroangle)
  {
    setPosition(position.getX(),position.getY(),position.getRotation().getDegrees(), gyroangle);
  }


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
   * @param distance area of apriltag in frame
   */
  public void addVision(Pose2d vision, double distance){

    if (m_useCamAngle){
      double stdDevs = 0.03*distance;
      //double velocity = Math.sqrt(Math.pow(RobotContainer.drivetrain.getFieldRelativeChassisSpeeds().vxMetersPerSecond,2)+Math.pow(RobotContainer.drivetrain.getFieldRelativeChassisSpeeds().vyMetersPerSecond,2)+Math.pow(RobotContainer.drivetrain.getFieldRelativeChassisSpeeds().omegaRadiansPerSecond,2));
      //double ATnum = 2/(RobotContainer.nvidia.GetNumberAprilTagsDetected()+0.1);
      //double finalStdDevs = stdDevs*velocity*ATnum+0.1;
      m_estimator.setVisionMeasurementStdDevs(VecBuilder.fill(stdDevs, stdDevs, 2.0*stdDevs));
      m_estimator.addVisionMeasurement(vision, Timer.getFPGATimestamp()); 

      // show apriltag estimate as 'dot' on field2d widget
     RobotContainer.operatorinterface.m_field.getObject("tag").setPose(vision);
     } else {
      double CurrentGyro = RobotContainer.gyro.getYaw()*DEGtoRAD;
      double VisionAngle = vision.getRotation().getRadians();
    
      Pose2d NewEstimate = new Pose2d(vision.getX(),vision.getY(),new Rotation2d(0.995*CurrentGyro + 0.005*VisionAngle));

      double stdDevs = 0.06*distance;
      m_estimator.setVisionMeasurementStdDevs(VecBuilder.fill(stdDevs, stdDevs, stdDevs));
      m_estimator.addVisionMeasurement(NewEstimate, Timer.getFPGATimestamp());

      // show apriltag estimate as 'dot' on field2d widget
      RobotContainer.operatorinterface.m_field.getObject("tag").setPose(vision);
     }
  }

  
  // -------------------- Robot Current Odometry Access Methods --------------------

  /** return robot's current position vector Pose2d */
  public Pose2d getPose2d() {
    return m_estimator.getEstimatedPosition();
  }

  Pose2d m_MemPoints[] = {new Pose2d(0,0,new Rotation2d(0.0)),
    new Pose2d(0,0,new Rotation2d(0.0)),
    new Pose2d(0,0,new Rotation2d(0.0)) };

  /** saves Pose2D coordinate for later recall
  * num = 0 to 2 (three memories available) */
  public void RecordPose2d(Pose2d point, int num)
  {
    if (num<m_MemPoints.length)
      m_MemPoints[num] = point;
  }

  /** recalls Pose2D coordinate previously saved 
  * num = 0 to 2 (three memories available) */
  public Pose2d RecallPoint(int num)
  {
    // return saved point.  If not in range, simply return 0,0,0 point
    if (num<m_MemPoints.length)
      return m_MemPoints[num];
    else
      return new Pose2d(0,0,new Rotation2d(0.0));
  }

  // -------------------- Subsystem Shuffleboard Methods --------------------

  /** Initialize subsystem shuffleboard page and controls */
  private void initializeShuffleboard() {}

  /** Update subsystem shuffle board page with current odometry values */
  private void updateShuffleboard() {
    Pose2d vector = getPose2d();
    RobotContainer.operatorinterface.m_robotX.setDouble(vector.getX());
    RobotContainer.operatorinterface.m_robotY.setDouble(vector.getY());
    RobotContainer.operatorinterface.m_robotAngle.setDouble(vector.getRotation().getDegrees());
    RobotContainer.operatorinterface.m_field.setRobotPose(vector.getX(),vector.getY(),vector.getRotation());
  }
  
}