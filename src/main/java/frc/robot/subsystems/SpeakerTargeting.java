// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import org.opencv.core.Point;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotContainer;
import frc.robot.util.AprilTagMap;
import frc.robot.util.Spline1D;

public class SpeakerTargeting extends SubsystemBase {
  private Limelight shotCamera;

  private static final int RED_SPEAKER_TAG_ID = 4;
  private static final int BLUE_SPEAKER_TAG_ID = 7;
  private static final double SHOT_ANGLE_TOLERANCE = 2;

  private double currentAngle = 0;
  private boolean speakerTargetPresent = false;
  private double currentHeightAngle = 0;

  private static final Spline1D ANGLE_CURVE = new Spline1D(new Point[]{
    new Point(1.8,0.2),
    new Point(2,0.18),
    new Point(2.7,0.138),
    new Point(3.1, 0.11),
    new Point(3.57, 0.087),
    new Point(4, 0.08),
    new Point(5.74, 0.063),
  });

  // private static final Spline1D LSPEED_CURVE = new Spline1D(new Point[]{
  //   new Point(30, 3000),
  //   new Point(4.5,)
  // });

  /** Creates a new SpeakerTargeting. */
  public SpeakerTargeting(Limelight shotCamera) {
    this.shotCamera = shotCamera;
  }

  @Override
  public void periodic() {
    if (DriverStation.isEnabled()) {
      UpdateShotData();
    }
  }

  private void UpdateShotData(){
    speakerTargetPresent = false;
    currentAngle = 0;
    currentHeightAngle = 0;
    if (shotCamera.isTargetPresent()) {
      // When using a pipeline that tracks all targets need to filter out which ones to use
      for (var tag : shotCamera.getFiducials()){
        
        //System.out.println(shotCamera.getLatestJSONDump().targetingResults.targets_Fiducials.length);
        if (tag.fiducialID == RED_SPEAKER_TAG_ID || tag.fiducialID == BLUE_SPEAKER_TAG_ID) {
          currentAngle = tag.tx;
          speakerTargetPresent = true;
          currentHeightAngle = tag.ty;
          return;
        }
      }
    }
  }

  public double getDesiredAngle(){
    return 0.352*Math.pow(getDistance(), -1.143);
  }

  public double getDesiredAngle(double distance){
    return 0.352*Math.pow(distance, -1.143);
  }
  
  public double getDesiredLSpeed(){
    return 3500;
  }

  public double getDesiredRSpeed(){
    return 5500;
  }

  /**
   * Decides if a target is present and in range
   * Note: Range can (and will) be adjusted post testing
   * 
   * @return boolean (true if target, false if not)
   */
  public boolean IsTarget() {
    //boolean target = shotCamera.isTargetPresent();
    RobotContainer.operatorinterface.SeesTarget.setBoolean(speakerTargetPresent);
    // double distance = EstimateDistance();

    // // we have valid target if distance is >2.9m
    // return (target == true && distance >= 2.90);
    return true;
  }

  public double getDistance(){
    // if (IsTarget()) {
    //   //double Dist = Math.pow(shotCamera.getTargetArea(), -0.562) * 1.5454;
    //   double Dist = (0.0 + 0.0103 * currentHeightAngle * currentHeightAngle - 0.0601 * currentHeightAngle + 2.0262);
    //   // Update shuffleboard
    //   RobotContainer.operatorinterface.TargetDistance.setDouble(Dist);
    //   //RobotContainer.operatorinterface.tY.setDouble(currentHeightAngle);
    //   return Dist+(RobotContainer.operatorinterface.DistanceAdjustment.getDouble(0)/10.0);
    // }
    // return 0;

    double m_distance;
    // find speaker position
    Pose2d speakerPose;
    // find current position
    Pose2d currentPose=RobotContainer.odometry.getPose2d();
    if (DriverStation.getAlliance().get() == Alliance.Red){
      speakerPose=AprilTagMap.AprilTags[3];
    } else {
      speakerPose=AprilTagMap.AprilTags[6];
    }
    // find differences in position
    double xDif = currentPose.getX()-speakerPose.getX();
    double yDif = currentPose.getY()-speakerPose.getY();
    // find distance
    m_distance = Math.sqrt(Math.pow(xDif,2)+Math.pow(yDif,2));

    System.out.println("Distance " + m_distance);
    System.out.println("Angle needed"+RobotContainer.speakertargeting.getDesiredAngle(m_distance));

    return m_distance;
  }

  public boolean IsAligned(){
    return IsTarget() && Math.abs(getSpeakerAngle()) < SHOT_ANGLE_TOLERANCE;
  }


  /**
   * finds angle of rotation to speaker
   * 
   * @return rotation angle
   */
  public double getSpeakerAngle() {
    double offsetDegrees;
    double m_angle;
    double m_endangle;


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

    return m_endangle;


    //double tx = shotCamera.getHorizontalTargetOffsetAngle();
    // double tx = currentAngle;
    // return tx;
  }

  /**
   * gets if the target can be seen, the shooter is at speed, and the cassette is at the right angle
   */
  public boolean IsSpunUp(){
    return IsTarget() &&
    (RobotContainer.cassetteangle.IsEffectorAtTarget(getDesiredAngle()) && RobotContainer.cassetteshooter.IsShooterAtSpeedSetpoint(getDesiredLSpeed(), getDesiredRSpeed()));
  }
}
