// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotContainer;
import frc.robot.util.AprilTagMap;
import frc.robot.util.Utils;


public class SpeakerTargeting extends SubsystemBase {

  /** Creates a new SpeakerTargeting. */
  public SpeakerTargeting() {
  }

  @Override
  public void periodic() {
  }

  public double getSpeakerDistance(){
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
    return Math.sqrt(Math.pow(xDif,2)+Math.pow(yDif,2));
  }

  public double getDesiredAngle(){
    return getDesiredAngle(getSpeakerDistance());
  }

  public double getDesiredAngle(double distance){
    return 0.352*Math.pow(distance, -1.143);
  }

  /**
   * get angle away from speaker in degrees
   * @return
   */
  public double getSpeakerAngle(Pose2d currentPose){

    // find speaker position
    Pose2d speakerPose;
    double endangle;
    
    if (DriverStation.getAlliance().get() == Alliance.Red){
      speakerPose=AprilTagMap.AprilTags[3];
      // find differences in position
      double xDif = speakerPose.getX()-currentPose.getX();
      double yDif = speakerPose.getY()-currentPose.getY();
      endangle = 0.0 + (Math.atan2(yDif,Math.abs(xDif)))/Odometry.DEGtoRAD;
     
    } else {
      speakerPose=AprilTagMap.AprilTags[6];
      // find differences in position
      double xDif = speakerPose.getX()-currentPose.getX();
      double yDif = speakerPose.getY()-currentPose.getY();
      endangle = 180.0 + (Math.atan2(-yDif,Math.abs(xDif)))/Odometry.DEGtoRAD;
    }

    // determine error to target angle
    double finalangle = Utils.AngleDifference(endangle, RobotContainer.odometry.getPose2d().getRotation().getDegrees());

    // set end angle in shuffleboard
    RobotContainer.odometry.m_angleAway.setDouble(finalangle);

    return finalangle;
  }
  
  public double getDesiredLSpeed(){
    return 3500;
  }

  public double getDesiredRSpeed(){
    return 5500;
  }

}
