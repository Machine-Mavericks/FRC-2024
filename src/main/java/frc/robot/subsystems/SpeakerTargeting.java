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


public class SpeakerTargeting extends SubsystemBase {

  /** Creates a new SpeakerTargeting. */
  public SpeakerTargeting() {
  }

  @Override
  public void periodic() {
  }

  public double getDesiredAngle(double distance){
    return 0.352*Math.pow(distance, -1.143);
  }

  /**
   * get angle away from speaker in degrees
   * @return
   */
  public double getSpeakerAngle(){
    // find current position
    Pose2d currentPose=RobotContainer.odometry.getPose2d();

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

    double finalangle = endangle;
    if (endangle > 180) {
      finalangle = 360 - endangle;
    } else if (endangle < -180) {
      finalangle = 360 + endangle;
    }
    return finalangle;
  }
  
  public double getDesiredLSpeed(){
    return 3500;
  }

  public double getDesiredRSpeed(){
    return 5500;
  }

}
