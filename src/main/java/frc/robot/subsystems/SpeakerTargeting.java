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
  
  public double getDesiredLSpeed(){
    return 3500;
  }

  public double getDesiredRSpeed(){
    return 5500;
  }

}
