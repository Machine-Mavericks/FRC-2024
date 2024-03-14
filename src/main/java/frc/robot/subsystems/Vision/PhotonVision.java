// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.Vision;

import java.util.ArrayList;

import org.photonvision.EstimatedRobotPose;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotContainer;
import frc.robot.data.ApriltagCameras;
import frc.robot.data.ApriltagCameras.ApriltagCameraConstants;

public class PhotonVision extends SubsystemBase {
  private AprilTagFieldLayout aprilTagFieldLayout = AprilTagFields.k2024Crescendo.loadAprilTagLayoutField();
  private ArrayList<ApriltagCamera> cameras = new ArrayList<>();

  /** Creates a new PhotonVision. */
  public PhotonVision() {
    // Should be resilient to missing cameras
    AddCamera(ApriltagCameras.CAMERA_0);
    AddCamera(ApriltagCameras.CAMERA_1);
    AddCamera(ApriltagCameras.CAMERA_2);
  }

  private void AddCamera(ApriltagCameraConstants constants){
    cameras.add(new ApriltagCamera(constants.name, constants.CameraTransform, aprilTagFieldLayout));
    RobotContainer.operatorinterface.AddCameraEntry(constants.name);
  }

  @Override
  public void periodic() {
    if (cameras.size() > 0) { // Really pedantically make sure we never div by 0 for the average position (this will never happen)
      ApplyAllVisionMeasurements();
    }
  }

  private void ApplyAllVisionMeasurements(){
    Translation2d averageTransform = new Translation2d();
    double averageRotation = 0.0;

    for (var camera : cameras){
      var poseData = camera.update();
      // if a camera loses connection, we just don't apply it's pose to odometry
      if (poseData.isPresent()) {
        EstimatedRobotPose estimatedRobotPose = poseData.get();

        // Area is used for standard deviations, just guess and throw in the average of all the tags used for the pose
        double areaSum = 0;
        for (var target : poseData.get().targetsUsed){
          areaSum += (target.getArea() / 100); // Area is in a range of 0-100, converts to a more sensible 0-1 (same as limelight)
        }
        
        Pose2d fieldPose2d = new Pose2d(
          estimatedRobotPose.estimatedPose.getX(), 
          estimatedRobotPose.estimatedPose.getY(), 
          new Rotation2d(estimatedRobotPose.estimatedPose.getRotation().getZ()));

        averageTransform = averageTransform.plus(fieldPose2d.getTranslation());
        averageRotation += fieldPose2d.getRotation().getDegrees();

        RobotContainer.swervepose.addVision(fieldPose2d, areaSum / (double)estimatedRobotPose.targetsUsed.size());
      }
    }

    // Big long formatting one-liner
    RobotContainer.operatorinterface.VisionPoseAverage.setString(averageTransform.div(cameras.size()).toString() + " Rotation2d(Deg:" + (averageRotation / cameras.size()) + ")");
  }
}
