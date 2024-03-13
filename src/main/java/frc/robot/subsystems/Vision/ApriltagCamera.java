// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.Vision;

import java.util.Optional;

import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Transform3d;

/** TODO: docs */
public class ApriltagCamera {
    PhotonCamera cam;
    PhotonPoseEstimator photonPoseEstimator;

    Pose3d lastEstimatedPose;
    
    public ApriltagCamera(String cameraName, Transform3d cameraPosition, AprilTagFieldLayout fieldLayout){
        cam = new PhotonCamera(cameraName);

        photonPoseEstimator = new PhotonPoseEstimator(fieldLayout, PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR, cameraPosition);
        photonPoseEstimator.setMultiTagFallbackStrategy(PoseStrategy.CLOSEST_TO_LAST_POSE);
    }

    public Optional<EstimatedRobotPose> update(){
        photonPoseEstimator.setLastPose(lastEstimatedPose);
        var result = photonPoseEstimator.update();
        if (result.isPresent()) {
            lastEstimatedPose = result.get().estimatedPose;
        }
        return result;
    }
}
