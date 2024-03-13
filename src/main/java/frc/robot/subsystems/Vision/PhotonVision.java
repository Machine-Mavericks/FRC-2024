// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.Vision;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.data.ApriltagCameras;
import frc.robot.data.ApriltagCameras.ApriltagCameraConstants;

public class PhotonVision extends SubsystemBase {
  private AprilTagFieldLayout aprilTagFieldLayout = AprilTagFields.k2024Crescendo.loadAprilTagLayoutField();
  private ApriltagCamera[] cameras;

  private static final int NUM_CAMERAS = 3;

  /** Creates a new PhotonVision. */
  public PhotonVision() {
    cameras = new ApriltagCamera[NUM_CAMERAS];

    TryAddCamera(ApriltagCameras.CAMERA_0);
    TryAddCamera(ApriltagCameras.CAMERA_1);
    TryAddCamera(ApriltagCameras.CAMERA_2);
  }

  private void TryAddCamera(ApriltagCameraConstants constants){
    cameras[0] = new ApriltagCamera(constants.name, constants.CameraTransform, aprilTagFieldLayout);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
