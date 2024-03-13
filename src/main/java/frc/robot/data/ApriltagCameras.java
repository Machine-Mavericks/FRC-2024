// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.data;

import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;

/** Constants for cameras mounted on robot */
public class ApriltagCameras {
    public static class ApriltagCameraConstants{
        public final String name;
        // CameraTransform uses the photon camera coordinate system, and is relative to the robot's center
        // See: https://docs.photonvision.org/en/latest/docs/apriltag-pipelines/coordinate-systems.html
        public final Transform3d CameraTransform;

        public ApriltagCameraConstants(String name, Transform3d CameraTransform){
            this.name = name;
            this.CameraTransform = CameraTransform;
        }
    }

    public static final ApriltagCameraConstants CAMERA_0 = new ApriltagCameraConstants(
        "ID0",
        new Transform3d(new Translation3d(0.0, 0.5, 0.0), new Rotation3d(0.0, 0.0, 0.0)) 
    );

    public static final ApriltagCameraConstants CAMERA_1 = new ApriltagCameraConstants(
        "ID1",
        new Transform3d(new Translation3d(0.0, 0.5, 0.0), new Rotation3d(0.0, 0.0, 0.0))
    );

    public static final ApriltagCameraConstants CAMERA_2 = new ApriltagCameraConstants(
        "ID2",
        new Transform3d(new Translation3d(0.0, 0.5, 0.0), new Rotation3d(0.0, 0.0, 0.0))
    );
}
