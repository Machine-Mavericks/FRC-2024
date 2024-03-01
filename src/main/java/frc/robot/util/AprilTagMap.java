// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.util;

import javax.swing.text.html.HTML.Tag;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;

/** Add your docs here. */
public class AprilTagMap {

    /**
     * Convert inches to meters
     * @param in
     * @return meters
     */
    public static double intom(double in){
        return in/39.37;
    }
    /**
     * Map of april tags relatie to field, remember, values for ATs 1-16 are 0-15
     */
    static Pose2d AprilTags[] = {
        new Pose2d(0,0,new Rotation2d()),
        new Pose2d(intom(593.68),intom(9.68),new Rotation2d(Math.toRadians(120.0))), //1
        new Pose2d(intom(637.21),intom(34.79),new Rotation2d(Math.toRadians(120.0))), //2
        new Pose2d(intom(652.73),intom(196.17),new Rotation2d(Math.toRadians(180.0))), //3
        new Pose2d(intom(652.73),intom(218.42),new Rotation2d(Math.toRadians(180.0))), //4
        new Pose2d(intom(578.77),intom(323),new Rotation2d(Math.toRadians(270.0))), //5
        new Pose2d(intom(72.5),intom(323),new Rotation2d(Math.toRadians(270.0))), //6
        new Pose2d(intom(-1.5),intom(218.42),new Rotation2d(Math.toRadians(0.0))), //7
        new Pose2d(intom(-1.5),intom(196.17),new Rotation2d(Math.toRadians(0.0))), //8
        new Pose2d(intom(14.02),intom(34.79),new Rotation2d(Math.toRadians(60.0))), //9
        new Pose2d(intom(57.54),intom(9.68),new Rotation2d(Math.toRadians(60.0))), //10
        new Pose2d(intom(468.69),intom(146.19),new Rotation2d(Math.toRadians(300.0))), //11
        new Pose2d(intom(468.69),intom(177.1),new Rotation2d(Math.toRadians(60.0))), //12
        new Pose2d(intom(441.74),intom(161.62),new Rotation2d(Math.toRadians(180.0))), //13
        new Pose2d(intom(209.48),intom(161.62),new Rotation2d(Math.toRadians(0.0))), //14
        new Pose2d(intom(182.73),intom(177.1),new Rotation2d(Math.toRadians(120.0))), //15
        new Pose2d(intom(182.73),intom(146.19),new Rotation2d(Math.toRadians(240.0))), //16
    };
    /**
     * Map of cameras relative to robot
     */
    static Pose2d CameraPos[] = {
        new Pose2d(7.5, -1.25, new Rotation2d(Math.toRadians(0.0))),        // #1
        new Pose2d(0.0, 0.0, new Rotation2d(Math.toRadians(-20.0))),      // #4
    };

    /**
     * Calculates the robots position on the field using an AprilTag detection and the camera number
     * @param detection apriltag detection from camera
     * @param camera number of camera (as per CameraPos[] order)
     * @return
     */
    public static Pose2d CalculateRobotFieldPose(double[] detection, int camera){
        // AprilTag detection entry list definition
        // double[0] = ApriTag ID
        // double[1] = timestamp in seconds
        // double[2] = x in metres
        // double[3] = y in metres
        // double[4] = z in metres
        // double[5] = yaw in radians
        // double[6] = pitch in radians
        // double[7] = roll in radians
        // double[8] = range in metres
        // double[9] = bearing in radians
        int TagId = (int)(detection[0]+0.1);
        if (TagId<1){
            TagId=1;
        }
        System.out.print(TagId);

        // calculate and return current robot field Pose given AprilTag detection data
        // AprilTag number - id of tag detected
        // AprilTagX, AprilTagY, AprilTagYaw - x, y and yaw(deg) to apriltag in camera coordinates
        // AprilTagBearing, AprilTagRange - bearing(deg) and range to apriltag
        // Step #1: Determine robot position and rotation in Camera [coordinate] Frame (CF)
        Translation2d RobotPosCF = new Translation2d(-CameraPos[camera].getX(),
                                            -CameraPos[camera].getY());
        Rotation2d RobotAngleCF = new Rotation2d (-CameraPos[camera].getRotation().getRadians());
        // AprilTag to robot camera in CF
        Translation2d ATtoRobotCameraCF = new Translation2d(-detection[8],0.0).rotateBy(new Rotation2d (Math.toRadians(detection[9])));
        // AprilTag to robot center in CF
        Translation2d ATtoRobotCenterCF = ATtoRobotCameraCF.plus(RobotPosCF);
        // Step #2: Robot position in AprilTag [coordinate] Frame (AF)
        Translation2d ATtoRobotCenterAF = ATtoRobotCenterCF.rotateBy(new Rotation2d(Math.toRadians(-detection[5])));
        Rotation2d RobotAngleAF =RobotAngleCF.rotateBy(new Rotation2d (Math.toRadians(-detection[5])));
        // Step #3: robot position in Field [coordinate] Frame (FF)
        Translation2d ATtoRobotCenterFF = (ATtoRobotCenterAF.rotateBy(new Rotation2d (Math.toRadians(180.0-AprilTags[TagId].getRotation().getDegrees()))))
                        .plus(AprilTags[TagId].getTranslation());
        Rotation2d RobotAngleFF = RobotAngleAF.rotateBy(new Rotation2d (Math.toRadians(180.0-AprilTags[TagId].getRotation().getDegrees())));
        // return robot pose2d (in field frame)
        return new Pose2d(ATtoRobotCenterFF, RobotAngleFF);
        //return new Pose2d();

    }
}
