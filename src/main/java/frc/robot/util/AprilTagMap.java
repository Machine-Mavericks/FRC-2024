// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.util;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;

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
        new Pose2d(0.0, -2.0, new Rotation2d(Math.toRadians(90.0))),      // #2
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

        // create internal AprilTag Pose2d object
        Pose2d tagPose = AprilTags[(int)(detection[0]+0.1)-1];

        // IDs 12, 11, 15 and 16 may need work
        double xChange=0; // the change in the xdirection of the field
        double yChange=0; // the change in the ydirection of the field
        double rotation=-Math.toDegrees(detection[5]); 
        if (detection[0]==1||detection[0]==2||detection[0]==9||detection[0]==10||(detection[0]==15&&rotation<0)||(detection[0]==12&&rotation>0)) { // camera looking towards bottom of field
            xChange = -detection[2]; // the inverse sign of the x that the camera feeds you
            yChange = detection[3]; // the y that the camera feeds you
        } else if (detection[0]==7||detection[0]==8||detection[0]==14||(detection[0]==11&&rotation>0)||(detection[0]==12&&rotation<0)) { // camera looking towards left of field
            xChange = detection[3]; // the y that the camera feeds you
            yChange = detection[2]; // the x that the camera feeds you
        } else if (detection[0]==6||detection[0]==5||(detection[0]==16&&rotation>0)||(detection[0]==11&&rotation<0)) { // camera looking towards top of field
            xChange = detection[2]; // the x that the camera feeds you
            yChange = -detection[3]; // the inverse sign of the y that the camera feeds you
        } else if (detection[0]==4||detection[0]==3||detection[0]==13||(detection[0]==16&&rotation<0)||(detection[0]==15&&rotation>0)) { // camera looking towards right of field
            xChange = -detection[3]; // the inverse sign of the y that the camera feeds you
            yChange = -detection[2]; // the inverse sign of the x that the camera feeds you
        }

        // alter tagPose by the x,y and yaw provided by the camera to get the camera's position on the field
        Pose2d cameraPose = new Pose2d(tagPose.getX()+xChange,tagPose.getY()+yChange,tagPose.getRotation()).rotateBy(new Rotation2d(rotation+180));

        // alter cameraPose by the x,y and rotation of the camera on the robot to get the robot's position on the field
        Pose2d robotPose = new Pose2d(cameraPose.getX()-CameraPos[camera].getX(), cameraPose.getY()-CameraPos[camera].getY(), cameraPose.getRotation()).rotateBy(CameraPos[camera].getRotation());

        return robotPose;
    }
}
