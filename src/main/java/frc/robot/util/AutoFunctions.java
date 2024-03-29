// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.util;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;

/** Add your docs here. */
public class AutoFunctions {
      /**
     * Convert inches to meters
     * @param in
     * @return meters
     */
    public static double intom(double in){
        return in/39.37;
    }

    /**
     * Approximate map of notes relative to field
     */
    public static Pose2d NotesAtStart[] = {
        new Pose2d(intom(114-10),intom(159.5),new Rotation2d(Math.toRadians(180))), //0
        new Pose2d(intom(114-10),intom(159.5+57),new Rotation2d(Math.toRadians(180))), //1
        new Pose2d(intom(144-10),intom(159.5+57+57),new Rotation2d(Math.toRadians(180))), //2
        new Pose2d(intom(324.5-10),intom(293.36),new Rotation2d(Math.toRadians(180))), //3
        new Pose2d(intom(324.5-10),intom(227.36),new Rotation2d(Math.toRadians(180))), //4
        new Pose2d(intom(324.5-10),intom(161.36),new Rotation2d(Math.toRadians(180))), //5
        new Pose2d(intom(324.5-10),intom(95.36),new Rotation2d(Math.toRadians(180))), //6
        new Pose2d(intom(324.5-10),intom(29.36),new Rotation2d(Math.toRadians(180))), //7
    };

    /**
     * Takes field relative coordinates for blue alliance and returns alliance relative coordinates
     * @param x  
     * @param y 
     * @param angle degrees
     * @return
     */
    public static Pose2d redVsBlue(double x, double y, double angle) {
        if (DriverStation.getAlliance().isPresent()) {
            if (DriverStation.getAlliance().get() == Alliance.Red){
                x = 16.4846 - x;
                angle = Math.toRadians(180-angle);
            }
        }
        
        return new Pose2d(x,y,new Rotation2d(angle));
    }


    public static Pose2d redVsBlue(Pose2d pose) {
        if (DriverStation.getAlliance().isPresent()) {
            if (DriverStation.getAlliance().get() == Alliance.Red){
                double x = 16.4846 - pose.getX();
                double y = pose.getY();
                Rotation2d angle = new Rotation2d(Math.toRadians(180)).rotateBy(pose.getRotation());
                return new Pose2d(x,y,angle);
            }
        }
        return pose;
    }
}
