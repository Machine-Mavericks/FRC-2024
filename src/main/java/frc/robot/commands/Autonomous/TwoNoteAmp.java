// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Autonomous;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.RobotContainer;
import frc.robot.commands.Drive.AutoDriveToFieldPose;
import frc.robot.commands.Drive.AutoDriveToFieldPoseSimple;
import frc.robot.commands.Drive.SteerToNote;
import frc.robot.commands.SemiAutonomous.AimThenShootSpeaker;
import frc.robot.commands.SemiAutonomous.CleanupShot;
import frc.robot.subsystems.Drivetrain;
import frc.robot.util.AutoFunctions;


public class TwoNoteAmp extends SequentialCommandGroup {
  
  /** Creates a new TwoNoteAmp. */
  public TwoNoteAmp() {
    
    addCommands(

    // set gyro angle from odometry
    //new SetGyroUsingAprilTag(),

    // drive away from speaker
    // new DriveToRelativePose(new Pose2d(-0.7, 0.0, new Rotation2d(0)),
    //                         0.5, // speed
    //                         0.1, // rotational speed(unit?)
    //                         5.0),

   new InstantCommand (()-> RobotContainer.gyro.resetGyroReverse()),

    new AutoDriveToFieldPoseSimple(AutoFunctions.redVsBlue(new Pose2d(2.0 ,7.0, new Rotation2d(Math.toRadians(180.0)))),  0.5, 0.5, 3),

    //new AutoDriveToPose(AutoFunctions.redVsBlue(new Pose2d(AutoFunctions.intom(75),AutoFunctions.intom(159.5+57+57),new Rotation2d(Math.toRadians(180)))),  0.5, 0.5, 3),
    //new AutoDriveToPose(AutoFunctions.redVsBlue(new Pose2d(2.0 ,7.0, new Rotation2d())),  0.5, 0.5, 3),

    //new AutoDriveToPose(new Pose2d(14.5,7.0,new Rotation2d(180)),  0.5, 0.5, 3),

    new AimThenShootSpeaker(),

    //new AutoDriveToPose(AutoFunctions.redVsBlue(new Pose2d(AutoFunctions.intom(75),AutoFunctions.intom(159.5+57+57),new Rotation2d(Math.toRadians(180)))),  0.5, 0.5, 3),

    new CleanupShot(),

    new AutoDriveToFieldPose(AutoFunctions.redVsBlue(new Pose2d(2.0 ,7.0, new Rotation2d(Math.toRadians(180.0)))),  0.5, 0.5, 3),

    new SteerToNote(true, 2.0, 0.2),

    new AimThenShootSpeaker(),

    new CleanupShot()

    //new AutoDriveToFieldPose(AutoFunctions.redVsBlue(new Pose2d(2.0 ,7.0, new Rotation2d(Math.toRadians(180.0)))),  0.5, 0.5, 3),
    );
  }
}