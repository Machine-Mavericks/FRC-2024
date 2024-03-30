// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Autonomous;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.RobotContainer;
import frc.robot.commands.Drive.AutoDriveOverLine;
import frc.robot.commands.SemiAutonomous.AimThenShootSpeaker;
import frc.robot.commands.SemiAutonomous.CleanupShot;


public class OneNoteAnywhere extends SequentialCommandGroup {
  
  /** Creates a new OneNoteAnywhere Auto. */
  public OneNoteAnywhere() {
    
    addCommands(

    new InstantCommand (()-> RobotContainer.gyro.resetGyroReverse()),
    
    // new TurnRobot(360, true, 3),
    // set gyro angle from odometry
    //new SetGyroUsingAprilTag(),

    // drive away from speaker
    // new DriveToRelativePose(new Pose2d(-1, 0.0, new Rotation2d(0)),
    //                         0.5, // speed
    //                         0.1, // rotational speed(unit?)
    //                         5.0),
    //new AutoDriveToPose(AutoFunctions.redVsBlue(new Pose2d()),  0.5, 0.5, 3),

    new AimThenShootSpeaker(),

    new CleanupShot(),

    new AutoDriveOverLine(0.5, 0.5, 2.0)


        // drive away from speaker
    // new DriveToRelativePose(new Pose2d(-1, 0.0, new Rotation2d(0)),
    //                         0.5, // speed
    //                         0.1, // rotational speed(unit?)
    //                         5.0)
    // new AutoDriveToPose(AutoFunctions.redVsBlue(new Pose2d(3,2.1, new Rotation2d())), 0.5, 0.5, 3),



    );
  }
}
