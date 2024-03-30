// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Autonomous;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.RobotContainer;
import frc.robot.commands.Drive.SteerToNote;
import frc.robot.commands.SemiAutonomous.AimThenShootSpeaker;
import frc.robot.commands.SemiAutonomous.CleanupShot;


public class ThreeNoteStage extends SequentialCommandGroup {
  
  /** Creates a new SixNoteAuto. */
  public ThreeNoteStage() {
    
    addCommands(
    
    // shoot preload
    
    //new AutoDriveToPose(AutoFunctions.redVsBlue(new Pose2d(  2.0,4.0, new Rotation2d())),  0.5, 0.5, 3),
    new InstantCommand (()-> RobotContainer.gyro.resetGyroReverse()),
    new AimThenShootSpeaker(),

    new CleanupShot(),

    //reoriant to zero angle 

    //new AutoDriveToPose(AutoFunctions.redVsBlue(new Pose2d(2.0,4.0, new Rotation2d(180))),  0.5, 0.5, 3),
    
    //drive arround to notes 
    //new AutoDriveToPose(AutoFunctions.redVsBlue(new Pose2d(  3.0,2.1, new Rotation2d())),  0.5, 0.5, 3),

    //new AutoDriveToPose(AutoFunctions.redVsBlue(new Pose2d(5.2,1.7, new Rotation2d())),  0.5, 0.5, 3),
    // drive to note 7
    //new AutoDriveToPose(AutoFunctions.redVsBlue(new Pose2d(  0.7,7.5, new Rotation2d())), 0.5, 0.5, 3),
    // pick up note
    new SteerToNote(true, 2.0, 0.2),

    //reoriant to zero angle 
    //new AutoDriveToPose(AutoFunctions.redVsBlue(new Pose2d(2.1,4, new Rotation2d(180))),  0.5, 0.5, 3),
     
    // drive to shoot 
    //new AutoDriveToPose(AutoFunctions.redVsBlue(new Pose2d(5.2,1.7, new Rotation2d())),  0.5, 0.5, 3),

    //new AutoDriveToPose(AutoFunctions.redVsBlue(new Pose2d(  3,2.1, new Rotation2d())),  0.5, 0.5, 3),

    //new AutoDriveToPose(AutoFunctions.redVsBlue(new Pose2d(2.0,4.0, new Rotation2d())),  0.5, 0.5, 3),

    new AimThenShootSpeaker(),

    new CleanupShot(),

    //reoriant to zero angle 
    //new AutoDriveToPose(AutoFunctions.redVsBlue(new Pose2d(2.1,4, new Rotation2d(180))),  0.5, 0.5, 3),

    //drive arround to notes 
    //new AutoDriveToPose(AutoFunctions.redVsBlue(new Pose2d(  3,2.1, new Rotation2d())),  0.5, 0.5, 3),

    //new AutoDriveToPose(AutoFunctions.redVsBlue(new Pose2d(5.2,1.7, new Rotation2d())),  0.5, 0.5, 3),

    // drive to note 6
    //new AutoDriveToPose(AutoFunctions.redVsBlue(new Pose2d(  2.45,7.5, new Rotation2d())),  0.5, 0.5, 3),
    
    new SteerToNote(true, 2.0, 0.2),

    //reoriant to zero angle 
    //new AutoDriveToPose(AutoFunctions.redVsBlue(new Pose2d(2.0,4.0, new Rotation2d(180))),  0.5, 0.5, 3),


    // drive to shoot 
    //new AutoDriveToPose(AutoFunctions.redVsBlue(new Pose2d(5.2,1.7, new Rotation2d())),  0.5, 0.5, 3),

    //new AutoDriveToPose(AutoFunctions.redVsBlue(new Pose2d(3,2.1, new Rotation2d())),  0.5, 0.5, 3),

    //new AutoDriveToPose(AutoFunctions.redVsBlue(new Pose2d(2.0,4.0, new Rotation2d())),  0.5, 0.5, 3),

    new AimThenShootSpeaker(),

    new CleanupShot(),

    //reoriant to zero angle 
    //new AutoDriveToPose(AutoFunctions.redVsBlue(new Pose2d(2.0,4.0, new Rotation2d(180))),  0.5, 0.5, 3),
    
    //drive arround to notes 
    //new AutoDriveToPose(AutoFunctions.redVsBlue(new Pose2d(  3,2.1, new Rotation2d())),  0.5, 0.5, 3),

    //new AutoDriveToPose(AutoFunctions.redVsBlue(new Pose2d(5.2,1.7, new Rotation2d())),  0.5, 0.5, 3),

    // drive to note 6 position 
    //new AutoDriveToPose(AutoFunctions.redVsBlue(new Pose2d(  2.45,7.5, new Rotation2d())),  0.5, 0.5, 3),

    // drive to note 5
    //new AutoDriveToPose(AutoFunctions.redVsBlue(new Pose2d(  4.1,7.5, new Rotation2d())),  0.5, 0.5, 3),
    
    new SteerToNote(true, 2.0, 0.2),

    //reoriant to zero angle 
    //new AutoDriveToPose(AutoFunctions.redVsBlue(new Pose2d(4.1,7.5, new Rotation2d(180))),  0.5, 0.5, 3),

     // drive to note 6 position 
    //new AutoDriveToPose(AutoFunctions.redVsBlue(new Pose2d(  2.45,7.5, new Rotation2d())),  0.5, 0.5, 3),
    //drive to shoot 
    //new AutoDriveToPose(AutoFunctions.redVsBlue(new Pose2d(5.2,1.7, new Rotation2d())),  0.5, 0.5, 3),
    
    //new AutoDriveToPose(AutoFunctions.redVsBlue(new Pose2d(  3,2.1, new Rotation2d())),  0.5, 0.5, 3),
    
    //new AutoDriveToPose(AutoFunctions.redVsBlue(new Pose2d(2.0,4.0, new Rotation2d())),  0.5, 0.5, 3),

    new AimThenShootSpeaker(),

    new CleanupShot(),

    //reoriant to zero angle 
    //new AutoDriveToPose(AutoFunctions.redVsBlue(new Pose2d(2.0,4.0, new Rotation2d(180))),  0.5, 0.5, 3),
     
    //drive arround to notes 
    //new AutoDriveToPose(AutoFunctions.redVsBlue(new Pose2d(  3,2.1, new Rotation2d())),  0.5, 0.5, 3),

    //new AutoDriveToPose(AutoFunctions.redVsBlue(new Pose2d(5.2,1.7, new Rotation2d())),  0.5, 0.5, 3),

    // drive to note 6 position 
    //new AutoDriveToPose(AutoFunctions.redVsBlue(new Pose2d(  2.45,7.5, new Rotation2d())),  0.5, 0.5, 3),

    // drive to note 5 position 
    //new AutoDriveToPose(AutoFunctions.redVsBlue(new Pose2d(  4.1,7.5, new Rotation2d())),  0.5, 0.5, 3),
    
    // drive to note 4
    //new AutoDriveToPose(AutoFunctions.redVsBlue(new Pose2d(  5.8,7.5, new Rotation2d())),  0.5, 0.5, 3),
    
    new SteerToNote(true, 2.0, 0.2),

    //reoriant to zero angle 
    //new AutoDriveToPose(AutoFunctions.redVsBlue(new Pose2d(5.8,7.5, new Rotation2d(180))),  0.5, 0.5, 3),

    // drive to note 5 position 
    //new AutoDriveToPose(AutoFunctions.redVsBlue(new Pose2d(  4.1,7.5, new Rotation2d())), 0.5, 0.5, 3),

    // drive to note 6 position 
    //new AutoDriveToPose(AutoFunctions.redVsBlue(new Pose2d(  2.45,7.5, new Rotation2d())),  0.5, 0.5, 3),
   
    //drive to shoot 
    //new AutoDriveToPose(AutoFunctions.redVsBlue(new Pose2d(5.2,1.7, new Rotation2d())),  0.5, 0.5, 3),
    
    //new AutoDriveToPose(AutoFunctions.redVsBlue(new Pose2d(  3,2.1, new Rotation2d())),  0.5, 0.5, 3),
    
    //new AutoDriveToPose(AutoFunctions.redVsBlue(new Pose2d(2.0,4.0, new Rotation2d())),  0.5, 0.5, 3),

    new AimThenShootSpeaker(),

    new CleanupShot(),

    //reoriant to zero angle 
    //new AutoDriveToPose(AutoFunctions.redVsBlue(new Pose2d(2.0,4.0, new Rotation2d(180))),  0.5, 0.5, 3),
    
    //reoriant to zero angle 
    //new AutoDriveToPose(AutoFunctions.redVsBlue(new Pose2d(2.0,4.0, new Rotation2d(180))), 0.5, 0.5, 3),
     
    //drive arround to notes 
    //new AutoDriveToPose(AutoFunctions.redVsBlue(new Pose2d(  3,2.1, new Rotation2d())),  0.5, 0.5, 3),

    //new AutoDriveToPose(AutoFunctions.redVsBlue(new Pose2d(5.2,1.7, new Rotation2d())),  0.5, 0.5, 3),

    // drive to note 6 position 
    //new AutoDriveToPose(AutoFunctions.redVsBlue(new Pose2d(  2.45,7.5, new Rotation2d())),  0.5, 0.5, 3),

    // drive to note 5 position 
    //new AutoDriveToPose(AutoFunctions.redVsBlue(new Pose2d(  4.1,7.5, new Rotation2d())),  0.5, 0.5, 3),
    
    // drive to note 4 position 
    //new AutoDriveToPose(AutoFunctions.redVsBlue(new Pose2d(  5.8,7.5, new Rotation2d())), 0.5, 0.5, 3),
    
    // drive to note 3 
    //new AutoDriveToPose(AutoFunctions.redVsBlue(new Pose2d(  7.3,7.5, new Rotation2d())), 0.5, 0.5, 3),
    
    //new SteerToNote(true, 2.0, 0.2),

    //reoriant to zero angle 
    //new AutoDriveToPose(AutoFunctions.redVsBlue(new Pose2d(5.8,7.5, new Rotation2d(180))),  0.5, 0.5, 3),

    // drive to note 4 position 
    //new AutoDriveToPose(AutoFunctions.redVsBlue(new Pose2d(  5.8,7.5, new Rotation2d())),  0.5, 0.5, 3),

    // drive to note 5 position 
    //new AutoDriveToPose(AutoFunctions.redVsBlue(new Pose2d(  4.1,7.5, new Rotation2d())), 0.5, 0.5, 3),

    // drive to note 6 position 
    //new AutoDriveToPose(AutoFunctions.redVsBlue(new Pose2d(  2.45,7.5, new Rotation2d())),  0.5, 0.5, 3),
   
    //drive to shoot 
    //new AutoDriveToPose(AutoFunctions.redVsBlue(new Pose2d(5.2,1.7, new Rotation2d())),  0.5, 0.5, 3),
    
    //new AutoDriveToPose(AutoFunctions.redVsBlue(new Pose2d(  3,2.1, new Rotation2d())),  0.5, 0.5, 3),
    
    //new AutoDriveToPose(AutoFunctions.redVsBlue(new Pose2d(2.0,4.0, new Rotation2d())),  0.5, 0.5, 3),

    new AimThenShootSpeaker(),

    new CleanupShot()

    );
  }
}
