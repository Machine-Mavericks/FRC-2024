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
import frc.robot.commands.Drive.SteerToNote;
import frc.robot.commands.SemiAutonomous.AimThenShootSpeaker;
import frc.robot.commands.SemiAutonomous.CleanupShot;
import frc.robot.subsystems.Drivetrain;
import frc.robot.util.AutoFunctions;


public class FourNoteSource extends SequentialCommandGroup {
  
  /** Creates a new FourNoteSource Auto. */
  public FourNoteSource() {
    
    //Move Forward 2 meeters
    //AimThenShootSpeaker 
    //SteerToNote 4
    //Move to previuse shoot position 
    //AimThenShootSpeaker 
    //SteerToNote 5 (preload or shoot based on time )
    //Move to previuse shoot position 
    //AimThenShootSpeaker 
    
   

    addCommands(
    
    // start robot with intake away from wall (use reverse reset command)
    new InstantCommand (()-> RobotContainer.gyro.resetGyroReverse()),

    new AutoDriveToFieldPose(AutoFunctions.redVsBlue(new Pose2d(2.0,4.0, new Rotation2d(Math.toRadians(180.0)))), 0.5*Drivetrain.MAX_VELOCITY_METERS_PER_SECOND, 0.5*Drivetrain.MAX_ANGULAR_VELOCITY_RADIANS_PER_SECOND, 3),    
    
    new AimThenShootSpeaker(),
    new CleanupShot(),

    //reoriant to zero angle 
    new AutoDriveToFieldPose(AutoFunctions.redVsBlue(new Pose2d(2.0,4.0, new Rotation2d(Math.toRadians(180.0)))), 0.5*Drivetrain.MAX_VELOCITY_METERS_PER_SECOND, 0.5*Drivetrain.MAX_ANGULAR_VELOCITY_RADIANS_PER_SECOND, 3),
    

    // pick up first note
    new SteerToNote(true, 2.0, 0.2),
    
    // move away from post
    new AutoDriveToFieldPose(AutoFunctions.redVsBlue(new Pose2d(  1.5,3.5, new Rotation2d(Math.toRadians(180.0)))), 0.5*Drivetrain.MAX_VELOCITY_METERS_PER_SECOND, 0.5*Drivetrain.MAX_ANGULAR_VELOCITY_RADIANS_PER_SECOND, 3),

    new AimThenShootSpeaker(),
    new CleanupShot(),


    //drive arround to note 7
    new AutoDriveToFieldPose(AutoFunctions.redVsBlue(new Pose2d(5.2,1.7, new Rotation2d(Math.toRadians(180.0)))), 0.5*Drivetrain.MAX_VELOCITY_METERS_PER_SECOND, 0.5*Drivetrain.MAX_ANGULAR_VELOCITY_RADIANS_PER_SECOND, 10)  
    
    // // drive to note 7
    // new AutoDriveToPose(AutoFunctions.redVsBlue(new Pose2d(  0.7,7.5, new Rotation2d())), 0.5*Drivetrain.MAX_VELOCITY_METERS_PER_SECOND, 0.5*Drivetrain.MAX_ANGULAR_VELOCITY_RADIANS_PER_SECOND, 3),
    // // pick up note
    // new SteerToNote(true, 2.0, 0.2),

    // //reoriant to zero angle 
    // new AutoDriveToPose(AutoFunctions.redVsBlue(new Pose2d(2.1,4, new Rotation2d(180))), 0.5*Drivetrain.MAX_VELOCITY_METERS_PER_SECOND, 0.5*Drivetrain.MAX_ANGULAR_VELOCITY_RADIANS_PER_SECOND, 3),
     
    // // drive to shoot 
    // new AutoDriveToPose(AutoFunctions.redVsBlue(new Pose2d(5.2,1.7, new Rotation2d())), 0.5*Drivetrain.MAX_VELOCITY_METERS_PER_SECOND, 0.5*Drivetrain.MAX_ANGULAR_VELOCITY_RADIANS_PER_SECOND, 3),   

    // new AutoDriveToPose(AutoFunctions.redVsBlue(new Pose2d(  3,2.1, new Rotation2d())), 0.5*Drivetrain.MAX_VELOCITY_METERS_PER_SECOND, 0.5*Drivetrain.MAX_ANGULAR_VELOCITY_RADIANS_PER_SECOND, 3),    

    // new AutoDriveToPose(AutoFunctions.redVsBlue(new Pose2d(2.0,4.0, new Rotation2d())), 0.5*Drivetrain.MAX_VELOCITY_METERS_PER_SECOND, 0.5*Drivetrain.MAX_ANGULAR_VELOCITY_RADIANS_PER_SECOND, 3),   

    // new AimThenShootSpeaker(),

    // new CleanupShot(),

    // //reoriant to zero angle 
    // new AutoDriveToPose(AutoFunctions.redVsBlue(new Pose2d(2.1,4, new Rotation2d(180))), 0.5*Drivetrain.MAX_VELOCITY_METERS_PER_SECOND, 0.5*Drivetrain.MAX_ANGULAR_VELOCITY_RADIANS_PER_SECOND, 3),

    // //drive arround to notes 
    // new AutoDriveToPose(AutoFunctions.redVsBlue(new Pose2d(  3,2.1, new Rotation2d())), 0.5*Drivetrain.MAX_VELOCITY_METERS_PER_SECOND, 0.5*Drivetrain.MAX_ANGULAR_VELOCITY_RADIANS_PER_SECOND, 3),    

    // new AutoDriveToPose(AutoFunctions.redVsBlue(new Pose2d(5.2,1.7, new Rotation2d())), 0.5*Drivetrain.MAX_VELOCITY_METERS_PER_SECOND, 0.5*Drivetrain.MAX_ANGULAR_VELOCITY_RADIANS_PER_SECOND, 3),   

    // // drive to note 6
    // new AutoDriveToPose(AutoFunctions.redVsBlue(new Pose2d(  2.45,7.5, new Rotation2d())), 0.5*Drivetrain.MAX_VELOCITY_METERS_PER_SECOND, 0.5*Drivetrain.MAX_ANGULAR_VELOCITY_RADIANS_PER_SECOND, 3),
    
    // new SteerToNote(true, 2.0, 0.2),

    // //reoriant to zero angle 
    // new AutoDriveToPose(AutoFunctions.redVsBlue(new Pose2d(2.1,4, new Rotation2d(180))), 0.5*Drivetrain.MAX_VELOCITY_METERS_PER_SECOND, 0.5*Drivetrain.MAX_ANGULAR_VELOCITY_RADIANS_PER_SECOND, 3),


    // // drive to shoot 
    // new AutoDriveToPose(AutoFunctions.redVsBlue(new Pose2d(5.2,1.7, new Rotation2d())), 0.5*Drivetrain.MAX_VELOCITY_METERS_PER_SECOND, 0.5*Drivetrain.MAX_ANGULAR_VELOCITY_RADIANS_PER_SECOND, 3),   

    // new AutoDriveToPose(AutoFunctions.redVsBlue(new Pose2d(3,2.1, new Rotation2d())), 0.5*Drivetrain.MAX_VELOCITY_METERS_PER_SECOND, 0.5*Drivetrain.MAX_ANGULAR_VELOCITY_RADIANS_PER_SECOND, 3),    

    // new AutoDriveToPose(AutoFunctions.redVsBlue(new Pose2d(2,4, new Rotation2d())), 0.5*Drivetrain.MAX_VELOCITY_METERS_PER_SECOND, 0.5*Drivetrain.MAX_ANGULAR_VELOCITY_RADIANS_PER_SECOND, 3),   

    // new AimThenShootSpeaker(),

    // new CleanupShot(),

    // //reoriant to zero angle 
    // new AutoDriveToPose(AutoFunctions.redVsBlue(new Pose2d(2.1,4, new Rotation2d(180))), 0.5*Drivetrain.MAX_VELOCITY_METERS_PER_SECOND, 0.5*Drivetrain.MAX_ANGULAR_VELOCITY_RADIANS_PER_SECOND, 3)

    );
  }
}
