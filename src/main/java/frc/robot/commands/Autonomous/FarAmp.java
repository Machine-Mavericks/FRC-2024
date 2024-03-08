// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Autonomous;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.RobotContainer;
import frc.robot.commands.DriveToRelativePose;
import frc.robot.commands.SetGyroUsingAprilTag;
import frc.robot.commands.SemiAutonomous.AimThenShootSpeaker;
import frc.robot.subsystems.CassetteEffector;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class FarAmp extends SequentialCommandGroup {
  /** Creates a new FarAmp. */
  public FarAmp() {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(

        // set gyro angle from odometry
      new SetGyroUsingAprilTag(),
    
    // drive away from speaker
    new DriveToRelativePose(new Pose2d(-1.246886, 0.8382, new Rotation2d(0)), 
                            0.5, // speed
                            0.1, // rotational speed(unit?)
                            5.0),

    new AimThenShootSpeaker(), //shoot Speaker 

      new InstantCommand(() -> RobotContainer.cassetteshooter.stopShooter()),
      new InstantCommand(() -> RobotContainer.cassetteangle.setAngle(CassetteEffector.NEUTRAL_ANGLE)),
    
      new DriveToRelativePose(new Pose2d(0, 0, new Rotation2d(20)), // orienting to zero 
                            0.5, // speed
                            0.1, // rotational speed(unit?)
                            5.0)
          
  //     new DriveToRelativePose(new Pose2d(-3.28016,1.499743, new Rotation2d(0)), // moving to pick up notes 
  //                           0.5, // speed
  //                           0.1, // rotational speed(unit?)
  //                           5.0),

  //   new DriveToRelativePose(new Pose2d(0, 0, new Rotation2d(-30)), // orianting to notes 
  //                           0.5, // speed
  //                           0.1, // rotational speed(unit?)
  //                           5.0),
   
  //   new SteerToNote(true, 3.0), // pick up note 

  //   new DriveToRelativePose(new Pose2d(0, 0, new Rotation2d(30)),// turning back to straght 
  //                           0.5, // speed
  //                           0.1, // rotational speed(unit?)
  //                           5.0),
   

  //   new DriveToRelativePose(new Pose2d(3.28016,-2.337943, new Rotation2d(0)), // going back to shoot 
  //                           0.5, // speed
  //                           0.1, // rotational speed(unit?)
  //                           5.0),

  //   new AimThenShootSpeaker(), // shoot 
      
  //     new InstantCommand(() -> RobotContainer.cassetteshooter.stopShooter()),
  //     new InstantCommand(() -> RobotContainer.cassetteangle.setAngle(CassetteEffector.NEUTRAL_ANGLE)),
  //     new CleanupShot(),

  //    new DriveToRelativePose(new Pose2d(0, 0, new Rotation2d(20)), // orienting to zero 
  //                           0.5, // speed
  //                           0.1, // rotational speed(unit?)
  //                           5.0),
 
  //   new DriveToRelativePose(new Pose2d(-3.28016,2.337943, new Rotation2d(0)), // going to get note  
  //                           0.5, // speed
  //                           0.1, // rotational speed(unit?)
  //                           5.0),
   
  //   new DriveToRelativePose(new Pose2d(0, 0, new Rotation2d(30)), // turning to note 
  //                           0.5, // speed
  //                           0.1, // rotational speed(unit?)
  //                           5.0),
    
  //   new SteerToNote(true, 3.0),

  //   new DriveToRelativePose(new Pose2d(0, 0, new Rotation2d(-30)), // going back to straght 
  //                           0.5, // speed
  //                           0.1, // rotational speed(unit?)
  //                           5.0),

  //   new DriveToRelativePose(new Pose2d(3.28016,-2.337943, new Rotation2d(0)), // going back to shoot 
  //                           0.5, // speed
  //                           0.1, // rotational speed(unit?)
  //                           5.0),

  //   new AimThenShootSpeaker(), // shoot 
  //     new InstantCommand(() -> RobotContainer.cassetteshooter.stopShooter()),
  //     new InstantCommand(() -> RobotContainer.cassetteangle.setAngle(CassetteEffector.NEUTRAL_ANGLE)),
  //     new CleanupShot(),

  //    new DriveToRelativePose(new Pose2d(0, 0, new Rotation2d(20)), // orienting to zero 
  //                           0.5, // speed
  //                           0.1, // rotational speed(unit?)
  //                           5.0)
                     
    );
  }
}
