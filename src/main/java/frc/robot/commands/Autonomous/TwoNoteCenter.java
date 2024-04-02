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
import frc.robot.util.AutoFunctions;


public class TwoNoteCenter extends SequentialCommandGroup {
  /** Creates a new TwoNoteStage. */
  public TwoNoteCenter() {
    
    addCommands(
     //new InstantCommand (()-> RobotContainer.gyro.resetGyroReverse()),
     new AimThenShootSpeaker(),
     new CleanupShot(),
     new AutoDriveToFieldPose(AutoFunctions.redVsBlue(new Pose2d(2.0 ,5.5, new Rotation2d(Math.toRadians(180.0)))),  0.5, 0.5, 3), 
     new SteerToNote(true, 2.0, 0.2),
     new AimThenShootSpeaker(),
     new CleanupShot()
    );
  }
}
