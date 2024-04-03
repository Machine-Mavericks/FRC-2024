// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Autonomous;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.RobotContainer;
import frc.robot.commands.Drive.AutoDriveToFieldPose;
import frc.robot.commands.Drive.AutoDriveToFieldPoseSimple;
import frc.robot.commands.Drive.SteerToNote;
import frc.robot.commands.Drive.TurnRobot;
import frc.robot.commands.SemiAutonomous.AimThenShootSpeaker;
import frc.robot.commands.SemiAutonomous.CleanupShot;
import frc.robot.util.AutoFunctions;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class FourNoteAmp extends SequentialCommandGroup {
  /** Creates a new FourNoteAmp. */
  public FourNoteAmp() {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
    
    //new AutoDriveToFieldPoseSimple(AutoFunctions.redVsBlue(new Pose2d(2.0,7.0,new Rotation2d(Math.toRadians(180.0)))), 0.75, 0.5, 3),    
    
    new AimThenShootSpeaker(),
    //new InstantCommand (()-> RobotContainer.gyro.resetGyroReverse()),
    //new AutoDriveToFieldPoseSimple(AutoFunctions.redVsBlue(new Pose2d(2.0,7.0,new Rotation2d(Math.toRadians(180.0)))), 0.75, 0.5, 3),    
 
    //new CleanupShot(),

    //new AutoDriveToFieldPoseSimple(AutoFunctions.redVsBlue(new Pose2d(2.0,7.0,new Rotation2d(Math.toRadians(180.0)))), 0.75, 0.5, 3),    
    new SteerToNote(true, 3.0, 0.2),
    //new WaitCommand(0.20),
    
    new AutoDriveToFieldPoseSimple(AutoFunctions.redVsBlue(new Pose2d(2.0,5.5,new Rotation2d(Math.toRadians(180.0)))), 0.75, 0.5, 3),    
    new AimThenShootSpeaker(),
    //new CleanupShot(),

    new SteerToNote(true, 2.0, 0.2),
    //new WaitCommand(0.20),
    new AimThenShootSpeaker(),
    //new CleanupShot(),

    new AutoDriveToFieldPoseSimple(AutoFunctions.redVsBlue(new Pose2d(2.0,4.6,new Rotation2d(Math.toRadians(150.0)))), 0.75, 0.5, 3),    
    new SteerToNote(true, 2.0, 0.2),
    new AimThenShootSpeaker(),
    new CleanupShot()
    );
  }
}
