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
import frc.robot.commands.Drive.SteerToNote;
import frc.robot.commands.SemiAutonomous.AimThenShootSpeaker;
import frc.robot.commands.SemiAutonomous.CleanupShot;
import frc.robot.util.AutoFunctions;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class ThreeNoteStageCenter extends SequentialCommandGroup {
  /** Creates a new ThreeNoteStageCenter. */
  public ThreeNoteStageCenter() {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
    //zero gyro
    new InstantCommand (()-> RobotContainer.gyro.resetGyroReverse()),
    //shoot preload 
    // new AutoDriveToFieldPose(AutoFunctions.redVsBlue(new Pose2d(1.5,5.5, new Rotation2d(Math.toRadians(180.0)))), 0.5, 0.5, 2),
    new AimThenShootSpeaker(),
    new CleanupShot(),
    //move to note 1 
    
    //pick up note 1
    new SteerToNote(true, 2.0, 0.2),
    // shoot note 1
    new AimThenShootSpeaker(),
    new CleanupShot(),
    //drive under the stage to note 5
    //new AutoDriveToFieldPose(AutoFunctions.redVsBlue(new Pose2d(4.15,4.9, new Rotation2d(Math.toRadians(180.0)))), 0.5, 0.18, 0.5, 3),
    new AutoDriveToFieldPose(AutoFunctions.redVsBlue(new Pose2d(6.3,4.3, new Rotation2d(Math.toRadians(180.0)))), 0.8, 0.21,0.5, 3),
    //pick up note 5 
    new SteerToNote(true, 2.0, 0.2),
    // drive to shoot position 
    //new AutoDriveToFieldPose(AutoFunctions.redVsBlue(new Pose2d(7.3,4.1, new Rotation2d(Math.toRadians(180.0)))), 0.5, 0.5, 3),
    // new AutoDriveToFieldPose(AutoFunctions.redVsBlue(new Pose2d(4.4,4.1, new Rotation2d(Math.toRadians(180.0)))), 0.5, 0.5, 3),
    new AutoDriveToFieldPose(AutoFunctions.redVsBlue(new Pose2d(4.25,5.2, new Rotation2d(Math.toRadians(180.0)))), 0.8,0.06, 0.5, 3),
    // shoot 
    new WaitCommand(1.1),
    new AimThenShootSpeaker(),
    new CleanupShot()
    
    );
  }
}
