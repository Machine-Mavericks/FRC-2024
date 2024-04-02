// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Autonomous;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.RobotContainer;
import frc.robot.commands.Drive.AutoDriveToFieldPose;
import frc.robot.commands.Drive.SteerToNote;
import frc.robot.commands.SemiAutonomous.AimThenShootSpeaker;
import frc.robot.commands.SemiAutonomous.CleanupShot;
import frc.robot.util.AutoFunctions;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class ThreeNoteCenter extends SequentialCommandGroup {
  /** Creates a new ThreeNoteCenter. */
  public ThreeNoteCenter() {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
    new InstantCommand (()-> RobotContainer.gyro.resetGyroReverse()),
    
    new AimThenShootSpeaker(),
    new CleanupShot(),
    
    new SteerToNote(true, 2.0, 0.2),
    
    
    new AimThenShootSpeaker(),
    new CleanupShot(),

    new AutoDriveToFieldPose(AutoFunctions.redVsBlue(new Pose2d(5.35,6.75, new Rotation2d(Math.toRadians(180.0)))), 0.5, 0.5, 3),
    new AutoDriveToFieldPose(AutoFunctions.redVsBlue(new Pose2d(7.0,5.65, new Rotation2d(Math.toRadians(180.0)))), 0.5, 0.5, 3),      
    
    new SteerToNote(true, 2.0, 0.2),
    
    new AutoDriveToFieldPose(AutoFunctions.redVsBlue(new Pose2d(4.35,6.75, new Rotation2d(Math.toRadians(180.0)))), 0.5, 0.5, 3),    
    
    new AimThenShootSpeaker(),
    new CleanupShot()

    );
  }
}
