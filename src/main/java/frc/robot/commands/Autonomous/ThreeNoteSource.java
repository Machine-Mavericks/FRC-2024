// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Autonomous;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.RobotContainer;
import frc.robot.commands.Drive.AutoDriveToFieldPose;
import frc.robot.commands.Drive.SteerToNote;
import frc.robot.commands.Mechanism.GroundIntake;
import frc.robot.commands.Mechanism.WaitForEffectorAngle;
import frc.robot.commands.Mechanism.WaitForShooterSpinup;
import frc.robot.commands.SemiAutonomous.AimThenShootSpeaker;
import frc.robot.commands.SemiAutonomous.CleanupShot;
import frc.robot.commands.SemiAutonomous.MoveThenShoot;
import frc.robot.subsystems.CassetteIntake;
import frc.robot.subsystems.Drivetrain;
import frc.robot.util.AutoFunctions;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class ThreeNoteSource extends SequentialCommandGroup {
  /** Creates a new ThreeNoteSource. */
  public ThreeNoteSource() {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
    //reset gyro
    //new InstantCommand (()-> RobotContainer.gyro.resetGyroReverse()),
    //shoot preload
    // new AutoDriveToFieldPose(AutoFunctions.redVsBlue(new Pose2d(4.0,2.5, new Rotation2d(Math.toRadians(120.0)))), 0.75,0.5, 5.0),
    new AutoDriveToFieldPose(AutoFunctions.redVsBlue(new Pose2d(2.9,3.1, new Rotation2d(Math.toRadians(130.0)))), 0.75,0.5, 5.0),
    new AimThenShootSpeaker(), 
    
    // move to shoot position 
    new AutoDriveToFieldPose(AutoFunctions.redVsBlue(new Pose2d(7.5,0.8, new Rotation2d(Math.toRadians(180.0)))), 0.75,0.5, 5.0),
    new SteerToNote(true, 2.0, 0.3), 
    new AutoDriveToFieldPose(AutoFunctions.redVsBlue(new Pose2d(4.8,1.75, new Rotation2d(Math.toRadians(130.0)))), 0.75,0.5, 5.0),
    new AimThenShootSpeaker(), 

    // move to note 6
    new AutoDriveToFieldPose(AutoFunctions.redVsBlue(new Pose2d(6.5,1.5, new Rotation2d(Math.toRadians(180.0)))), 0.75,0.5, 5.0),
    new AutoDriveToFieldPose(AutoFunctions.redVsBlue(new Pose2d(7.5,2.5, new Rotation2d(Math.toRadians(180.0)))), 0.75,0.5, 5.0),
    new SteerToNote(true, 2.0, 0.3),
    new AutoDriveToFieldPose(AutoFunctions.redVsBlue(new Pose2d(5.5,2.0, new Rotation2d(Math.toRadians(130.0)))), 0.75,0.5, 5.0),
    new AimThenShootSpeaker(),
    new CleanupShot()
   
    );
  }
}
