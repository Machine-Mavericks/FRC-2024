// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Autonomous;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.RobotContainer;
import frc.robot.commands.SemiAutonomous.AimThenShootSpeaker;
import frc.robot.commands.SemiAutonomous.AutoDriveOverLine;
import frc.robot.commands.SemiAutonomous.AutoDriveToPose;
import frc.robot.commands.SemiAutonomous.CleanupShot;
import frc.robot.commands.SemiAutonomous.DriveToRelativePose;
import frc.robot.commands.SemiAutonomous.TurnRobot;
import frc.robot.subsystems.Drivetrain;
import frc.robot.util.AutoFunctions;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class OneNoteAuto extends SequentialCommandGroup {
  /** Creates a new OneDonutAuto. */
  public OneNoteAuto() {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(

    new InstantCommand (()-> RobotContainer.gyro.resetGyroReverse()),
    
    // delay shot by time recorded in shuffleboard
    new DelayCommand(RobotContainer.operatorinterface.getAutoDelay()),

    // new TurnRobot(360, true, 3),
    // set gyro angle from odometry
    //new SetGyroUsingAprilTag(),

    // drive away from speaker
    // new DriveToRelativePose(new Pose2d(-1, 0.0, new Rotation2d(0)),
    //                         0.5, // speed
    //                         0.1, // rotational speed(unit?)
    //                         5.0),
    //new AutoDriveToPose(AutoFunctions.redVsBlue(new Pose2d()), 0.5*Drivetrain.MAX_VELOCITY_METERS_PER_SECOND, 0.5*Drivetrain.MAX_ANGULAR_VELOCITY_RADIANS_PER_SECOND, 3),

    new AimThenShootSpeaker(),

    new CleanupShot(),

    new AutoDriveOverLine(0.5, 0.5, 2.0)


        // drive away from speaker
    // new DriveToRelativePose(new Pose2d(-1, 0.0, new Rotation2d(0)),
    //                         0.5, // speed
    //                         0.1, // rotational speed(unit?)
    //                         5.0)
    // new AutoDriveToPose(AutoFunctions.redVsBlue(new Pose2d(3,2.1, new Rotation2d())), 0.5*Drivetrain.MAX_VELOCITY_METERS_PER_SECOND, 0.5*Drivetrain.MAX_ANGULAR_VELOCITY_RADIANS_PER_SECOND, 3)



    );
  }
}
